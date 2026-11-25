#include "encos_core/socketcan_transport.hpp"

#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>
#include <linux/rtnetlink.h>
#include <net/if.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <mutex>
#include <system_error>

namespace encos {
namespace {

class Fd {
 public:
  explicit Fd(int fd) : value(fd) {}
  ~Fd() { if (value >= 0) ::close(value); }
  Fd(const Fd&) = delete;
  Fd& operator=(const Fd&) = delete;
  const int value;
};

[[noreturn]] void fail(const char* context, int error = errno) {
  throw std::system_error(error, std::generic_category(), context);
}

int open_socket(int family, int type, int protocol) {
  const int fd = ::socket(family, type, protocol);
  if (fd < 0) {
    fail("socket");
  }
  return fd;
}

template<class T>
void option(int fd, int level, int name, const T& value) {
  if (::setsockopt(fd, level, name, &value, sizeof(value)) != 0) {
    fail("SocketCAN setsockopt");
  }
}

std::string link_kind(const ifinfomsg* message, int length) {
  for (auto* outer = IFLA_RTA(message); RTA_OK(outer, length); outer = RTA_NEXT(outer, length)) {
    if ((outer->rta_type & NLA_TYPE_MASK) != IFLA_LINKINFO) {
      continue;
    }
    int nested_length = RTA_PAYLOAD(outer);
    for (auto* inner = static_cast<const rtattr*>(RTA_DATA(outer)); RTA_OK(inner, nested_length);
         inner = RTA_NEXT(inner, nested_length)) {
      if ((inner->rta_type & NLA_TYPE_MASK) == IFLA_INFO_KIND && RTA_PAYLOAD(inner) > 0) {
        const auto* kind = static_cast<const char*>(RTA_DATA(inner));
        return {kind, ::strnlen(kind, RTA_PAYLOAD(inner))};
      }
    }
  }
  return {};
}

struct Link { unsigned flags{}; unsigned mtu{}; std::string kind; };

Link inspect_link(unsigned index) {
  const Fd netlink(open_socket(AF_NETLINK, SOCK_RAW | SOCK_CLOEXEC, NETLINK_ROUTE));
  option(netlink.value, SOL_SOCKET, SO_RCVTIMEO, timeval{0, 200000});
  struct Request { nlmsghdr header; ifinfomsg info; };
  Request request{};
  request.header.nlmsg_len = NLMSG_LENGTH(sizeof(ifinfomsg));
  request.header.nlmsg_type = RTM_GETLINK;
  request.header.nlmsg_flags = NLM_F_REQUEST;
  request.header.nlmsg_seq = 1;
  request.info.ifi_family = AF_UNSPEC;
  request.info.ifi_index = static_cast<int>(index);
  sockaddr_nl kernel{};
  kernel.nl_family = AF_NETLINK;
  if (::sendto(netlink.value, &request, request.header.nlmsg_len, 0,
               reinterpret_cast<sockaddr*>(&kernel), sizeof(kernel)) < 0) {
    fail("RTM_GETLINK send");
  }
  alignas(nlmsghdr) std::array<std::uint8_t, 16384> buffer{};
  sockaddr_nl peer{};
  socklen_t peer_size = sizeof(peer);
  const auto bytes = ::recvfrom(netlink.value, buffer.data(), buffer.size(), 0,
                                reinterpret_cast<sockaddr*>(&peer), &peer_size);
  if (bytes < 0) {
    fail("RTM_GETLINK receive");
  }
  if (peer.nl_pid != 0) {
    fail("RTM_GETLINK invalid sender", EPROTO);
  }
  int remaining = static_cast<int>(bytes);
  for (auto* header = reinterpret_cast<nlmsghdr*>(buffer.data()); NLMSG_OK(header, remaining);
       header = NLMSG_NEXT(header, remaining)) {
    if (header->nlmsg_seq != 1) {
      continue;
    }
    if (header->nlmsg_type == NLMSG_ERROR && header->nlmsg_len >= NLMSG_LENGTH(sizeof(nlmsgerr))) {
      fail("RTM_GETLINK", -static_cast<const nlmsgerr*>(NLMSG_DATA(header))->error);
    }
    if (header->nlmsg_type != RTM_NEWLINK || header->nlmsg_len < NLMSG_LENGTH(sizeof(ifinfomsg))) {
      continue;
    }
    const auto* info = static_cast<const ifinfomsg*>(NLMSG_DATA(header));
    if (info->ifi_index != static_cast<int>(index)) {
      continue;
    }
    Link link{info->ifi_flags, 0, link_kind(info, IFLA_PAYLOAD(header))};
    int length = IFLA_PAYLOAD(header);
    for (auto* attr = IFLA_RTA(info); RTA_OK(attr, length); attr = RTA_NEXT(attr, length)) {
      if (attr->rta_type == IFLA_MTU && RTA_PAYLOAD(attr) == sizeof(link.mtu)) {
        std::memcpy(&link.mtu, RTA_DATA(attr), sizeof(link.mtu));
      }
    }
    return link;
  }
  fail("RTM_GETLINK no matching interface", ENODEV);
}

Link require_virtual(unsigned index, WireFormat format) {
  const auto link = inspect_link(index);
  if (link.kind != "vcan") {
    fail("P1 SocketCAN permits vcan interfaces only", EPERM);
  }
  if (!(link.flags & IFF_UP)) {
    fail("vcan interface is down", ENETDOWN);
  }
  if (format == WireFormat::fd_brs && link.mtu < CANFD_MTU) {
    fail("vcan MTU does not support CAN FD", EMSGSIZE);
  }
  return link;
}

bool link_error(int error) {
  return error == ENETDOWN || error == ENODEV || error == ENXIO || error == ENOTCONN;
}

int link_liveness(int fd, unsigned index) {
  ifreq request{};
  request.ifr_ifindex = static_cast<int>(index);
  if (::ioctl(fd, SIOCGIFNAME, &request) != 0 || ::ioctl(fd, SIOCGIFFLAGS, &request) != 0) {
    return errno;
  }
  return (request.ifr_flags & IFF_UP) ? 0 : ENETDOWN;
}

bool fd_length(unsigned length) {
  return length <= 8 || length == 12 || length == 16 || length == 20 ||
         length == 24 || length == 32 || length == 48 || length == 64;
}

std::chrono::nanoseconds realtime_now() {
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
      std::chrono::system_clock::now().time_since_epoch());
}

std::optional<std::chrono::nanoseconds> wall_offset() {
  // Discard a clock pair if the thread was descheduled between its samples.
  for (unsigned attempt = 0; attempt < 6; ++attempt) {
    const auto before = Clock::now();
    const auto wall = realtime_now();
    const auto after = Clock::now();
    if (after - before <= std::chrono::microseconds(5)) {
      return wall - std::chrono::duration_cast<std::chrono::nanoseconds>(before.time_since_epoch());
    }
  }
  return {};
}

SocketReceive decode_kernel(const canfd_frame& raw, ssize_t size, msghdr& msg,
                            std::chrono::nanoseconds initial_offset) {
  if ((size != CAN_MTU && size != CANFD_MTU) || (msg.msg_flags & MSG_TRUNC)) {
    return {ReceiveStatus::io_error, {}, EPROTO};
  }
  CanFrame frame;
  frame.extended = raw.can_id & CAN_EFF_FLAG;
  frame.rtr = raw.can_id & CAN_RTR_FLAG;
  frame.error = raw.can_id & CAN_ERR_FLAG;
  frame.id = raw.can_id & (frame.error ? CAN_ERR_MASK : (frame.extended ? CAN_EFF_MASK : CAN_SFF_MASK));
  frame.kind = size == CANFD_MTU ? FrameKind::fd : FrameKind::classic;
  frame.length = raw.len;
  if (frame.kind == FrameKind::fd) {
    frame.brs = raw.flags & CANFD_BRS;
    frame.esi = raw.flags & CANFD_ESI;
    if (!fd_length(frame.length)) {
      return {ReceiveStatus::io_error, {}, EPROTO};
    }
  } else if (frame.length > CAN_MAX_DLEN) {
    return {ReceiveStatus::io_error, {}, EPROTO};
  }
  std::copy_n(raw.data, frame.length, frame.data.begin());
  const bool local_echo = msg.msg_flags & MSG_CONFIRM;
  ReceivedFrame sample{frame, Clock::now(), local_echo ? Origin::local_echo : Origin::unknown, {}, {}};
  sample.userspace_stamp = sample.stamp;
  for (auto* control = CMSG_FIRSTHDR(&msg); control; control = CMSG_NXTHDR(&msg, control)) {
    if (control->cmsg_level != SOL_SOCKET) {
      continue;
    }
    if (control->cmsg_type == SCM_TIMESTAMPNS && control->cmsg_len >= CMSG_LEN(sizeof(timespec))) {
      timespec stamp{};
      std::memcpy(&stamp, CMSG_DATA(control), sizeof(stamp));
      sample.kernel_realtime = std::chrono::seconds(stamp.tv_sec) + std::chrono::nanoseconds(stamp.tv_nsec);
    }
    if (control->cmsg_type == SO_RXQ_OVFL && control->cmsg_len >= CMSG_LEN(sizeof(std::uint32_t))) {
      std::uint32_t dropped{};
      std::memcpy(&dropped, CMSG_DATA(control), sizeof(dropped));
      sample.socket_dropped = dropped;
    }
  }
  if (sample.kernel_realtime && !(msg.msg_flags & MSG_CTRUNC)) {
    const auto current_offset = wall_offset();
    if (current_offset && *current_offset >= initial_offset - std::chrono::microseconds(100) &&
        *current_offset <= initial_offset + std::chrono::microseconds(100)) {
      // Use the larger offset so uncertainty cannot make an old receive timestamp newer.
      const auto estimate = *sample.kernel_realtime - std::max(initial_offset, *current_offset);
      sample.stamp = std::min(sample.stamp, TimePoint{std::chrono::duration_cast<Clock::duration>(estimate)});
      if (!local_echo) {
        sample.origin = Origin::remote;  // Logical peer on this vcan, including localhost.
      }
    }
  }
  return {ReceiveStatus::frame, sample};
}

}  // namespace

struct SocketCanTransport::Impl {
  explicit Impl(const SocketCanOptions& options)
      : index(::if_nametoindex(options.interface.c_str())), format(options.format),
        socket(open_socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW)),
        wake(::eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC)), offset(wall_offset()) {
    if (index == 0) {
      fail("CAN interface not found", ENODEV);
    }
    if (wake.value < 0) {
      fail("eventfd");
    }
    if (!offset) {
      fail("cannot pair SocketCAN and monotonic clocks", EAGAIN);
    }
    if (!valid_wire_format(format)) {
      throw std::invalid_argument("invalid SocketCAN format");
    }
    require_virtual(index, format);
    option(socket.value, SOL_CAN_RAW, CAN_RAW_LOOPBACK, 1);
    option(socket.value, SOL_CAN_RAW, CAN_RAW_RECV_OWN_MSGS, 0);
    option(socket.value, SOL_SOCKET, SO_TIMESTAMPNS, 1);
    option(socket.value, SOL_SOCKET, SO_RXQ_OVFL, 1);
    if (format == WireFormat::fd_brs) {
      option(socket.value, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, 1);
    }
    const can_err_mask_t mask = options.error_frames ? CAN_ERR_MASK : 0;
    option(socket.value, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, mask);

    std::vector<can_filter> filters;
    for (const auto id : options.ids) {
      if (id > CAN_SFF_MASK) {
        throw std::invalid_argument("filter ID is not a standard CAN ID");
      }
      filters.push_back({id, CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG});
    }
    if (filters.empty()) {
      filters.push_back({0, CAN_EFF_FLAG | CAN_RTR_FLAG});
    }
    if (::setsockopt(socket.value, SOL_CAN_RAW, CAN_RAW_FILTER, filters.data(),
                     static_cast<socklen_t>(filters.size() * sizeof(can_filter))) != 0) {
      fail("CAN_RAW_FILTER");
    }
    sockaddr_can address{};
    address.can_family = AF_CAN;
    address.can_ifindex = static_cast<int>(index);
    if (::bind(socket.value, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
      fail("SocketCAN bind");
    }
    require_virtual(index, format);
  }

  const unsigned index;
  const WireFormat format;
  const Fd socket, wake;
  const std::optional<std::chrono::nanoseconds> offset;
  std::mutex rx_mutex;
};

SocketCanTransport::SocketCanTransport(const SocketCanOptions& options) : impl_(std::make_unique<Impl>(options)) {}
SocketCanTransport::~SocketCanTransport() = default;

SocketSend SocketCanTransport::send_with_status(const CanFrame& frame) {
  if (!matches_wire_format(frame, impl_->format) || frame.id > CAN_SFF_MASK || frame.error ||
      frame.rtr || frame.extended || frame.esi ||
      (frame.kind == FrameKind::classic ? frame.length > CAN_MAX_DLEN : !fd_length(frame.length))) {
    return {SendStatus::invalid_frame, EINVAL};
  }
  canfd_frame raw{};
  raw.can_id = frame.id;
  raw.len = frame.length;
  if (frame.brs) {
    raw.flags = CANFD_BRS | CANFD_FDF;
  }
  std::copy_n(frame.data.begin(), frame.length, raw.data);
  const size_t size = frame.kind == FrameKind::classic ? CAN_MTU : CANFD_MTU;
  const auto written = ::send(impl_->socket.value, &raw, size, MSG_DONTWAIT);
  if (written == static_cast<ssize_t>(size)) {
    return {SendStatus::accepted};
  }
  const int error = written < 0 ? errno : EIO;
  if (error == EAGAIN || error == ENOBUFS) {
    return {SendStatus::full, error};
  }
  return {link_error(error) ? SendStatus::disconnected : SendStatus::io_error, error};
}

SendStatus SocketCanTransport::send(const CanFrame& frame) { return send_with_status(frame).status; }

SocketReceive SocketCanTransport::receive_for(Clock::duration wait) {
  if (wait.count() < 0 || wait > std::chrono::seconds(60)) {
    return {ReceiveStatus::io_error, {}, EINVAL};
  }
  std::unique_lock<std::mutex> lock(impl_->rx_mutex, std::try_to_lock);
  if (!lock) {
    return {ReceiveStatus::busy, {}, EBUSY};
  }
  pollfd fds[]{{impl_->socket.value, POLLIN, 0}, {impl_->wake.value, POLLIN, 0}};
  const auto seconds = std::chrono::duration_cast<std::chrono::seconds>(wait);
  const auto remainder = std::chrono::duration_cast<std::chrono::nanoseconds>(wait - seconds);
  const timespec timeout{seconds.count(), remainder.count()};
  const int ready = ::ppoll(fds, 2, &timeout, nullptr);
  if (ready < 0) {
    const int error = errno;
    return {error == EINTR ? ReceiveStatus::interrupted : ReceiveStatus::io_error, {}, error};
  }
  if (ready == 0) {
    const auto error = link_liveness(impl_->socket.value, impl_->index);
    if (error != 0) {
      return {link_error(error) ? ReceiveStatus::disconnected : ReceiveStatus::io_error, {}, error};
    }
    return {ReceiveStatus::timeout, {}};
  }
  if (fds[1].revents & POLLIN) {
    std::uint64_t counter{};
    const auto discarded = ::read(impl_->wake.value, &counter, sizeof(counter));
    (void)discarded;
    return {ReceiveStatus::interrupted, {}};
  }
  if (fds[0].revents & (POLLERR | POLLHUP | POLLNVAL)) {
    int error{};
    socklen_t length = sizeof(error);
    if (::getsockopt(impl_->socket.value, SOL_SOCKET, SO_ERROR, &error, &length) != 0) {
      error = errno;
    }
    if ((fds[0].revents & (POLLHUP | POLLNVAL)) || link_error(error)) {
      return {ReceiveStatus::disconnected, {}, error ? error : ENOTCONN};
    }
    return {ReceiveStatus::io_error, {}, error ? error : EIO};
  }
  canfd_frame raw{};
  alignas(cmsghdr) char ancillary[CMSG_SPACE(sizeof(timespec)) + CMSG_SPACE(sizeof(std::uint32_t))]{};
  iovec payload{&raw, sizeof(raw)};
  msghdr msg{};
  msg.msg_iov = &payload;
  msg.msg_iovlen = 1;
  msg.msg_control = ancillary;
  msg.msg_controllen = sizeof(ancillary);
  const auto size = ::recvmsg(impl_->socket.value, &msg, MSG_DONTWAIT);
  if (size < 0) {
    const int error = errno;
    if (error == EAGAIN) {
      return {ReceiveStatus::timeout, {}};
    }
    if (error == EINTR) {
      return {ReceiveStatus::interrupted, {}, error};
    }
    return {link_error(error) ? ReceiveStatus::disconnected : ReceiveStatus::io_error, {}, error};
  }
  return decode_kernel(raw, size, msg, *impl_->offset);
}

std::optional<ReceivedFrame> SocketCanTransport::receive() {
  return receive_for(std::chrono::milliseconds::zero()).sample;
}

void SocketCanTransport::interrupt() noexcept {
  const std::uint64_t one = 1;
  const auto discarded = ::write(impl_->wake.value, &one, sizeof(one));
  (void)discarded;
}

bool SocketCanTransport::connected() const {
  try {
    require_virtual(impl_->index, impl_->format);
    return true;
  } catch (const std::system_error&) {
    return false;
  }
}

}  // namespace encos
