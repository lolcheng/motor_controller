"""Check transmitted bytes against manual vectors on a Linux pseudo-terminal."""

from contextlib import contextmanager
import os
import pty
import select
import subprocess
import sys
import time


def run(*args):
    return subprocess.run([sys.argv[1], *args], capture_output=True, text=True, timeout=3)


@contextmanager
def terminal():
    master, slave = pty.openpty()
    try:
        yield master, os.ttyname(slave)
    finally:
        os.close(slave)
        os.close(master)


def read_packet(master, size):
    received = bytearray()
    deadline = time.monotonic() + 2
    while len(received) < size:
        assert select.select([master], [], [], max(0, deadline - time.monotonic()))[0]
        received.extend(os.read(master, size - len(received)))
    return received


def serial_packet(frame):
    # Build the wrapper independently from the literal CAN vector.
    arb_id, hex_data = frame.split('#')
    data = bytes.fromhex(hex_data)
    packet = b'\xaa' + int(arb_id, 16).to_bytes(2, 'big') + bytes([len(data) + 5]) + data
    return packet + bytes([sum(packet) & 0xff])


def send(master, device, command, frame, *details):
    result = run(device, *command.split())
    assert result.returncode == 0, (command, result.stderr)
    for expected in (f'TX CAN {frame} ', *details):
        assert expected in result.stdout, (command, expected, result.stdout)
    packet = serial_packet(frame)
    received = read_packet(master, len(packet))
    assert received == packet, (command, received.hex(), packet.hex())
    assert not select.select([master], [], [], 0)[0], command


def rejects(command, message):
    result = run(*command.split())
    assert result.returncode != 0 and message in result.stderr, (command, result)
    assert 'TX CAN ' not in result.stdout and 'SERIAL ' not in result.stdout, result.stdout


def test_command_vectors(master, device):
    def transmit(command, frame, *details):
        send(master, device, command, frame, *details)

    # §7–§9.1 and §11. The 30-degree vector is derived from §9.1.2's field
    # positions, float32 0x41f00000, and the CLI's 20 rpm / 1 A / ACK 2 defaults.
    transmit('DISCOVER', '7FF#FFFF0082', 'SERIAL AA 07 FF 09 FF FF 00 82 39')
    transmit('SET_ID 1 2', '7FF#000100040002')
    transmit('RESET_ID', '7FF#7F7F00057F7F')
    transmit('ZERO 1', '7FF#00010003')
    transmit('ZERO 2046 -10', '7FF#07FE0003FC18')
    transmit('POS 1 30', '001#283E00000032002A', 'DLC=8',
            'SERIAL AA 00 01 0D 28 3E 00 00 00 32 00 2A 7A', 'telemetry type=2')
    transmit('POS 1 0 50 10 1', '001#20000000007D0191')
    transmit('POS 1 -90 20 5 2', '001#38568000003200CA')
    transmit('VEL 1 50 10', '001#41424800000064')
    transmit('VEL 1 -60 5 3', '001#43C27000000032', 'DLC=7', 'telemetry type=3')
    transmit('CURRENT 1 5', '001#6101F4')
    transmit('TORQUE 1 -5 2', '001#66FE0C')
    transmit('DAMPING 1', '001#690000')
    transmit('DAMPING 1 0', '001#680000', 'EXPECTED_REPLY none')
    transmit('DISSIPATIVE 1 5', '001#6D01F4')
    transmit('REGENERATIVE 1 5', '001#7101F4')
    transmit('BRAKE 1 RELEASE', '001#8001', 'brake prefix=B2')
    transmit('BRAKE 1 ENGAGE', '001#8000')
    transmit('ELECTROMAGNETIC_BRAKE 1 RELEASE', '001#750001', 'brake type=6')
    transmit('ELECTROMAGNETIC_BRAKE 1 ENGAGE', '001#750000')
    transmit('MIXED 1 20 5 0 0 0 0 500 0 5 -12.5 12.5 -18 18 -30 30', '001#0147FF7FFF7FF7FF')

    names = {
        'POSITION': 1, 'VELOCITY': 2, 'PHASE_CURRENT': 3, 'POWER': 4, 'ACCELERATION': 5,
        'KT': 22, 'KP_RANGE': 23, 'KD_RANGE': 24, 'POSITION_RANGE': 25, 'VELOCITY_RANGE': 26,
        'TORQUE_RANGE': 27, 'CURRENT_RANGE': 28, 'UUID': 29, 'VERSION': 30, 'TIMEOUT': 31,
        'CURRENT_PI': 32, 'VELOCITY_PI': 33, 'POSITION_PD': 34, 'KT_CALIBRATION': 35,
        'BRAKE': 37, 'ZERO_OFFSET': 38, 'ENCODER': 39,
    }
    for name, code in names.items():
        transmit(f'QUERY 1 {name}', f'001#E1{code:02X}')
    transmit('QUERY 1 30', '001#E11E')
    transmit('QUERY 1 KT_SEGMENT 10', '001#EA24', 'Kt group=10')

    # §9.2. Position range uses the signed field definition, correcting the
    # printed example's negative endpoint as documented in docs/issues.md.
    configurations = {
        'ACCELERATION 20': 'C10107D0',  # §9.2.1 uses ACK=1 for the documented echo.
        'COMMUNICATION 1': 'C10201', 'KT 2.5': 'C00400FA', 'KP_RANGE 0 500': 'C005000001F4',
        'KD_RANGE 0 50': 'C00600000032', 'POSITION_RANGE -12.5 12.5': 'C007FB1E04E2',
        'VELOCITY_RANGE -18 18': 'C008F8F80708', 'TORQUE_RANGE -90 90': 'C009FC7C0384',
        'CURRENT_RANGE -90 90': 'C00AFC7C0384', 'TIMEOUT 500': 'C00B01F4',
        'CURRENT_PI 0.01 50': 'C00C006401F4', 'VELOCITY_PI 0.006 0.1': 'C00D02582710',
        'POSITION_PD 0.006 0.00004': 'C00E02580004', 'KT_CALIBRATION 2': 'C00F02',
        'KT_SEGMENT 1 2.5 2.6 2.7 2.8': 'C1100FA10410E118', 'ZERO_OFFSET -10': 'C011FC18',
    }
    for arguments, payload in configurations.items():
        transmit(f'CONFIG 1 {arguments}', f'001#{payload}')
    transmit('CONFIG 1 4 2.5', '001#C00400FA')

    rejects(f'{device} ENABLE 1', 'independent CAN enable frame')
    rejects(f'{device} POS 1 NaN', 'invalid finite number')
    rejects(f'{device} POS 1 1e9999', 'invalid finite number')
    rejects(f'{device} POS 1 30deg', 'invalid finite number')
    rejects(f'{device} POS 1 30 20 1 4', 'invalid decimal argument')
    rejects(f'{device} POS 2047 30', 'invalid decimal argument')
    rejects(f'{device} VEL 1 60', 'wrong number')
    rejects(f'{device} MIXED 1 20 5 0 0 0', 'wrong number')
    rejects(f'{device} QUERY 1 KT_SEGMENT', 'invalid query group')
    rejects(f'{device} QUERY 1 6', 'reserved query code')
    rejects(f'{device} CONFIG 1 ACCELERATION 20.01', 'fixed-point field out of range')
    rejects(f'{device} CONFIG 1 ACCELERATION -0.01', 'fixed-point field out of range')
    rejects(f'{device} CONFIG 1 ACCELERATION 20 0', 'wrong number of command arguments')
    rejects(f'{device} CONFIG 1 3 10', 'reserved configuration code')
    rejects(f'{device} CONFIG 1 KT 2 5', 'wrong number')

    rejects(f'{device} --fd POS 1 30', 'unknown ENCOS command')
    rejects(f'{device}/missing POS 1 30', 'open USB-CAN serial device')
    rejects(f'{device} preview', 'unknown ENCOS command')
    assert not select.select([master], [], [], 0)[0]


def test_serial():
    with terminal() as (master, device):
        rejects(f'{device} POS 1 30 --allow-query', 'invalid USB-CAN command')
        rejects(f'{device} CONFIG 1 TIMEOUT 500 --allow-query', 'invalid USB-CAN command')
        assert not select.select([master], [], [], 0)[0]

        with subprocess.Popen([sys.argv[1], device, 'QUERY', '1', 'POSITION', '--allow-query'],
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True) as process:
            expected = bytes.fromhex('AA 00 01 07 E1 01 94')
            try:
                request = read_packet(master, len(expected))
                assert request == expected, request.hex()
                os.write(master, bytes.fromhex('AA 00 01 0B A0 01 42 B4 00 00 4D'))
                stdout, stderr = process.communicate(timeout=2)
                assert process.returncode == 0 and 'RX 001#A00142B40000 query=1' in stdout, (stdout, stderr)
            finally:
                if process.poll() is None:
                    process.kill()


def test_console():
    with terminal() as (master, device), subprocess.Popen(
        [sys.argv[1], device], stdin=subprocess.PIPE,
        stdout=subprocess.PIPE, stderr=subprocess.PIPE,
    ) as process:
        output = bytearray()

        def wait_output(expected):
            deadline = time.monotonic() + 2
            while expected not in output:
                assert select.select([process.stdout], [], [], max(0, deadline - time.monotonic()))[0], output
                chunk = os.read(process.stdout.fileno(), 4096)
                assert chunk, output
                output.extend(chunk)
            del output[:output.index(expected) + len(expected)]

        try:
            wait_output(b'(HELP, EXIT)\n')
            assert not select.select([master], [], [], .03)[0]

            # Receive valid Type 2 feedback with no terminal command or query outstanding.
            feedback = bytes.fromhex('AA 00 01 0D 40 00 00 00 00 00 00 64 5C')
            os.write(master, feedback + feedback)
            for _ in range(2):
                wait_output(b'RX 001#4000000000000064 type=2 fault=0')

            process.stdin.write(b'POS 1 30\n')
            process.stdin.flush()
            expected = bytes.fromhex('AA 00 01 0D 28 3E 00 00 00 32 00 2A 7A')
            assert read_packet(master, len(expected)) == expected
            wait_output(b'TX CAN 001#283E00000032002A DLC=8')
            os.write(master, expected + feedback)
            wait_output(b'RX 001#283E00000032002A possible_tx_echo')
            wait_output(b'RX 001#4000000000000064 type=2 fault=0')

            # The process still reads before the next input and idling never re-sends POS.
            assert not select.select([master], [], [], .1)[0]
            os.write(master, feedback)
            wait_output(b'RX 001#4000000000000064 type=2 fault=0')

            replies = {
                '7FF#FFFF010001': 'management=discover status=success motor_id=1',
                '7FF#80800180': 'management=discover status=failure',
                '7FF#00010103': 'management=zero status=success motor_id=1',
                '7FF#00020104': 'management=set_id status=success motor_id=2',
                '7FF#7F7F01057F7F': 'management=reset_id status=success motor_id=1',
                '7FF#00010100': 'management=write_failed status=failure motor_id=1',
                '001#207FFF7FF7FF6464': 'type=1 fault=0 motor_c=25 mos_c=25 quantized=32767,2047,2047',
                '001#4041F00000000064': 'type=2 fault=0 position_rad=0.523599 phase_current_a=0 motor_c=25',
                '001#6042480000FF9C64': 'type=3 fault=0 velocity_rad_s=5.23599 phase_current_a=-1 motor_c=25',
                '001#800101': 'type=4 config=1 status=success fault=0',
                '001#830100': 'type=4 config=1 status=failure fault=3',
                '001#A01E010203040506': 'query=30 fault=0 hardware= 1 2 3 software= 4 5 6',
                '001#A01D0308090A0B': 'query=29 fault=0 uuid_part=3 uuid_bytes=08090A0B',
                '001#A02701DCD82328': 'query=39 fault=0 encoder_calibrated=1 encoder_counts=-9000,9000',
                '001#C000': 'type=6 brake=engaged fault=0',
                '001#B201': 'brake_prefix=B2 brake=released',
                '001#FFFE0107D0': 'config_echo=1 value=20',
                '001#FFFE0201': 'undecoded reason=ambiguous',
                '001#EE010FA10410E118': 'Kt_header=1 Kt_group=1 Kt_values= 2.5 2.6 2.7 2.8',
                '001#EE120FA10410E118': 'Kt_header=18 Kt_group=unknown Kt_values= 2.5 2.6 2.7 2.8',
            }
            for frame, fields in replies.items():
                os.write(master, serial_packet(frame))
                wait_output(f'RX {frame} {fields}\n'.encode())

            process.stdin.write(b'EXIT\n')
            process.stdin.flush()
            assert process.wait(timeout=3) == 0
        finally:
            if process.poll() is None:
                process.kill()


if __name__ == '__main__':
    with terminal() as (master, device):
        test_command_vectors(master, device)
    test_serial()
    test_console()
