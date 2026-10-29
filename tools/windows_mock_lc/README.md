# Standalone Windows serial mock LC

`mock_lc_serial.exe` is a ROS-free, responder-only Lift Controller mock for an isolated
Jetson-to-Windows RS485 bench. It uses the native Windows COM API and compiles the existing
`jetson_lc_protocol` codec/parser and `jetson_lc_tools` deterministic mock logic directly from
this repository.

Do not connect this tool to an operational lift or production controller. It is only for an
isolated bench connection between two USB-RS485 adapters. The mock never initiates commands.

## Build on Windows

Use a Visual Studio 2022 Developer PowerShell with CMake installed. From the repository root:

```powershell
cmake -S tools/windows_mock_lc -B tools/windows_mock_lc/build -A x64
cmake --build tools/windows_mock_lc/build --config Release
```

No ROS or Boost installation is required. The executable is written to approximately:

```text
tools\windows_mock_lc\build\Release\mock_lc_serial.exe
```

The tool must be built inside the repository because its standalone CMake target reuses the
existing protocol and mock-state source files rather than copying protocol logic.

## Physical bench setup and run

1. Plug a USB-RS485 adapter into the Windows laptop.
2. Find its COM port in Device Manager, for example `COM5`.
3. Connect the physical RS485 lines to the Jetson-side adapter: A to A, B to B, and GND to
   GND where the adapter manufacturers support or recommend it. If there is no communication,
   check the adapter labels because some vendors label A/B polarity oppositely.
4. Start the Windows mock:

   ```powershell
   .\tools\windows_mock_lc\build\Release\mock_lc_serial.exe COM5
   ```

5. On the Jetson, locate its adapter and start the unchanged real driver:

   ```bash
   ls /dev/ttyUSB*
   ./scripts/start_driver.sh /dev/ttyUSB0
   ```

6. On the Jetson, monitor communication and LC state:

   ```bash
   ros2 topic echo /lc/communication_status
   ros2 topic echo /lc/status
   ```

7. Exercise the command state:

   ```bash
   ./scripts/start_hold.sh
   ./scripts/manual_down.sh
   ./scripts/manual_stop.sh
   ./scripts/complete_maintenance.sh
   ./scripts/abort_maintenance.sh
   ```

8. Verify that the Windows console shows each RX/TX frame and fully decodes the corresponding
   `0x12` command state.

The serial port is configured for 38400 baud, 8 data bits, no parity, one stop bit, and no flow
control. `COM10` and higher are accepted; the program converts all port arguments to the
Windows `\\.\COMn` device form internally. The adapter is expected to handle RS485 direction
automatically. The program does not toggle RTS for direction control.

## Options

```powershell
mock_lc_serial.exe COM5 [--baud 38400] [--delay-ms N] [--drop-every N] [--bad-crc-every N]
```

- `--delay-ms N`: wait before each response.
- `--drop-every N`: do not respond to every Nth valid request.
- `--bad-crc-every N`: corrupt the CRC of every Nth response.

All fault modes default to disabled. Request counting and fault timing match the Linux PTY
mock: only complete, valid parsed requests are counted.

## Emulated behaviour

- Lift ID is `1`.
- `0x10` returns the existing deterministic 32-byte configuration: 900 kg capacity, 1600 mm
  car width, 1400 mm distance, 1000 mm door width, no second door, and the existing service
  floor bytes.
- `0x11` returns floor 16 and the existing deterministic status/session transitions. Reserved
  door-status bits remain OFF.
- `0x12` prints all command bits, accepts and ACKs Start/Hold, request-clear, and abort, and
  treats periodic Start/Hold frames as one held active request.
- Start/Hold progresses through `0x01`, `0x02`, `0x04`, `0x05`, and `0x06`. At `0x06`, Jetson
  Operation Permitted and Robot Maintenance Mode are ON and Lift Abnormal Mode is OFF.
- Request-clear progresses through `0x07`, `0x08`, and `0x09`. Abort reports `0x0B`.
- The shared parser tolerates partial reads, multiple frames per read, noise before the header,
  invalid CRC, malformed Data Length, and subsequent resynchronisation.
