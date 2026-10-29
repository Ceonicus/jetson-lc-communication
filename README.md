# Jetson–Lift Controller communication (Phase 1)

This ROS 2 workspace implements only the Phase 1 Jetson–LC serial driver. It does not
schedule maintenance, control the robot, run vision, or implement the maintenance workflow.

## Packages

- `jetson_lc_interfaces`: status/configuration/health messages and atomic command services.
- `jetson_lc_protocol`: ROS-free C++ frame, CRC, parser, and protocol codecs.
- `jetson_lc_driver`: one normal ROS node using one Boost.Asio communication thread.
- `jetson_lc_tools`: a deterministic Linux pseudo-terminal mock LC.
- `tools/windows_mock_lc`: a standalone ROS-free Windows COM-port mock for physical RS485
  bench testing; see its [README](tools/windows_mock_lc/README.md).

## Design documents

The approved protocol, workflow, and implementation architecture are located under `docs/`:

- `docs/architecture.md`
- `docs/Jetson LC Communication Protocol V6 - 290726.pdf`
- `docs/Maintenance Workflow V5 - 290726.pdf`

The PDFs define the approved protocol and workflow behaviour. `architecture.md` defines the
Phase 1 implementation structure and boundaries.

For the physical Windows-mock bench after boot, see the [Jetson quick-start](docs/QUICKSTART.md).

## Dependencies

- ROS 2 with `ament_cmake`
- C++17 compiler
- Boost.Asio / Boost.System
- Linux pseudo-terminal support for the mock LC
- GTest for tests

Install missing package dependencies with:

```bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build and test

```bash
colcon build \
  --packages-select \
  jetson_lc_interfaces \
  jetson_lc_protocol \
  jetson_lc_driver \
  jetson_lc_tools \
  --symlink-install
source install/setup.bash
colcon test \
  --packages-select \
  jetson_lc_interfaces \
  jetson_lc_protocol \
  jetson_lc_driver \
  jetson_lc_tools
colcon test-result --verbose
```

## Serial device permissions

The configured user must have permission to access the serial device.

Check the device:

```bash
ls -l /dev/ttyUSB0
```

On systems using the `dialout` group:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in before testing again.

Do not run the ROS node as root merely to bypass serial permissions.

## Run

Edit `src/jetson_lc_driver/config/lc_driver.yaml` for the actual serial adapter and Lift ID,
then run:

```bash
source install/setup.bash
ros2 launch jetson_lc_driver lc_driver.launch.py
```

To run without hardware, use two terminals.

### Terminal 1: Start the mock LC

```bash
source install/setup.bash
ros2 run jetson_lc_tools mock_lc
```

The mock prints a slave PTY path, for example:

```text
/dev/pts/5
```

### Terminal 2: Start the driver

```bash
source install/setup.bash
ros2 run jetson_lc_driver lc_driver_node --ros-args \
  -p serial_device:=/dev/pts/5
```

Replace `/dev/pts/5` with the path printed by the mock.

Fault options are `--delay-ms N`, `--drop-every N`, and `--bad-crc-every N`. The mock
advances Start/Hold through preparation to Ready, advances request-clear through states
`0x07`–`0x09`, and reports `0x0B` after abort. A quick serial smoke test is available as:

```bash
ros2 run jetson_lc_tools mock_lc --self-test
```

Inspect topics:

```bash
ros2 topic echo /lc/status
ros2 topic echo /lc/distance
ros2 topic echo /lc/configuration
ros2 topic echo /lc/communication_status
```

Queue Start/Hold (the response means queued, not completed):

```bash
ros2 service call /lc/set_command_state \
  jetson_lc_interfaces/srv/SetLcCommandState \
  "{destination_floor: 0, hall_call_disable: true, manual_motion: 0, maintenance_request: 1}"
```

Queue request-clear/completion:

```bash
ros2 service call /lc/set_command_state \
  jetson_lc_interfaces/srv/SetLcCommandState \
  "{destination_floor: 0, hall_call_disable: false, manual_motion: 0, maintenance_request: 0}"
```

Queue abort, or request a fresh `0x10` sample:

```bash
ros2 service call /lc/set_command_state \
  jetson_lc_interfaces/srv/SetLcCommandState \
  "{destination_floor: 0, hall_call_disable: true, manual_motion: 0, maintenance_request: 2}"
ros2 service call /lc/refresh_configuration \
  jetson_lc_interfaces/srv/RefreshLcConfiguration "{}"
```

Manual Up/Down is rejected unless the latest status is fresh and reports session `0x06`,
Jetson operation permitted, Robot Maintenance Mode, and no Lift Abnormal Mode. If those
conditions or communication are lost while motion is active, the desired command is changed
to Stop and the full stop state receives highest retry priority. Safety is checked again on the
communication thread before applying a queued motion, so a service request accepted just
before status changes may be preempted by Stop. Receipt is never fabricated.

## Convenience scripts

Run the mock and driver in separate terminals:

```bash
# Terminal 1
./scripts/start_mock_lc.sh
```

```bash
# Terminal 2
./scripts/start_driver.sh /dev/pts/3
```

Replace `/dev/pts/3` with the path printed by the mock. Run command scripts from a third
terminal; service acceptance means the command was queued, not that LC operation completed:

```bash
./scripts/start_hold.sh
./scripts/manual_down.sh
./scripts/manual_stop.sh
./scripts/complete_maintenance.sh
./scripts/abort_maintenance.sh
./scripts/refresh_configuration.sh
```

Topic inspection is still performed manually, for example:

```bash
ros2 topic echo /lc/status
ros2 topic echo /lc/communication_status
```

## Current assumptions and integration restrictions

- CRC is fixed to CRC-16/MODBUS (`0xFFFF`, reflected `0xA001`, no final XOR), covering
  Data Length through Data Portion, transmitted low byte first. ENV still needs to provide a
  known-good frame for final confirmation.
- The approved 100 ms reply timeout is enforced. All other periods remain YAML parameters.
  The command retry period must remain greater than the 100 ms response timeout. This prevents
  immediate retries from monopolising the serial transaction schedule and allows `0x11` status
  polling between command attempts.
- Once request-clear begins completion, the approved workflow takes precedence over the
  architecture's general poll example: hold and `0x10` traffic are suppressed and only `0x11`
  is polled until state `0x09`.
- Destination floor values other than zero are rejected. The protocol does not define the
  usable floor range or the destination MSB/second-door selector precisely enough to expose
  it safely, and Phase 1 maintenance commands use zero.
- Serial adapter RS485 direction control, inter-frame turnaround, whether every periodic hold
  is ACKed, and LC-side communication-loss/manual-motion limits require real-hardware
  confirmation. No Linux-specific direction-control ioctl is enabled.
- The service-floor bit array is published as the 16 protocol bytes in wire order; bit 0 of
  each byte is its LSB as specified by the approved protocol.
