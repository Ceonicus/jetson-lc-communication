# Jetson ↔ Lift Controller bench quick-start

Assumes the workspace is already built (`install/setup.bash` exists). Physical
serial settings: **38400 baud, 8N1**. Keep the approved **100 ms response deadline**
unchanged. Reserved door command bits must remain unused.

## 1. After boot — verify serial adapter

```bash
uname -r
lsusb
lsmod | grep -E 'pl2303|usbserial'
ls -l /dev/ttyUSB0
groups
test -r /dev/ttyUSB0 && echo "read OK"
test -w /dev/ttyUSB0 && echo "write OK"
```

Healthy: current kernel `5.15.199-tegra`; ATEN UC-232A `0557:2008`;
`pl2303` and `usbserial` loaded; `/dev/ttyUSB0` exists; `dialout` listed;
both `read OK` and `write OK` printed.

The custom PL2303 module was built for `5.15.199-tegra`. This quick-start assumes
permanent installation under `/lib/modules/<kernel>/extra` with automatic loading
through `modules-load.d` and availability through `modprobe`.
The older [full bench guide](bench_testing.md) describes manual `insmod` loading.

If `/dev/ttyUSB0` is missing:

```bash
sudo modprobe pl2303
ls -l /dev/ttyUSB0
```

If `modprobe` fails after a kernel/JetPack update, the PL2303 module may need to be
rebuilt for the new kernel. If `dialout` is missing:

```bash
sudo usermod -aG dialout "$USER"
```

Log out and back in, then repeat the permission checks.

## 2. Start Windows mock prerequisite

- Windows mock LC must already be running on the Windows-side ATEN COM port,
  without fault injection, **before starting the Jetson driver**.
- Physical RS485 wiring should already be connected. Jetson expects `/dev/ttyUSB0`.
- Windows details: [Windows mock README](../tools/windows_mock_lc/README.md).

## 3. Start LC driver — Terminal 1

```bash
cd ~/Desktop/jetson-lc-communication
source install/setup.bash
./scripts/start_driver.sh /dev/ttyUSB0
```

Leave this terminal running.

## 4. Check communication health — Terminal 2

```bash
cd ~/Desktop/jetson-lc-communication
source install/setup.bash
ros2 topic echo /lc/communication_status
```

Healthy fresh run:

```text
serial_connected: true
receiving_valid_frames: true
status_stale: false
consecutive_timeouts: 0
crc_error_count: 0
malformed_frame_count: 0
last_error: ''
```

During normal polling, `last_function_code` alternates between decimal **16 and 17**
(`0x10` and `0x11`); echoes may repeat values. Completion states 7–8 use only `0x11`;
normal polling resumes at state 9. Commands can also report `0x12` (decimal 18).

## 5. Check LC data

In Terminal 2, use Ctrl+C to stop each echo before starting the next:

```bash
ros2 topic echo /lc/status
ros2 topic echo /lc/distance
ros2 topic echo /lc/configuration
```

**Deterministic Windows mock bench values only — not production LC values:**

| Topic | Expected fields |
| --- | --- |
| `/lc/status` | `lift_id: 1`, `current_floor: 16` |
| `/lc/distance` | `distance_from_car_top_to_current_floor_sill_mm: 1400`, `valid: true` |
| `/lc/configuration` | `rated_capacity_kg: 900`, `car_width_mm: 1600`, `door_width_mm: 1000`, `has_second_door: false` |

## 6. Bench command workflow

Prepare Terminal 3 for commands:

```bash
cd ~/Desktop/jetson-lc-communication
source install/setup.bash
./scripts/start_hold.sh
```

In Terminal 2, stop any previous echo with Ctrl+C, then monitor:

```bash
ros2 topic echo /lc/status
```

Expected Ready state:

```text
jetson_operation_permitted: true
robot_maintenance_mode: true
lift_abnormal_mode: false
automatic_service_session_state: 6
automatic_service_abnormal_reason: 0
```

**Do not issue Manual Down until state 6 / Ready for Jetson/Robot Control is confirmed.**

`SetLcCommandState` returning `accepted=true` means validated/queued only; it does
not prove the LC completed the operation. Confirm actual progress and completion
from `/lc/status`. Keep Terminal 2 monitoring throughout.

In Terminal 3, run each command separately; check the Windows mock console for
Manual Down ON, then OFF, before continuing:

```bash
./scripts/manual_down.sh
```

Stop Manual Down:

```bash
./scripts/manual_stop.sh
```

Normal completion:

```bash
./scripts/complete_maintenance.sh
```

Confirm `/lc/status` sequence **7 → 8 → 9**, ending with:

```text
jetson_operation_permitted: false
robot_maintenance_mode: false
lift_abnormal_mode: false
automatic_service_session_state: 9
```

## 7. Abort test

Start a fresh maintenance session in Terminal 3:

```bash
./scripts/start_hold.sh
```

Wait for state 6 on `/lc/status`, then:

```bash
./scripts/abort_maintenance.sh
```

Expected mock status: `automatic_service_session_state: 11`.
Decimal **11 = 0x0B Aborted**.

## 8. Refresh configuration

```bash
./scripts/refresh_configuration.sh
```

Queues a fresh `0x10` request; confirm the update on `/lc/configuration`.

## 9. Stop

Driver terminal: **Ctrl+C**.

## 10. Quick bench validation

Windows mock running; serial/permission checks passed. Run command lines separately
so each observation below is confirmed before continuing.

Terminal 1:

```bash
cd ~/Desktop/jetson-lc-communication
source install/setup.bash
./scripts/start_driver.sh /dev/ttyUSB0
```

Terminal 2:

```bash
cd ~/Desktop/jetson-lc-communication
source install/setup.bash
ros2 topic echo /lc/communication_status
```

After health passes, Ctrl+C in Terminal 2 and monitor status:

```bash
ros2 topic echo /lc/status
```

Terminal 3 (run one line at a time):

```bash
cd ~/Desktop/jetson-lc-communication
source install/setup.bash
./scripts/start_hold.sh
# Wait for state 6 and Ready flags in Terminal 2 before issuing Manual Down.
./scripts/manual_down.sh
# Confirm Manual Down ON in the Windows mock console.
./scripts/manual_stop.sh
# Confirm Manual Down OFF in the Windows mock console.
./scripts/complete_maintenance.sh
# Confirm 7 -> 8 -> 9 in Terminal 2.
```

## 11. Pass criteria

- [ ] `/dev/ttyUSB0` exists; read/write permission OK.
- [ ] `serial_connected: true`, `receiving_valid_frames: true`, `status_stale: false`.
- [ ] No timeouts, CRC errors, or malformed frames during the fresh healthy run.
- [ ] `0x10`/`0x11` polling works.
- [ ] Start/Hold reaches state 6 with Ready flags.
- [ ] Manual Down ON/OFF confirmed in the Windows mock console.
- [ ] Completion reaches 7 → 8 → 9 with final flags above.

Successful Windows mock testing does **not** prove final ENV production-LC interoperability.
