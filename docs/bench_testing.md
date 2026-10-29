# Jetson–LC Physical Bench Validation Commands

Assumed physical setup:

```text
Jetson
  ↓
ATEN USB → RS232
  ↓
UT-201B RS232 → RS485
  ↓
T/R+ ───────── T/R+
T/R- ───────── T/R-
GND  ───────── GND
  ↓
UT-201B RS485 → RS232
  ↓
ATEN RS232 → USB
  ↓
Windows laptop
```

`VCC` must remain unconnected.

---

## 1. Windows: build the mock LC

Open **Visual Studio Developer Command Prompt**.

Go to the repository:

```bat
cd Desktop\jetson-lc-communication
```

Configure:

```bat
cmake -S tools\windows_mock_lc ^
  -B tools\windows_mock_lc\build ^
  -A x64
```

Build:

```bat
cmake --build tools\windows_mock_lc\build --config Release
```

The executable should be at:

```text
tools\windows_mock_lc\build\Release\mock_lc_serial.exe
```

---

## 2. Windows: start the mock LC

Check Device Manager for the ATEN COM port.

Current bench setup used:

```text
COM3
```

Start:

```bat
tools\windows_mock_lc\build\Release\mock_lc_serial.exe COM3
```

Leave this terminal running.

Expected:

```text
Standalone mock LC listening on COM3 at 38400 baud, 8N1, no flow control
Lift ID: 1
Responder only; press Ctrl+C to stop.
```

Do not use fault injection for the normal validation run.

---

# Jetson setup

## 3. Check the ATEN USB adapter

```bash
lsusb
```

Expected device:

```text
0557:2008 ATEN International Co., Ltd UC-232A Serial Port
```

---

## 4. Load the PL2303 driver

The current Jetson kernel does not include PL2303 by default, so after a reboot:

```bash
sudo modprobe usbserial
sudo insmod ~/pl2303-module/pl2303.ko
```

Check:

```bash
lsmod | grep -E 'pl2303|usbserial'
```

Expected:

```text
pl2303
usbserial
```

Check serial device:

```bash
ls -l /dev/ttyUSB*
```

Expected:

```text
/dev/ttyUSB0
```

Optional kernel confirmation:

```bash
sudo dmesg | grep -iE 'pl2303|ttyUSB|0557|2008' | tail -n 30
```

Expected:

```text
pl2303 converter detected
pl2303 converter now attached to ttyUSB0
```

---

## 5. Check serial permissions

```bash
groups
```

Make sure:

```text
dialout
```

is listed.

Check access:

```bash
test -r /dev/ttyUSB0 && echo "read OK"
test -w /dev/ttyUSB0 && echo "write OK"
```

Expected:

```text
read OK
write OK
```

If `dialout` is missing:

```bash
sudo usermod -aG dialout jetson-nano
```

Then either log out/in or:

```bash
newgrp dialout
```

---

# Start the Jetson driver

## 6. Terminal 1 — driver

```bash
cd ~/Desktop/jetson-lc-communication
source install/setup.bash

./scripts/start_driver.sh /dev/ttyUSB0
```

Equivalent direct command:

```bash
ros2 run jetson_lc_driver lc_driver_node --ros-args \
  -p serial_device:=/dev/ttyUSB0
```

Leave it running.

---

# Observe communication

## 7. Terminal 2 — communication health

```bash
cd ~/Desktop/jetson-lc-communication
source install/setup.bash

ros2 topic echo /lc/communication_status
```

Healthy result:

```text
serial_connected: true
receiving_valid_frames: true
status_stale: false
consecutive_timeouts: 0
last_error: ''
```

For a fresh run, the following counters should not increase:

```text
crc_error_count
malformed_frame_count
```

If they already contain values from earlier tests, the important thing is that they remain constant during healthy communication.

---

## 8. Check LC status

```bash
ros2 topic echo /lc/status
```

Initial mock state should be approximately:

```text
current_floor: 16
jetson_operation_permitted: false
robot_maintenance_mode: false
lift_abnormal_mode: false
automatic_service_session_state: 0
automatic_service_abnormal_reason: 0
```

---

## 9. Check distance

```bash
ros2 topic echo /lc/distance
```

Expected mock value:

```text
distance_from_car_top_to_current_floor_sill_mm: 1400
valid: true
```

---

## 10. Check configuration

```bash
ros2 topic echo /lc/configuration
```

Expected deterministic mock configuration includes:

```text
rated capacity: 900 kg
car width: 1600 mm
door width: 1000 mm
```

---

# Maintenance command validation

Use another Jetson terminal:

```bash
cd ~/Desktop/jetson-lc-communication
source install/setup.bash
```

---

## 11. Start / Hold Robot Maintenance

```bash
./scripts/start_hold.sh
```

Equivalent service call:

```bash
ros2 service call /lc/set_command_state \
  jetson_lc_interfaces/srv/SetLcCommandState \
  "{destination_floor: 0, hall_call_disable: true, manual_motion: 0, maintenance_request: 1}"
```

Windows should decode:

```text
Hall Call Disable: ON
Manual Up: OFF
Manual Down: OFF
Automatic Service Request: ON
Request Type: START_HOLD
Reserved Door Bits: OFF
```

Monitor:

```bash
ros2 topic echo /lc/status
```

Expected progression:

```text
0x01
→ 0x02
→ 0x04
→ 0x05
→ 0x06
```

Ready state:

```text
automatic_service_session_state: 6
jetson_operation_permitted: true
robot_maintenance_mode: true
lift_abnormal_mode: false
```

---

## 12. Manual Down ON

Only run after state `0x06`:

```bash
./scripts/manual_down.sh
```

Equivalent:

```bash
ros2 service call /lc/set_command_state \
  jetson_lc_interfaces/srv/SetLcCommandState \
  "{destination_floor: 0, hall_call_disable: true, manual_motion: 2, maintenance_request: 1}"
```

Windows should decode:

```text
Manual Down: ON
Automatic Service Request: ON
Request Type: START_HOLD
```

---

## 13. Manual Down OFF / Stop

```bash
./scripts/manual_stop.sh
```

Equivalent:

```bash
ros2 service call /lc/set_command_state \
  jetson_lc_interfaces/srv/SetLcCommandState \
  "{destination_floor: 0, hall_call_disable: true, manual_motion: 0, maintenance_request: 1}"
```

Windows should decode:

```text
Manual Down: OFF
Automatic Service Request: ON
Request Type: START_HOLD
```

---

# Normal completion

## 14. Complete maintenance / Request Clear

```bash
./scripts/complete_maintenance.sh
```

Equivalent:

```bash
ros2 service call /lc/set_command_state \
  jetson_lc_interfaces/srv/SetLcCommandState \
  "{destination_floor: 0, hall_call_disable: false, manual_motion: 0, maintenance_request: 0}"
```

Windows should decode:

```text
Hall Call Disable: OFF
Manual Up: OFF
Manual Down: OFF
Automatic Service Request: OFF
Request Type: REQUEST_CLEAR
Reserved Door Bits: OFF
```

Monitor:

```bash
ros2 topic echo /lc/status
```

Expected:

```text
0x07 Completion Request Received
→
0x08 Returning to Auto Mode
→
0x09 Service Complete / Normal Operation Restored
```

Final state:

```text
automatic_service_session_state: 9
jetson_operation_permitted: false
robot_maintenance_mode: false
lift_abnormal_mode: false
```

During `0x07` and `0x08`, the driver should only poll `0x11`.

After `0x09`, normal `0x10` and `0x11` polling resumes.

---

# Abort validation

## 15. Start a new session

```bash
./scripts/start_hold.sh
```

Wait until:

```text
automatic_service_session_state: 6
```

Then:

```bash
./scripts/abort_maintenance.sh
```

Equivalent:

```bash
ros2 service call /lc/set_command_state \
  jetson_lc_interfaces/srv/SetLcCommandState \
  "{destination_floor: 0, hall_call_disable: true, manual_motion: 0, maintenance_request: 2}"
```

Windows should decode:

```text
Manual Up: OFF
Manual Down: OFF
Automatic Service Request: ON
Request Type: ABORT
```

Expected LC state:

```text
0x0B Aborted
```

Check:

```bash
ros2 topic echo /lc/status
```

---

# Configuration refresh

## 16. Request fresh `0x10`

```bash
./scripts/refresh_configuration.sh
```

Equivalent:

```bash
ros2 service call /lc/refresh_configuration \
  jetson_lc_interfaces/srv/RefreshLcConfiguration "{}"
```

---

# Windows timing validation

During normal operation, Windows prints:

```text
Request 0x11 timing:
  frame parsed after: X ms
  reply prepared after: X ms
  reply write started after: X ms
  reply write completed after: X ms
```

For this bench setup we observed normal replies around:

```text
~1–3 ms
```

which is comfortably below the required:

```text
100 ms
```

All received frames should show:

```text
CRC: VALID
```

---

# Optional physical fault test

While communication is running, disconnect the physical serial link.

Monitor:

```bash
ros2 topic echo /lc/communication_status
```

Expected:

```text
receiving_valid_frames: false
status_stale: true
```

Depending on which device is disconnected:

```text
serial_connected: false
```

may also occur.

Reconnect and verify communication recovers.

---

# Optional Windows fault injection

Stop the normal Windows mock and run one of these.

Dropped responses:

```bat
tools\windows_mock_lc\build\Release\mock_lc_serial.exe COM3 --drop-every 5
```

Delayed responses:

```bat
tools\windows_mock_lc\build\Release\mock_lc_serial.exe COM3 --delay-ms 150
```

Bad CRC:

```bat
tools\windows_mock_lc\build\Release\mock_lc_serial.exe COM3 --bad-crc-every 5
```

Then monitor on Jetson:

```bash
ros2 topic echo /lc/communication_status
```

Expected:

```text
drop/delay:
  LC response timeout

bad CRC:
  crc_error_count increases
  invalid CRC
```

---

# Stop the test

Jetson driver:

```text
Ctrl+C
```

Windows mock:

```text
Ctrl+C
```

---

# Bench validation pass criteria

```text
[ ] Windows mock opens COM port at 38400 8N1
[ ] Jetson exposes /dev/ttyUSB0
[ ] Driver opens serial port
[ ] 0x10 requests/replies work
[ ] 0x11 requests/replies work
[ ] CRC is valid
[ ] Distance = 1400 mm received
[ ] Start/Hold reaches 0x06
[ ] Periodic hold frames are received
[ ] Manual Down ON received correctly
[ ] Manual Down OFF received correctly
[ ] Completion request received correctly
[ ] Completion progresses 0x07 → 0x08 → 0x09
[ ] Abort reaches 0x0B
[ ] Reserved door bits remain OFF
[ ] Normal response latency stays below 100 ms
[ ] No new timeout/CRC/malformed counters during healthy operation
[ ] Physical disconnect is detected
[ ] Communication recovers after reconnect
```