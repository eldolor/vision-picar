# Our fork of the UGV Rover's motor-board firmware

`0001-feedback-mm-odometers-and-board-time.patch` applies to Waveshare's
[`ugv_base_ros`](https://github.com/waveshareteam/ugv_base_ros) at commit
`2e7df97` -- the **ROS Driver for Robots** board's ESP32 firmware (C++, an
Arduino sketch). Why it exists, and how it is judged: `PLAN-ros-alignment.md`
3.28.

It adds three keys to the `T:1001` feedback frame, in `baseInfoFeedback()`
(`ROS_Driver/ugv_advance.h`), and changes nothing else:

| key | value |
|---|---|
| `odlm`, `odrm` | each wheel's travel since boot in whole millimetres, a C `long` truncated toward zero -- the same `en_odom_l`/`en_odom_r` the stock `odl`/`odr` truncate to centimetres |
| `ms` | `millis()` when the frame was built: zero at boot, wraps after 49.7 days |

`odl`/`odr` are untouched, so Waveshare's tools and a host that does not
know the new keys behave exactly as on a stock board. `robot/hardware_robot.py`
uses the new keys when they are present.

## Licence

The firmware is GPL-3.0 (its `LICENSE`), and so is this patch. Running it on
your own robot needs nothing more. Giving someone the compiled firmware --
or a robot running it -- means giving them this source too.

## Build and flash (hardware day, after the arrival checks)

```bash
git clone https://github.com/waveshareteam/ugv_base_ros && cd ugv_base_ros
git checkout 2e7df97
git apply /path/to/vision-picar/firmware/ugv_base_ros/0001-feedback-mm-odometers-and-board-time.patch
bash /path/to/vision-picar/firmware/ugv_base_ros/build.sh .      # compile only
```

**Before the first flash, dump the stock image** so it can be put back:
`esptool.py --port <port> read_flash 0 ALL stock-ugv_base_ros.bin`.
Waveshare's ESP32 Download Tool restores their published image too.
Board: "ESP32 Dev Module" (`esp32:esp32:esp32`) -- the chip is the original
ESP32, `GUIDE-robot-base.md` section 1.
