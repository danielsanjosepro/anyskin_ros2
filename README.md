# AnySkin ROS2 wrapper

## Get started

Start broadcasting tactile data with the sensor:
```bash
pixi run -e humble anyskin-ros2 -h
```
To publish compressed images use:

```bash
pixi run -e humble republish-compressed
```

## Set udev-rules

If you use many sensors, you might want to set some udev rules so that they are always mapped / symlinked to the same device name.
Add a script to `scripts/99_anyskin.rules` that looks as follows:
```bash
SUBSYSTEM=="tty", ATTRS{idVendor}=="239a", ATTRS{idProduct}=="80cb", SYMLINK+="anyskin_left", MODE="0666", ATTR{device/latency_timer}="1"
# SUBSYSTEM=="tty", ATTRS{idVendor}=="xxxx", ATTRS{idProduct}=="xxxx", SYMLINK+="anyskin_right", MODE="0666", ATTR{device/latency_timer}="1"
```
you should mofify the `idVendor` and `idProduct` with the correct numbers. You can check it using `lsusb` or `cyme`.

Finally, run the script `scripts/set_udev_rules.sh`, and check the `ls /dev/anyskin_*` are available.
Now you can start the container with:

```bash
NAMESPACE:=left_finger DEVICE:=/dev/anyskin_left docker compose up anyskin_ros2_magnitude_broadcaster
```

