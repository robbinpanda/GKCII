cat /proc/bus/input/devices | tee devices_info.txt
python3 src/control_pkg/src/g29_info.py
