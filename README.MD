开启程序
source install/setup.bash
ros2 launch bringup bringup.launch.py

一键校准
source install/setup.bash
ros2 launch hik_camera hik_camera.launch.py
ros2 run hsv_calibrator hsv_calibrator
校准结果在终端打印
在对应的config/green_light.yaml修改