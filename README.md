# arm_test_tools

```
使用環境：Ubuntu 16.04 + ROS Kinetic
驅動程式：youbot_driver_ros_interface
```

+ 剛開始接觸 youBot 手臂控制時主要用到的測試相關檔案
+ 一切資料夾位置編排都不考慮 ros workspace 架構，將所需檔案拉到自己的 rospkg 中，並設置好 CMakeLists.txt 就可以用

## Listeners

沒有機台可以測的時候，用來檢測程式指令發布用的程式

+ base listener : subscribe from <i>/cmd_vel</i>
+ arm listener : subscribe from <i>/arm_controller/position_command</i>

## Arm Test

+ Publish to <i>/arm_controller/position_command</i>
+ Subscribe from <i>/joint_states</i>
