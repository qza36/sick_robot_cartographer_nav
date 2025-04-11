# 使用sick picoscan150 2D 雷达配置cartographer进行建图导航


## env
- ubuntu 2204
- ros-humble

## build
```
colcon build --symlink --cmake-args -DCMAKE_BUILD_TYPE=Release -G Ninja
```
## run lidar_driver
```
source install/setup.bash
ros2 launch sick_scan_xd sick_picoscan.launch.py hostname:=192.168.192.100 udp_receiver_ip:=192.168.192.110
```

## run cartographer
```
source install/setup.bash
ros2 launch rmslaunch launch.rmsros2.py 
```

## save map 
``` 
ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename : '${HOME}/rmsros2/src/rmslaunch/map/cartographer_map.pbstream'}"
ros2 run nav2_map_server map_saver_cli -f ~/rmsros2/src/rmslaunch/map/map

```

## strat navigation
```
ros2 launch rmslaunch nav.launch.py
```

## show transferPose
```
ros2 run rmslaunch cartoPose

```

## NOTES
- 修改雷达驱动程序，发布laser_link2cloud_frame的tf，因为原始参数发布map2world，与建图导航冲突
- 转化/cloud_all_fields_fullframe到/scan，cartographer订阅消息类型为LaserScan的/scan，而cloud_all_fields_fullframe是pointclou2类型
- 由于cartographer使用要求imu的frame和tracing_package的frame一致，所以将/sick_scansegment_xd/imu重新在base_link发布，并重映射为/imu
