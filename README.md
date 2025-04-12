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
ros2 launch rmslaunch carto.launch.py
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
- 由于cartographer使用要求imu的frame和tracing_frame的frame一致，所以将/sick_scansegment_xd/imu重新在base_link发布，并重映射为/imu

## 使用
- 首先启动cartographer构建地图，构图结束后保存地图，关闭进程（不然会和导航冲突）
    -   ``ros2 launch rmslaunch carto.launch.py``
- 启动navgation2
    - ``ros2 launch rmslaunch nav.launch.py ``
- 定点导航
    - 点击Rviz2 上边菜单栏里的``2D Goal Pose ``，设置机器人的导航目标点，千万不要点``2D Pose Estimate``，这个是设置初始位姿的
    - 获得速度指令
        - ``/cmd_vel``发布速度指令的话题，其消息接口类型为``geometry_msgs::msg::Twist``
        - 示例代码
      ```c++
        #include <rclcpp/rclcpp.hpp>
        #include <geometry_msgs/msg/twist.hpp>
        #include <cstring>
      
        class CmdVelSubscriber : public rclcpp::Node
        {
        public:
        CmdVelSubscriber()
        : Node("cmd_vel_subscriber")
        {
          // 创建一个订阅者，订阅/cmd_vel,当这个话题被发布的时候调用topic_callback回调函数`
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&CmdVelSubscriber::topic_callback, this, std::placeholders::_1)); 
        }
        private:
        void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
        {
         // 这样子就拿到了速度指令并存放在 vx vy vz这个三个变量里面
         float vx = msg->linear.x;
         float vy = msg->linear.y;
         float vz = msg->linear.z;
         rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
        }
        };
```

