# NVILIDAR ROS DRIVER(V1.0.1)


## How to [install ROS](http://wiki.ros.org/cn/ROS/Installation)

[ubuntu](http://wiki.ros.org/cn/Installation/Ubuntu)

[windows](http://wiki.ros.org/Installation/Windows)

## How to Create a ROS workspace

[Create a workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

you also can with this:

    1)  $mkdir -p ~/nvilidar_ros_ws/src
        $cd ~/nvilidar_ros_ws/src
    2)  $cd..
    3)  $catkin_make
    4)  $source devel/setup.bash
    5)  echo $ROS_PACKAGE_PATH
        /home/user/nvilidar_ws/src:/opt/ros/kinetic/share


## How to build NVILiDAR ROS Package

    1) Clone this project to your catkin's workspace src folder
    	(1). git clone https://github.com/nvilidar/     nvilidar_ros.git  
          or
          git clone https://gitee.com/nvilidar/nvilidar_ros.git
    	(2). git chectout master
    2) Copy the ros source file to the "~/nvilidar_ros_ws/src"
    2) Running "catkin_make" to build nvilidar_node and nvilidar_client
    3) Create the name "/dev/nvilidar" to rename serialport
    --$ roscd nvilidar_ros/startup
    --$ sudo chmod 777 ./*
    --$ sudo sh initenv.sh


## How to Run NVILIDAR ROS Package
#### 1. Run NVILIDAR node and view in the rviz
------------------------------------------------------------
	roslaunch nvilidar_ros lidar_view.launch

#### 2. Run NVILIDAR node and view using test application
------------------------------------------------------------
	roslaunch nvilidar_ros lidar.launch

	rosrun nvilidar_ros nvilidar_client

## NVILIDAR ROS Parameter
|  value   |  information  |
|  :----:    | :----:  |
| serialport_baud  | if use serialport,the lidar's serialport |
| serialport_name  | if use serialport,the lidar's port name |
| ip_addr  | if use udp socket,the lidar's ip addr,default:192.168.1.200 |
| lidar_udp_port  | if use udp socket,the lidar's udp port,default:8100 |
| config_tcp_port  | if use udp socket,config the net converter's para,default:8200 |
| frame_id  | it is useful in ros,lidar ros frame id |
| resolution_fixed  | Rotate one circle fixed number of points,it is 'true' in ros,default |
| auto_reconnect  | lidar auto connect,if it is disconnet in case |
| reversion  | lidar's point revert|
| inverted  | lidar's point invert|
| angle_max  | lidar angle max value,max:180.0°|
| angle_max  | lidar angle min value,min:-180.0°|
| range_max  | lidar's max measure distance,default:64.0 meters|
| range_min  | lidar's min measure distance,default:0.0 meters|
| aim_speed  | lidar's run speed,default:10.0 Hz|
| sampling_rate  | lidar's sampling rate,default:10.0 K points in 1 second|
| sensitive  | lidar's data with sensitive,default:false|
| tailing_level  | lidar's tailing level,The smaller the value, the stronger the filtering,default:6|
| angle_offset  | angle offset,default:0.0|
| adp_change_flag  | change apd value,don't change it if nessesary,default:false|
| adp_value  | change apd value,if the 'apd_change_flag' is true,it is valid,default:500|
| single_channel  | it is default false,don't change it|
| ignore_array_string  | if you want to filter some point's you can change it,it is anti-clockwise for the lidar.eg. you can set the value "30,60,90,120",you can remove the 30°~60° and 90°~120° points in the view|