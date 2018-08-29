# 关于速腾16线激光雷达的使用
### 已经配置好，网线直接连上就可以了
### 可参考github上的[readme.md](https://github.com/MUZUIXIAOHAI/ros_rslidar)
### 首先把库从github上拉下来
```
git clone https://github.com/MUZUIXIAOHAI/ros_rslidar.git
```
### 然后安装需要的库libpcap-dev
```
sudo apt-get install libpcap-dev
```
### 编译包(在ros目录下)
```
catkin_make
```
### 运行激光雷达，查看点云数据
```
roslaunch rslidar_pointcloud rs_lidar_16.launch
```

***

# 把16线激光雷达数据合成高质量2D激光雷达数据
### 可参考[蓝鲸机器人论坛的帖子](https://community.bwbot.org/topic/521/%E4%BD%BF%E7%94%A8pointcloud_to_laserscan%E5%8C%85%E5%B0%86%E9%80%9F%E8%85%BE%E8%81%9A%E5%88%9B3d%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE%E8%BD%AC%E6%8D%A2%E6%88%90%E9%AB%98%E8%B4%A8%E9%87%8F2d%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BE)（叫“使用pointcloud_to_laserscan包将速腾聚创3D激光雷达转换成高质量2d激光雷达”）
### 利用pointcloud_to_laserscan把点云数据合成2D激光雷达数据（需要先启动16线激光雷达生成点云数据）
'''
roslaunch pointcloud_to_laserscan xiaoqiang_rslidar.launch
'''

***

# 利用cartogragper包构建高质量地图
### 可参考[蓝鲸论坛的这个帖子](https://community.bwbot.org/topic/137/%E5%B0%8F%E5%BC%BAros%E6%9C%BA%E5%99%A8%E4%BA%BA%E6%95%99%E7%A8%8B-16-___%E5%A4%A7%E8%8C%83%E5%9B%B4%E6%BF%80%E5%85%89%E9%9B%B7%E8%BE%BEslam%E4%B8%8E%E5%AE%9E%E6%97%B6%E5%9B%9E%E8%B7%AF%E9%97%AD%E5%90%88%E6%B5%8B%E8%AF%95)，或者参考小强机器人手册中的第16章节
### 使用rplidar构建2D地图（也可以使用16线激光雷达合成的2D激光雷达数据去跑）
```
roslaunch cartographer_ros xiaoqiang_rplidar_2d.launch
```
### 采集完地图数据后使用下面命令保存地图，地图数据就保存在执行命令的文件夹下
```
rosrun map_server map_saver --occ 51 --free 49 -f work0
```

***

# agv启动movebase包
### 启动里程计、IMU、雷达、以及move_base包（导航包）
```
roslaunch adv_comm agv_move_base.launch
```
## “agv_move_base.launch”说明：
### 启动里程计语句：
```
<include file="$(find adv_comm)/launch/get_message.launch" />
```
### 启动IMU语句：
```
<include file="$(find razor_imu_9dof)/launch/razor-pub.launch" />
```
### 启动激光雷达语句：
#### 此处可以配置激光雷达的frame_id、以及串口名称、串口波特率
```
<node name="rplidarNode"          pkg="rplidar_ros"  type="rplidarNode" output="screen">
    <param name="serial_port"         type="string" value="/dev/ttyUSB001"/>
    <param name="serial_baudrate"     type="int"    value="115200"/>
    <param name="frame_id"            type="string" value="laser"/>
    <param name="inverted"            type="bool"   value="false"/>
    <param name="angle_compensate"    type="bool"   value="true"/>
  </node>
```
### 启动move_base包：
#### 这里使用深蓝学院配置的move_base包和参数
#### move_base包参数的含义和详情可以参考[ros官网move_base的说明](http://wiki.ros.org/move_base)
```
<node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
        <rosparam file="$(find adv_comm)/param/move_base_params.yaml" command="load" />
        <rosparam file="$(find adv_comm)/param/global_costmap_params.yaml" command="load" ns="global_costmap"/>
        <rosparam file="$(find adv_comm)/param/local_costmap_params.yaml" command="load" ns="local_costmap"/>
        <rosparam file="$(find adv_comm)/param/global_planner_params.yaml" command="load" ns="GlobalPlanner"/>
        <rosparam file="$(find adv_comm)/param/dwa_local_planner_params.yaml" command="load" ns="DWAPlannerROS"/>
    </node>
```

***

# agv启动定位、开始激光导航
### agv启动amcl定位、载入地图、模型、IMU与里程计数据融合以及启动rviz
```
roslaunch adv_sim agv_amcl_slam_v2.launch
```
## “agv_amcl_slam_v2.launch”说明
##
### 构建tf树，一些必要的对应
```
<node pkg="tf" type="static_transform_publisher" name="base_footprint_to_base_link_broadcaster" args="0 0 0 0 0 0 /base_footprint /base_link 100"/>
<node name="base_imu_link" pkg="tf" type="static_transform_publisher" args="0 0 0 0 3.1425926 0 /base_link /base_imu_link 50"/>
<node name="map_to_odom_link" pkg="tf" type="static_transform_publisher" args="0 0 0 0 0 0 /map /odom 50"/>
```
###
### 对里程计数据和IMU数据进行融合，利用robot_pose_ekf包
```
<include file="$(find robot_pose_ekf)/robot_pose_ekf.launch" />
```
###
### 载入小车模型
```
<param name="/use_sim_time" value="ture" />

<!-- Load the URDF/Xacro model of our robot -->
<arg name="urdf_file" default="$(find xacro)/xacro.py '$(find adv_sim)/my_xacro/myrobot.urdf'" />
<param name="robot_description" command="$(arg urdf_file)" />

<node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher">
    <param name="publish_frequency" type="double" value="20.0" />
</node>
<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" >
   <param name="/use_gui" value="false"/>
   <param name="rate" value="20.0"/>
</node>
```
###
### 载入地图
```
<node name="map_server" pkg="map_server" type="map_server" args="/root/catkin_ws/src/maps/work1.yaml"/>
```
###
### 使用amcl定位
### 参数的说明可以在[ros官网](http://wiki.ros.org/amcl)上查询
```
<node pkg="amcl" type="amcl" name="amcl" output="screen">
        <param name="use_map_topic"             value="ture"/>
        <param name="odom_frame_id"             value="odom_combined"/>
      <param name="base_frame_id"             value="base_footprint"/>
      <param name="global_frame_id"           value="map"/>
      <param name="odom_model_type"           value="diff"/>
      <param name="gui_publish_rate"          value="10.0"/>

  <param name="laser_max_beams"           value="60"/>
        <param name="laser_min_range"           value="0.2"/>
        <param name="laser_max_range"           value="6.0"/>

  <param name="min_particles"             value="500"/>
      <param name="max_particles"             value="5000"/>

      <param name="laser_z_hit"               value="0.95"/>
      <param name="laser_z_short"             value="0.025"/>
      <param name="laser_z_max"               value="0.025"/>
      <param name="laser_z_rand"              value="0.05"/>
      <param name="laser_sigma_hit"           value="0.2"/>
      <param name="laser_lambda_short"        value="0.1"/>

      <param name="update_min_d"              value="0.1"/>
      <param name="update_min_a"              value="0.2"/>
      <param name="resample_interval"         value="3"/>

      <param name="transform_tolerance"       value="0.5"/>

      <param name="recovery_alpha_slow"       value="0.0"/>
      <param name="recovery_alpha_fast"       value="0.0"/>
        <param name="initial_cov_xx"            value="0.25"/>
        <param name="initial_cov_yy"            value="0.25"/>
        <param name="initial_cov_aa"            value="10.0"/>
    </node>
```
###
### 启动rviz开始导航
```
<node pkg="rviz" type="rviz" name="rviz"
    args="-d $(find adv_sim)/test_odom.rviz"/>
```
***
***
***
# 里程计说明：
## “get_message.launch”
*adv_comm->launch->get_message.launch*
该文件是生成里程计的launch文件，源代码在**adv_comm->src->rplidar_client.cpp**文件
该文件订阅了“cmd_vel”主题，接收速度指令，通过串口传输到agv上，从而驱动agv动作
发布“odom”主题，利用agv传回来的电机脉冲计算出行驶里程
`initial_all()`是初始化串口和设置一些初始参数
`sp1operation()`是发送速度信息到agv，控制agv行动，并且接收agv采集回来的电机脉冲数据
`calculate_odom(odom)`是计算里程计数据
`odom_pub.publish(odom)`发布里程计主题
`velCallback_motorspeed()`该函数是有速度主题发出，就进入该函数，把速度信息给到agv
***
# IMU说明：
## “razor-pub.launch”
*razor_imu_9dof->launch->razor-pub.launch*
IMU主程序可不用修改
IMU的参数设置在这个文件“razor_imu_9dof/config/razor.yaml”
该文件下：
`port: /dev/ttyUSB003`这里更改串口名称
***
# 雷达说明：
## 雷达串口、波特率可直接在此处修改
```
<!-- start rplidar -->
<node name="rplidarNode"          pkg="rplidar_ros"  type="rplidarNode" output="screen">
  <param name="serial_port"         type="string" value="/dev/ttyUSB001"/>
  <param name="serial_baudrate"     type="int"    value="115200"/>
  <param name="frame_id"            type="string" value="laser"/>
  <param name="inverted"            type="bool"   value="false"/>
  <param name="angle_compensate"    type="bool"   value="true"/>
</node>
```
***
# move_base包说明：
## move_base包主要是一些参数设置，这里就说一些主要设置的参数
move_base包的参数在“adv_comm/param”文件夹下
**虽然在dwa_local_planner_params.yaml文件下定义了最高速为0.3m/s，但是在rplidar_client.cpp文件下，在对agv下发速度之前就限制了最高速为0.1m/s，以保证安全**
***
# 里程计与IMU数据融合
## “robot_pose_ekf/robot_pose_ekf.launch”
[遇到问题可以查看下这个帖子](https://blog.csdn.net/qq_25241325/article/details/80825327)（调用一下服务看下主题正不正常）
这个包主要使用odom与imu进行融合，输出odom_combined
```
<node pkg="robot_pose_ekf" type="robot_pose_ekf" name="robot_pose_ekf" output="screen">
  <param name="output_frame" value="odom_combined"/>
  <param name="base_footprint_frame" value="base_footprint"/>
  <param name="freq" value="10.0"/>
  <param name="sensor_timeout" value="1.0"/>  
  <param name="odom_used" value="true"/>
  <param name="imu_used" value="true"/>
  <param name="vo_used" value="false"/>
  <param name="debug" value="true"/>
```
查看配置参数可以了解到该包可以融合里程计、IMU、视觉里程计以及GPS数据，这里就使能了里程及以及IMU的融合
***
# 模型与地图的载入
## 模型文件放在”adv_sim/my_xacro/myrobot.urdf”
## 地图文件放在“/maps/work1.yaml”
**地图使用cartogragper包制作的比hector_slam包制作的效果要好，两个包都是只给激光主题，没有给里程计或者imu数据**
***
# amcl定位
```
<node pkg="amcl" type="amcl" name="amcl" output="screen">
        <param name="use_map_topic"             value="ture"/>
        <param name="odom_frame_id"             value="odom_combined"/>
      <param name="base_frame_id"             value="base_footprint"/>
      <param name="global_frame_id"           value="map"/>
      <param name="odom_model_type"           value="diff"/>
      <param name="gui_publish_rate"          value="10.0"/>

  <param name="laser_max_beams"           value="60"/>
        <param name="laser_min_range"           value="0.2"/>
        <param name="laser_max_range"           value="6.0"/>

  <param name="min_particles"             value="500"/>
      <param name="max_particles"             value="5000"/>

      <param name="laser_z_hit"               value="0.95"/>
      <param name="laser_z_short"             value="0.025"/>
      <param name="laser_z_max"               value="0.025"/>
      <param name="laser_z_rand"              value="0.05"/>
      <param name="laser_sigma_hit"           value="0.2"/>
      <param name="laser_lambda_short"        value="0.1"/>

      <param name="update_min_d"              value="0.1"/>
      <param name="update_min_a"              value="0.2"/>
      <param name="resample_interval"         value="3"/>

      <param name="transform_tolerance"       value="0.5"/>

      <param name="recovery_alpha_slow"       value="0.0"/>
      <param name="recovery_alpha_fast"       value="0.0"/>
        <param name="initial_cov_xx"            value="0.25"/>
        <param name="initial_cov_yy"            value="0.25"/>
        <param name="initial_cov_aa"            value="10.0"/>
    </node>
```
amcl定位参数设置：（[官网更多参数说明](http://wiki.ros.org/amcl)）
odom_frame_id要使用融合后的里程计主题;
update_min_d为更新距离，此处为走0.1m更新一次粒子;
min_particles、max_particles为最小粒子、最大粒子数;
laser_min_range、laser_max_range为激光最小、最大范围;
odom_model_type为模型类型这里diff为差速控制
