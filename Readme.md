# 关于速腾16线激光雷达的使用
### 已经配置好，网线直接连上就可以了
### 可参考github上的readme.md
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

# ---

# 把16线激光雷达数据合成高质量2D激光雷达数据
### 可参考蓝鲸机器人论坛的帖子（叫“使用pointcloud_to_laserscan包将速腾聚创3D激光雷达转换成高质量2d激光雷达”）
### 利用pointcloud_to_laserscan把点云数据合成2D激光雷达数据（需要先启动16线激光雷达生成点云数据）
'''
roslaunch pointcloud_to_laserscan xiaoqiang_rslidar.launch
'''

# ---

# 利用cartogragper包构建高质量地图
### 可参考蓝鲸论坛的这个帖子，或者参考小强机器人手册中的第16章节
### 使用splidar构建2D地图（也可以使用16线激光雷达合成的2D激光雷达数据去跑）
```
roslaunch cartographer_ros xiaoqiang_rplidar_2d.launch
```
### 采集完地图数据后使用下面命令保存地图，地图数据就保存在执行命令的文件夹下
```
rosrun map_server map_saver --occ 51 --free 49 -f work0
```

# ---
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
#### move_base包参数的含义和详情可以参考ros官网的说明
```
<node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
        <rosparam file="$(find adv_comm)/param/move_base_params.yaml" command="load" />
        <rosparam file="$(find adv_comm)/param/global_costmap_params.yaml" command="load" ns="global_costmap"/>
        <rosparam file="$(find adv_comm)/param/local_costmap_params.yaml" command="load" ns="local_costmap"/>
        <rosparam file="$(find adv_comm)/param/global_planner_params.yaml" command="load" ns="GlobalPlanner"/>
        <rosparam file="$(find adv_comm)/param/dwa_local_planner_params.yaml" command="load" ns="DWAPlannerROS"/>
    </node>
```

# ---
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
### 参数的说明可以在ros官网上查询
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
