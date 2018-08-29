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

#---

# 把16线激光雷达数据合成高质量2D激光雷达数据
### 可参考蓝鲸机器人论坛的帖子（叫“使用pointcloud_to_laserscan包将速腾聚创3D激光雷达转换成高质量2d激光雷达”）
### 利用pointcloud_to_laserscan把点云数据合成2D激光雷达数据（需要先启动16线激光雷达生成点云数据）
'''
roslaunch pointcloud_to_laserscan xiaoqiang_rslidar.launch
'''

#---

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

#---
# agv启动movebase包
###启动里程计、IMU、雷达、以及move_base包（导航包）
```
roslaunch adv_comm agv_move_base.launch
```
## “agv_move_base.launch”说明：
###启动里程计语句：
```
<include file="$(find adv_comm)/launch/get_message.launch" />
```
###启动IMU语句：
```
<include file="$(find razor_imu_9dof)/launch/razor-pub.launch" />
```
###启动激光雷达语句：
####此处可以配置激光雷达的frame_id、以及串口名称、串口波特率
```
<node name="rplidarNode"          pkg="rplidar_ros"  type="rplidarNode" output="screen">
    <param name="serial_port"         type="string" value="/dev/ttyUSB001"/>
    <param name="serial_baudrate"     type="int"    value="115200"/>
    <param name="frame_id"            type="string" value="laser"/>
    <param name="inverted"            type="bool"   value="false"/>
    <param name="angle_compensate"    type="bool"   value="true"/>
  </node>
```
###启动move_base包：
####这里使用深蓝学院配置的move_base包和参数
####move_base包参数的含义和详情可以参考ros官网的说明
```
<node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
        <rosparam file="$(find adv_comm)/param/move_base_params.yaml" command="load" />
        <rosparam file="$(find adv_comm)/param/global_costmap_params.yaml" command="load" ns="global_costmap"/>
        <rosparam file="$(find adv_comm)/param/local_costmap_params.yaml" command="load" ns="local_costmap"/>
        <rosparam file="$(find adv_comm)/param/global_planner_params.yaml" command="load" ns="GlobalPlanner"/>
        <rosparam file="$(find adv_comm)/param/dwa_local_planner_params.yaml" command="load" ns="DWAPlannerROS"/>
    </node>
```
