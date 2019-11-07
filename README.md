# gcs
基于ROS rviz控件的地面站界面设计（ROS catkin工作空间）
## Get started
1.确认安装serial包和joy包
```
sudo apt install ros-kinetic-joy-drivers
sudo apt install ros-kinetic-serial
```
2.在工作空间根目录下执行
```
catkin_make -DBODY_FRAME=2WING or 4WING
```
必须指定一个机架类型