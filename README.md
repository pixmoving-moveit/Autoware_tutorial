# Autoware Tutorial for Self-driving Car
## 2018 GUIYANG PIX MOVE-IT HACKATHON

## 2018 GUIYANG MOVE-IT HACKATHON INTRODUCTION
2018 GUIYANG MOVE-IT HACKATHON is the first open source hackathon workshop to make real size self-driving car in China, which is organized by a GUIYANG startup company PIX.

There are two cars, one is robot cafe car and another is Honda Civic.

[Moveit introduction](https://www.pixmoving.com/move-it)
![](./images/moveit.png)

![](./images/moveit_all.jpg)

## What we did
### Robot cafe car
* Can control robot cafe car throttle, brake and steer
* Generate map and waypoints
* Make robot cafe car self-driving follow waypoints
* Detect obstacle with Lidar

![](./images/robotcafe.jpeg)

### Honda Civic
* CAN control Civic throttle, brake and steer through comma.ai panda.
* CAN control Civic under low speed.

There are a lot of comma.ai implementation in China with Honda Civic but can not totally control steer under 20km/h.

We completely control Civic even at low speed.

It is pity for limited time we did not totally make Civic self-driving but easy to continue.

![](./images/civic.jpeg)

### Police gesture detection
* Set up the first open source Chinese Police gesture dataset.
* Train a model through transfer learning with a pre-trained inception model

![](./images/gesture.jpeg)

The Chinese police gesture detection by camera performance is not excellent due to dataset reason and limited time.

More information about is ongoing [Police gestures dataset and detection]()  

## MOVE-IT HACKATHON open source code links
continue...

## Autoware workflow
![](./images/autoware_workflow.png)

## How to install Autoware
[Autoware install method](https://github.com/CPFL/Autoware)

If there are some error when you use ndt_matching with GPU cuda computer:
* Download Autoware develop branch
* Remove the following folder.
`Autoware/ros/src/sensing/fusion/packages/autoware_camera_lidar_calibrator`
* Compile Autoware develop branch.

## Lidar Velodyne 32C
Velodyne 32C is a new Lidar while driver and calibration file should be updated.

These files has been saved to xxx

### launch file
Copy Velodyne 32C launch file to the following address:
`Autoware/ros/src/sensing/drivers/lidar/packages/velodyne/velodyne_pointcloud/launch/32c_points.launch`

### calibration file
Copy Velodyne 32C calibration yaml file to the following address:
`Autoware/ros/src/sensing/drivers/lidar/packages/velodyne/velodyne_pointcloud/params/VLP-32C.yaml`

### driver
Copy Velodyne 32C driver cc file to the following address:
`Autoware/ros/src/sensing/drivers/lidar/packages/velodyne/velodyne_driver/src/driver/driver.cc`

### factor distance to resolution  
Velodyne 32C factor distance to resolution is different from others Velodyne products, this factor should be modified as following:

1. driver file  

```
gedit Autoware/ros/src/sensing/drivers/lidar/packages/velodyne/velodyne_pointcloud/src/lib/rawdata.cc
```

2. Change
`float distance = tmp.uint * DISTANCE_RESOLUTION;` to
`float distance = tmp.uint * 0.004;`

## How to connect Velodyne Lidar  
* Install ros-velodyne driver:

```
sudo apt-get install ros-VERSION-velodyne
```
* Connect velodyne 32C and disconnect wifi
* Velodyne 32C IP setting
* Robot Cafe car: 192.168.1.201
* Civic: 192.168.0.201
* Computer IP set with in Lidar net, e.g. (robot cafe car 192.168.1.100; Civic 192.168.0.100)  
* View Lidar data

```
roslaunch velodyne_pointcloud 32c_points.launch
rosrun rviz rviz -f velodyne
```
[ROS Velodyne driver install and connect guide](http://wiki.ros.org/velodyne/Tutorials/Getting%20Started%20with%20th)

The computer ip should be set in Lidar net but last not 201 (which is for Lidar).

The Lidars ip can be set same ip.

## How to record rosbag
Make sure you have connected with Lidar successfully and have enough free disk space.
```
roslaunch velodyne_pointcloud 32c_points.launch
rosrun rviz rviz -f velodyne
rosbag record -a
```

## How to generate map and waypoints
* Load simulation rosbag file and pause.
* Downsample rosbag files.
* Change downsample rosbag message name to /points_raw
* Active ndt_localizer | ndt_mapping
* waypoint_maker | waypoint_saver
* Run whole simulation rosbag
* Output pcb
* Save waypoints

## How to Simulate
 Here is simulation with generated pcb and waypoints file.
 * Setup, sensing
 * Load Map load pcb file
 * Computing setting
 * Rviz to simulate
 * If error please check every settings and redo it again.

![](./images/sim1.png)
![](./images/sim2.png)

## How to make self-driving car follow waypoints

![](./images/follow1.png)

## How to detect obstacle with Lidar
![](./images/obstacle.png)

## Topics in the future
### Autoware Lidar obstacle detection failure on upslope.
Autoware Lidar obstacle detection function will false detect upslope as obstacle and don't move.

### Police gestures detection
* The dataset should be big and diversity enough to prevent deep learning model over fitting.
* LSTM model has been used instead of CNN model to consider time serial.  
* Police gesture detection have been localized for different countries.

### The robot cafe car CAN control
Through we have control robot cafe car through CAN, the driving performance is really bad, especially on upslope.

There are still a lot of improvement potential for vehicle OEM and Tier1 e.g. Bosch to do in the hardware and CAN control.

## Reference
* [Autoware](https://github.com/CPFL/Autoware)
* [Apollo](https://github.com/ApolloAuto/apollo)
* [Busmaster](https://github.com/rbei-etas/busmaster)
* [comma.ai panda](https://github.com/commaai/panda)
* [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam)

## License
The specific code is distributed under MIT License.
