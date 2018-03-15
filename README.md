# Autoware Tutorial for Self-driving Car
## 2018 GUIYANG PIX MOVE-IT HACKATHON

## 2018 GUIYANG MOVE-IT HACKATHON INTRODUCTION
2018 GUIYANG MOVE-IT HACKATHON is the first open source hackathon workshop to make real size self-driving car in China organized by a GUIYANG startup company PIX.

There are two cars, one is robot cafe and another is Honda Civic.

[Moveit introduction](https://www.pixmoving.com/move-it)
![](./images/moveit.png)

![](./images/moveit_all.jpg)

## What we implemented
### Robot  car
* Can control robot cafe car throttle, brake and steer
* Generate map and waypoints
* Make robot cafe car self-driving follow waypoints
* Detect obstacle with Lidar

![](./images/robotcafe.jpeg)

### Honda Civic
* CAN control Civic throttle, brake and steer through comma.ai panda.
* CAN control Civic under low speed.

There are a lot of comma.ai implementation in China with Honda Civic but can not control steer under 20km/h.

We completely control Civic even under low speed.

It is pity for limited time we did not totally make Civic self-driving but easy to continue.
![](./images/civic.jpeg)

### Police gesture detection
* Set up the first open source Chinese Police gesture dataset.
* Train a model through transfer learning with a pre-trained inception model

![](./images/gesture.jpeg)

The Chinese police gesture detection by camera performance is not excellent due to dataset reason and limited time.

More information about [Police gestures dataset and detection is doing]()  

## MOVE-IT HACKATHON open source code links
continue...

## Autoware workflow
![](./images/autoware_workflow.png)

## How to install Autoware
[Autoware install method](https://github.com/CPFL/Autoware)

If there are some error when you use ndt_matching with GPU computer:
* Download Autoware develop branch
* Remove the following folder.
/ros/src/sensing/fusion/packages/autoware_camera_lidar_calibrator
* Compile Autoware develop branch.

## Lidar Velodyne 32C
Velodyne 32C is a new Lidar while driver and calibration file has been updated.

These files has been saved to

### launch file
### calibration file
### driver
### factor distance to resolution  

##  How to connect Lidar  

## How to record rosbag

## How to generate map and waypoints

## How to Simulate

## How to make self-driving car follow waypoints

## How to detect obstacle with Lidar

## Topics in the future
### Autoware Lidar obstacle detection failure on upslope.
Autoware Lidar obstacle detection function will wrongly detect upslope as obstacle and don't move.

### Police gestures detection
* The dataset should be big and diversity enough to prevent deep learning model over fitting.
* LSTM model has been used instead of CNN model to consider time serial.  
* Police gesture detection have been localized for different countries.

### The robot cafe car CAN control
Through we have control robot cafe car through CAN, the driving performance is really bad.

There are still a lot of improvement opportunities for vehicle OEM and Tier1 e.g. Bosch to do in the hardware and CAN control.

## Reference
* [Autoware](https://github.com/CPFL/Autoware)
* [Apollo](https://github.com/ApolloAuto/apollo)
* [Busmaster](https://github.com/rbei-etas/busmaster)
* [comma.ai panda](https://github.com/commaai/panda)
* [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam)

## License
The specific code is distributed under MIT License.
