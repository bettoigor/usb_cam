# USB Generic camera
ROS driver for generic USB camera

## Installing the driver
Clone this repository to your ROS workspace and run ```catkin_make``` commando  to compile the package.

## Configuration parameters
To configure the camera parameters, edit the file [usb_cam_params.yaml](https://github.com/bettoigor/usb_cam/blob/master/cfg/usb_cam_params.yaml)
The following parameters can be changed:
```
usb_cam:
  video:
    video_source: 0
    frame_width: 640
    frame_height: 480
  control:
    rate: 30
    show_img: true
```

## Running the node
This node can be used separated or as parte of other launch file. To run the node, execute 
the following command\:

```
roslaunch usb_cam camera.launch 
```

After perform the configuration, the node can be launch as following:
```
rosrun image_view image_view image:= soybot_model soybot_model.launch
```
and the image can be seen:


![Configurator](/doc/img_1.jpg)

## Checking the Environment
### Checking the published topics
To check all the topics type:

```
rostopic list
```

### Send control commands to the robot
The robot is controlled using the topic cmd_vel, that can be tested using the following command:
```
rostopic pub -1 /soybot/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: -0.0"
```
where x and z are the linear and angular velocities, respectively.

### Show image
It is possible to see the images generated by the simulator using ros image view:
```
rosrun image_view image_view image:=image_topic
```
where ```image_topic``` is the desired topic.

