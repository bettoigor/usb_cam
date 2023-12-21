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


![Configurator](/doc/img_1.png)