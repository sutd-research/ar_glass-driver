# ROS Driver for AR Glass

This driver provides a ROS interface to communicate to AR Glass. Images from the glass can be obtained through a ROS service. Images can be sent to the glass by publishing to the topic mentioned below. 


## Dependencies
* ROS
* httpserver

Tested on Ubuntu 16.04 ROS Kinetic. Not interfaced with HTTP communication yet.

**Dependency Installation** <br /> 
>      pip install httpserver

#### Subscribed Topics
- sensor_msgs/Image : /AR_Send_Image
  
#### Services
- ar_glass/Image: /AR_Take_Image

#### Note:
>  The service and topic names are configured in the /config/parameters.yaml file. User can change the names by editing the file.

Compile and run the package
>       roslaunch ar_glass ar_glass.launch

#### Permissions:
It might be required to provide execution permissions to the python executable.
>       sudo chmod +x ar_glass/scripts/ar_glass_driver.py


