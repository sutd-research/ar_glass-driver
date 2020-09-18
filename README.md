# ROS Driver for AR Glass

This driver provides a ROS interface to communicate to AR Glass. Images from the glass can be obtained through a ROS service. Images can be sent to the glass by publishing to the topic mentioned below. 


## Dependencies
* ROS
* Python 3.5 / 3.6
* httpserver

**Dependency Installation** <br /> 
>      pip install httpserver

#### Subscribed Topics
- sensor_msgs/Image : /image_receiver
  
#### Services
- ar_glass/Image: /capture

#### Note:
>  The service and topic names are configured in the /config/parameters.yaml file. User can change the names by editing the file.

Compile and run the package
>       roslaunch ar_glass ar_glass.launch

#### Permissions:
It might be required to provide execution permissions to the python executable.
>       sudo chmod +x ar_glass/scripts/ar_glass_driver.py


