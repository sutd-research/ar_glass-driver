# ROS Driver for AR Glass

This driver provides a ROS interface to communicate with AR Glass. Images from the glass can be obtained through a ROS service. Bounding Box information can be sent to the glass by publishing to the topic mentioned below. 


## Dependencies
* ROS
* Python 2.7x
* Numpy

Tested on Ubuntu 16.04 ROS Kinetic

**Compiling** <br /> 
* Copy the package into catkin workspace. 
    > (For information regading ROS installation and creating a catkin workspace, please refer ROS installation tutorials)
* Compile the package
    ```
    # Open a terminal and go to catkin workspace directory
    cd ~/catkin_ws

    # Compile the package
    catkin_make --only-pkg-with-deps ar_glass
    ```

* Configure IP Address of the AR Glass
    * The IP address is configured within /config/parameters.yaml 
    * Find the IP address of the AR Glass in the local network 
        > The AR Glass and the host computer running this ROS package should be in the same network
    * Edit the 'server' parameter to match the IP address of the AR Glass. <br />Example:
        > server: '192.168.1.141'


* Run the package
    ```
    roslaunch ar_glass ar_glass.launch
    ```


#### Subscribed Topics
- ar_glass/BoundingBox : /AR_Send_Image

    ```
    std_msgs/Header header
        uint32 seq
        time stamp
        string frame_id
    int32 left_x
    int32 right_x
    int32 top_y
    int32 bottom_y
    ```
  
#### Services
- ar_glass/Image: /AR_Take_Image

#### Note:
>  The service and topic names are configured in the /config/parameters.yaml file. User can change the names by editing the file.

#### Permissions:
It might be required to provide execution permissions to the python executable.
>       sudo chmod +x ar_glass/scripts/ar_glass_driver.py


