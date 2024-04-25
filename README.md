# IMU_Sesnsor_MPU9250
This is regarding caluculating the orientation and caliberating the IMU Senosr MPU9250 through ROS Rviz application using IMU PLugin which can be installed using "_sudo apt-get install ros-noetic-imu-tools_"

- The box_description folder is just ordinary URDf file generated directly from the Fusion 360 using the fusion2urdf plugin. The dimesion of the box is not that much of a concern.

- The MPU9250_Code is the Arduino Code which needs to be uploaded in the Arduino Boeard the code only gives the Acclerometer and Gyroscope values for Magnetometer and further information about the MPU9250
  code, you can use this github link as a reference "https://github.com/kriswiner/MPU9250/blob/master/MPU9250BasicAHRS.ino".

- Inside the box_description folder there will be a python file called "imu_listener.py" which takes in the angular velocity values from the Gyroscope to caluculate the Orientation values.

- To connect the Arduino with the ROS you need to use few commands, you need to open multiple terminals.
    - Terminal 1 "roscore"
    - Terminal 2 "sudo chmod a+rw /dev/ttyACM0" This gives the permission to the specific USB channel that is connected to the Arduino "ttyACM0" can wary
    - Terminal 3 "_rosrun rosserial_python serila_node.py _port:=/dev/ttyACM0 baud:= 115200" this is the command which helps in sending the information from ROS to the Arduino
    - Terminal 4 "roslaunch box_description display.launch" in this you need remove the robotmodel present in the rviz application and add the imu plugin where a box visulaization is already present 
    - Terminal 5 "rosrun box_desription imu_listener.py" initializes the python to accept the anngular velocity values to calculate the orientation values.
    - Terminal 6 "rostopic echo /imu_data_q" is the topic written in the python file where we can get all the values that is linear accleration , angular velocities and the orientation.
 
- One important after adding the plugin in the Rviz you need to make it gets connected with a topic here the topic is "/imu_data_q"
