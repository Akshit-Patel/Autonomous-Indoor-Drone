# Autonomous-Indoor-Drone

**Instructions for setup**

1.) Setup px4-Firmware

2.) Copy-Paste following files to your Firmware

a) Simulator/config/10017_aeroBITS -> Firmware/ROMFS/px4fmu_common/init.d-posix/

b) Simulator/launch/aeroBITS.launch -> Firmware/launch/

c) Simulator/models/aeroBITS and Simulaor/models/aeroBITS_sensors/aeroBITS_sensors -> Firmware/Tools/sitl_gazebo/models/

d) Simulator/worlds/flipkart.world and Simulator/worlds/flipkart_lite.world -> Firmware/Tools/sitl_gazebo/worlds/

e) Simulator/models/meshes/aeroBITS.stl -> Firmware/Tools/sitl_gazebo/models/rotors_description/meshes/

**To quickly test model**

a) `roslaunch px4 posix_sitl.launch vehicle:=aeroBITS`

Team custom droneshould be visible in gazebo custom world. You may hover it around using QgroundControl

b) If `GAZEBO_MODEL_PATH` is correctly setup than you may launch it, and try drag and drop that model

**To launch model into Flipkart World**

a) `roslaunch px4 aeroBITS.launch`

**QGroundControl Camera Feed**

1.) Click on Q icon

2.) Select general in application setting

    a) Video source - UDP h.264 Video Stream
    b) UDP port 5600
    c) Aspect Ratio 1.777777

3.) Select Autodelete files if space issues 

 
**To Do -**

1.) More detailed instructions on launching Firmware

2.) Change Name of stl file

3.) Instructions to launch flipkart world with more than 3 gates

4.) Update Camera position on drone- Some issues

5.) Make readme clean
