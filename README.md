# skid_robot
1, Tai cac goi can thiet
  
    - nav2: sudo apt install ros-humble-navigation2, sudo apt install ros-humble-nav2-bringup
   
    - mapping: sudo apt install ros-humble-cartographer, sudo apt install ros-humble-cartographer-ros
    
    - velodyne: https://github.com/ros-drivers/velodyne.git 
   
    - imu: https://github.com/LORD-MicroStrain/microstrain_inertial.git

2, Chinh cac dan 
   
    - trong cac file launch cua package data, mapping, va nav2, chinh lai cac duong dan sao cho phu hop moi may

3, Huong dan chay
   
    * mapping:
      
        - chay file launch data: ros2 launch data data_launch_2.py
      
        - chay file launch cartographer: ros2 launch mapping cartographer_velodyne.launch.py
   
    * nav2: 
       
        - chay file launch data: ros2 launch data data_launch_2.py
      
        - chay file launch nav2: ros2 launch nav2 amcl_launch.py
