# kinect_bringup
## Installation
Follow this [process](http://demura.net/%E6%9C%AA%E5%88%86%E9%A1%9E/13560.html) to install `libfreenect2` and `iai_kinect2`.
## Initial build
```
catkin build -DCMAKE_BUILD_TYPE=”Release”  -Dfreenect2_DIR=~/src/freenect2/lib/cmake/freenect2
```
## Running single kinect
```
roscore
```
```
roslaunch kinect2_bridge kinect2_bridge.launch base_name:=kinect_center
```
```
roslaunch kinect_bringup kinect_center_streaming.launch
```
### depth_method:=cpu
```
roslaunch kinect2_bridge kinect2_bridge.launch base_name:=kinect_center depth_method:=cpu
```
###  extrinsic calibration
```
rosrun kinect_bringup tf_interactive_marker.py world kinect_test 1.4784 0.0693 0.4358 0.046 0.4315 -3.1227
```
#  USAGE(dataset generator)
1.launch gazebo and rviz
```
roslaunch kinect_bringup empty_world.launch
```
1.5. delete ground plane (manual)
```
```
2.turn on gravity on gazebo
```
rosservice call /gazebo/set_physics_properties "
time_step: 0.001
max_update_rate: 1000.0
gravity:
  x: 0.0
  y: 0.0
  z: 0.0
ode_config:
  auto_disable_bodies: False
  sor_pgs_precon_iters: 0
  sor_pgs_iters: 50
  sor_pgs_w: 1.3
  sor_pgs_rms_error_tol: 0.0
  contact_surface_layer: 0.001
  contact_max_correcting_vel: 100.0
  cfm: 0.0
  erp: 0.2
  max_contacts: 20"
```
3.spown the sensor on gazebo and rviz
```
roscd kinect_bringup/urdf/models
rosrun gazebo_ros spawn_model -urdf -file kinect.urdf -model kinect -x 1.0 -y 0.0 -z 0.5 -R 0.0 -P 0.35 -Y 3.14
rosrun kinect_bringup tf_camera.py world kinect 1 0 0.5 0 0.35 3.14
```
4.spown target object you want to collect for datasets
```
roscd kinect_bringup/urdf/models
rosrun gazebo_ros spawn_model -urdf -file object.urdf -model kobject -x 0 -y 0 -z 0.05 -R 0.0 -P 0.582 -Y 0.0
```
5.generating object pose automatically
```
rosrun kinect_bringup random_move_euler.py
```
or
```
rosrun kinect_bringup tf_state_publisher.py world kobject 0.1 0.1 0.1 0 0 0
```
6.record object's point cloud and pose
```
roscd kinect_bringup/data
mkdir data1 ; cd data1
mkdir learn_data
rosrun kinect_bringup record_euler input:=/kinect/sd/points number_of_data
```
7.normalizing point cloud and make voxel
```
roscd kinect_bringup/data/data1
rosrun kinect_bringup make_voxel_data number_of_data
```
8.save dataset in hdf5 format
```
roscd kinect_bringup/data/data1
rosrun kinect_bringup from_csv2hdf.py -n number of dataset
```
##  verification of precision
```
roslaunch kinect_bringup object.launch
```
```
rosrun kinect_bringup tf_interactive_marker_object.py world object_origin 0.1 0.1 0.1 0 0 0
```
```
rosrun tf tf_echo estimated_tf object_origin
```


#  USAGE(getting registration upside model)
1.launch gazebo and rviz
```
roslaunch kinect_bringup empty_world.launch
```
1.5. delete ground plane (manual)

2.turn on gravity on gazebo
```
rosservice call /gazebo/set_physics_properties "
time_step: 0.001
max_update_rate: 1000.0
gravity:
  x: 0.0
  y: 0.0
  z: 0.0
ode_config:
  auto_disable_bodies: False
  sor_pgs_precon_iters: 0
  sor_pgs_iters: 50
  sor_pgs_w: 1.3
  sor_pgs_rms_error_tol: 0.0
  contact_surface_layer: 0.001
  contact_max_correcting_vel: 100.0
  cfm: 0.0
  erp: 0.2
  max_contacts: 20"
```
3.delete grand_plane in gazebo

4.spown the sensor on gazebo and rviz
```
roscd kinect_bringup/urdf/models
rosrun gazebo_ros spawn_model -urdf -file kinect.urdf -model kinect -x 0 -y 0 -z 0.5 -R 0 -P 1.5708 -Y 0
rosrun kinect_bringup tf_camera.py world kinect 0 0 0.5 0 1.5708 0
```
5.spown target object you want to collect for datasets
```
roscd kinect_bringup/urdf/models
rosrun gazebo_ros spawn_model -urdf -file hv8.urdf -model kobject -x 0 -y 0 -z 0 -R 0.0 -P 0.0 -Y 0.0
```
5.save model cloud at kinect_bringup/pcds
```
rosrun kinect_bringup record_pc
```
