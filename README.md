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
###  dataset generate
```
roslaunch gazebo_ros empty_world.launch
```
```
rviz
```
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
```
roscd kinect_bringup/urdf/models
rosrun gazebo_ros spawn_model -urdf -file kinect.urdf -model kinect 1 0 0.5 0 0.35 3.14
rosrun kinect_bringup tf_camera.py world kinect 1 0 0.5 0 0.35 3.14
```
集めたい対象物体のurdfを指定する
```
roscd kinect_bringup/urdf/models
rosrun gazebo_ros spawn_model -urdf -file object.urdf -model kobject -x 0 -y 0 -z 0.05 -r 0.0 -p 0.582 -y 0.0
```
```
rosrun kinect_bringup random_move_euler.py
```
```
roscd /kinect_bringup/data
mkdir data1 || cd data1
rosrun kinect_bringup record input:=/kinect/sd/points number_of_data
```
make dataset(Normalized voxel and centroid) after record pcd files
```
roscd kinect_bringup/data/data1
rosrun kinect_bringup make_voxel_data number_of_data
```
```
roscd kinect_bringup/data/data1
rosrun kinect_bringup from_csv2hdf.py -n number of dataset
```
