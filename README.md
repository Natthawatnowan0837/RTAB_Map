# RTAB_Map
```bash
ros2 launch realsense2_camera rs_launch.py     enable_color:=true     enable_depth:=true     enable_gyro:=true     enable_accel:=true     unite_imu_method:=2     pointcloud.enable:=false     rgb_width:=640 rgb_height:=480     depth_width:=640 depth_height:=480     fps:=30

ros2 launch rtabmap_launch rtabmap.launch.py approx_sync:=true rtabmap_viz:=false rviz:=true