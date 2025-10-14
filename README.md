# RTAB_Map
```bash
ros2 launch rtabmap_launch rtabmap.launch.py     approx_sync:=true     rtabmap_viz:=true     rviz:=true     rgb_topic:=/camera/color/image_raw     depth_topic:=/camera/aligned_depth_to_color/image_raw     camera_info_topic:=/camera/color/camera_info     frame_id:=camera_link     qos:=2     topic_queue_size:=30     sync_queue_size:=30

ros2 launch realsense2_camera rs_launch.py   align_depth:=true   enable_color:=true   enable_depth:=true   pointcloud.enable:=false   depth_module.profile:=848x480x30   rgb_camera.profile:=848x480x30   enable_gyro:=false   enable_accel:=false   unite_imu_method:=0

colcon build --parallel-workers 1
