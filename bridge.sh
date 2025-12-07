ros2 run ros_gz_bridge parameter_bridge \
  /camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  /camera/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /camera/image/depth@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /camera2/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
  /camera2/image@sensor_msgs/msg/Image@gz.msgs.Image \
  /camera2/image/depth@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan \
  /lidar/points@sensor_msgs/msg/PointCloud2@gz.msgs.PointCloudPacked \
  /tf@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V \
  /world/default/dynamic_pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V \
  /world/default/pose/info@tf2_msgs/msg/TFMessage@gz.msgs.Pose_V \
  /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
