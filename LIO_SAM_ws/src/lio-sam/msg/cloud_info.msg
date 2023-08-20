# Cloud Info
Header header

int32[] start_ring_index
int32[] end_ring_index

int32[] point_column_index  # point column index in range image
int32[] point_range  # point range 

int64 imu_available
int64 odom_available

# Attitude for LOAM initialization
float32 imu_roll_init
float32 imu_pitch_init
float32 imu_yaw_init

# Initial guess from imu pre-integration
float32 init_guess_x
float32 init_guess_y
float32 init_guess_z
float32 init_guess_roll
float32 init_guess_pitch
float32 init_guess_yaw

# Point cloud messages
sensor_msgs/PointCloud2 cloud_deskewed 
sensor_msgs/PointCloud2 cloud_corner
sensor_msgs/PointCloud2 cloud_surface

# 3rd party messages
sensor_msgs/PointCloud2 keyframe_cloud
sensor_msgs/PointCloud2 keyframe_color
sensor_msgs/PointCloud2 keyframe_poses
sensor_msgs/PointCloud2 keyframe_map