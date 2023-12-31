;; Auto-generated. Do not edit!


(when (boundp 'lio_sam::cloud_info)
  (if (not (find-package "LIO_SAM"))
    (make-package "LIO_SAM"))
  (shadow 'cloud_info (find-package "LIO_SAM")))
(unless (find-package "LIO_SAM::CLOUD_INFO")
  (make-package "LIO_SAM::CLOUD_INFO"))

(in-package "ROS")
;;//! \htmlinclude cloud_info.msg.html
(if (not (find-package "SENSOR_MSGS"))
  (ros::roseus-add-msgs "sensor_msgs"))
(if (not (find-package "STD_MSGS"))
  (ros::roseus-add-msgs "std_msgs"))


(defclass lio_sam::cloud_info
  :super ros::object
  :slots (_header _start_ring_index _end_ring_index _point_column_index _point_range _imu_available _odom_available _imu_roll_init _imu_pitch_init _imu_yaw_init _init_guess_x _init_guess_y _init_guess_z _init_guess_roll _init_guess_pitch _init_guess_yaw _cloud_deskewed _cloud_corner _cloud_surface _keyframe_cloud _keyframe_color _keyframe_poses _keyframe_map ))

(defmethod lio_sam::cloud_info
  (:init
   (&key
    ((:header __header) (instance std_msgs::Header :init))
    ((:start_ring_index __start_ring_index) (make-array 0 :initial-element 0 :element-type :integer))
    ((:end_ring_index __end_ring_index) (make-array 0 :initial-element 0 :element-type :integer))
    ((:point_column_index __point_column_index) (make-array 0 :initial-element 0 :element-type :integer))
    ((:point_range __point_range) (make-array 0 :initial-element 0 :element-type :integer))
    ((:imu_available __imu_available) 0)
    ((:odom_available __odom_available) 0)
    ((:imu_roll_init __imu_roll_init) 0.0)
    ((:imu_pitch_init __imu_pitch_init) 0.0)
    ((:imu_yaw_init __imu_yaw_init) 0.0)
    ((:init_guess_x __init_guess_x) 0.0)
    ((:init_guess_y __init_guess_y) 0.0)
    ((:init_guess_z __init_guess_z) 0.0)
    ((:init_guess_roll __init_guess_roll) 0.0)
    ((:init_guess_pitch __init_guess_pitch) 0.0)
    ((:init_guess_yaw __init_guess_yaw) 0.0)
    ((:cloud_deskewed __cloud_deskewed) (instance sensor_msgs::PointCloud2 :init))
    ((:cloud_corner __cloud_corner) (instance sensor_msgs::PointCloud2 :init))
    ((:cloud_surface __cloud_surface) (instance sensor_msgs::PointCloud2 :init))
    ((:keyframe_cloud __keyframe_cloud) (instance sensor_msgs::PointCloud2 :init))
    ((:keyframe_color __keyframe_color) (instance sensor_msgs::PointCloud2 :init))
    ((:keyframe_poses __keyframe_poses) (instance sensor_msgs::PointCloud2 :init))
    ((:keyframe_map __keyframe_map) (instance sensor_msgs::PointCloud2 :init))
    )
   (send-super :init)
   (setq _header __header)
   (setq _start_ring_index __start_ring_index)
   (setq _end_ring_index __end_ring_index)
   (setq _point_column_index __point_column_index)
   (setq _point_range __point_range)
   (setq _imu_available (round __imu_available))
   (setq _odom_available (round __odom_available))
   (setq _imu_roll_init (float __imu_roll_init))
   (setq _imu_pitch_init (float __imu_pitch_init))
   (setq _imu_yaw_init (float __imu_yaw_init))
   (setq _init_guess_x (float __init_guess_x))
   (setq _init_guess_y (float __init_guess_y))
   (setq _init_guess_z (float __init_guess_z))
   (setq _init_guess_roll (float __init_guess_roll))
   (setq _init_guess_pitch (float __init_guess_pitch))
   (setq _init_guess_yaw (float __init_guess_yaw))
   (setq _cloud_deskewed __cloud_deskewed)
   (setq _cloud_corner __cloud_corner)
   (setq _cloud_surface __cloud_surface)
   (setq _keyframe_cloud __keyframe_cloud)
   (setq _keyframe_color __keyframe_color)
   (setq _keyframe_poses __keyframe_poses)
   (setq _keyframe_map __keyframe_map)
   self)
  (:header
   (&rest __header)
   (if (keywordp (car __header))
       (send* _header __header)
     (progn
       (if __header (setq _header (car __header)))
       _header)))
  (:start_ring_index
   (&optional __start_ring_index)
   (if __start_ring_index (setq _start_ring_index __start_ring_index)) _start_ring_index)
  (:end_ring_index
   (&optional __end_ring_index)
   (if __end_ring_index (setq _end_ring_index __end_ring_index)) _end_ring_index)
  (:point_column_index
   (&optional __point_column_index)
   (if __point_column_index (setq _point_column_index __point_column_index)) _point_column_index)
  (:point_range
   (&optional __point_range)
   (if __point_range (setq _point_range __point_range)) _point_range)
  (:imu_available
   (&optional __imu_available)
   (if __imu_available (setq _imu_available __imu_available)) _imu_available)
  (:odom_available
   (&optional __odom_available)
   (if __odom_available (setq _odom_available __odom_available)) _odom_available)
  (:imu_roll_init
   (&optional __imu_roll_init)
   (if __imu_roll_init (setq _imu_roll_init __imu_roll_init)) _imu_roll_init)
  (:imu_pitch_init
   (&optional __imu_pitch_init)
   (if __imu_pitch_init (setq _imu_pitch_init __imu_pitch_init)) _imu_pitch_init)
  (:imu_yaw_init
   (&optional __imu_yaw_init)
   (if __imu_yaw_init (setq _imu_yaw_init __imu_yaw_init)) _imu_yaw_init)
  (:init_guess_x
   (&optional __init_guess_x)
   (if __init_guess_x (setq _init_guess_x __init_guess_x)) _init_guess_x)
  (:init_guess_y
   (&optional __init_guess_y)
   (if __init_guess_y (setq _init_guess_y __init_guess_y)) _init_guess_y)
  (:init_guess_z
   (&optional __init_guess_z)
   (if __init_guess_z (setq _init_guess_z __init_guess_z)) _init_guess_z)
  (:init_guess_roll
   (&optional __init_guess_roll)
   (if __init_guess_roll (setq _init_guess_roll __init_guess_roll)) _init_guess_roll)
  (:init_guess_pitch
   (&optional __init_guess_pitch)
   (if __init_guess_pitch (setq _init_guess_pitch __init_guess_pitch)) _init_guess_pitch)
  (:init_guess_yaw
   (&optional __init_guess_yaw)
   (if __init_guess_yaw (setq _init_guess_yaw __init_guess_yaw)) _init_guess_yaw)
  (:cloud_deskewed
   (&rest __cloud_deskewed)
   (if (keywordp (car __cloud_deskewed))
       (send* _cloud_deskewed __cloud_deskewed)
     (progn
       (if __cloud_deskewed (setq _cloud_deskewed (car __cloud_deskewed)))
       _cloud_deskewed)))
  (:cloud_corner
   (&rest __cloud_corner)
   (if (keywordp (car __cloud_corner))
       (send* _cloud_corner __cloud_corner)
     (progn
       (if __cloud_corner (setq _cloud_corner (car __cloud_corner)))
       _cloud_corner)))
  (:cloud_surface
   (&rest __cloud_surface)
   (if (keywordp (car __cloud_surface))
       (send* _cloud_surface __cloud_surface)
     (progn
       (if __cloud_surface (setq _cloud_surface (car __cloud_surface)))
       _cloud_surface)))
  (:keyframe_cloud
   (&rest __keyframe_cloud)
   (if (keywordp (car __keyframe_cloud))
       (send* _keyframe_cloud __keyframe_cloud)
     (progn
       (if __keyframe_cloud (setq _keyframe_cloud (car __keyframe_cloud)))
       _keyframe_cloud)))
  (:keyframe_color
   (&rest __keyframe_color)
   (if (keywordp (car __keyframe_color))
       (send* _keyframe_color __keyframe_color)
     (progn
       (if __keyframe_color (setq _keyframe_color (car __keyframe_color)))
       _keyframe_color)))
  (:keyframe_poses
   (&rest __keyframe_poses)
   (if (keywordp (car __keyframe_poses))
       (send* _keyframe_poses __keyframe_poses)
     (progn
       (if __keyframe_poses (setq _keyframe_poses (car __keyframe_poses)))
       _keyframe_poses)))
  (:keyframe_map
   (&rest __keyframe_map)
   (if (keywordp (car __keyframe_map))
       (send* _keyframe_map __keyframe_map)
     (progn
       (if __keyframe_map (setq _keyframe_map (car __keyframe_map)))
       _keyframe_map)))
  (:serialization-length
   ()
   (+
    ;; std_msgs/Header _header
    (send _header :serialization-length)
    ;; int32[] _start_ring_index
    (* 4    (length _start_ring_index)) 4
    ;; int32[] _end_ring_index
    (* 4    (length _end_ring_index)) 4
    ;; int32[] _point_column_index
    (* 4    (length _point_column_index)) 4
    ;; int32[] _point_range
    (* 4    (length _point_range)) 4
    ;; int64 _imu_available
    8
    ;; int64 _odom_available
    8
    ;; float32 _imu_roll_init
    4
    ;; float32 _imu_pitch_init
    4
    ;; float32 _imu_yaw_init
    4
    ;; float32 _init_guess_x
    4
    ;; float32 _init_guess_y
    4
    ;; float32 _init_guess_z
    4
    ;; float32 _init_guess_roll
    4
    ;; float32 _init_guess_pitch
    4
    ;; float32 _init_guess_yaw
    4
    ;; sensor_msgs/PointCloud2 _cloud_deskewed
    (send _cloud_deskewed :serialization-length)
    ;; sensor_msgs/PointCloud2 _cloud_corner
    (send _cloud_corner :serialization-length)
    ;; sensor_msgs/PointCloud2 _cloud_surface
    (send _cloud_surface :serialization-length)
    ;; sensor_msgs/PointCloud2 _keyframe_cloud
    (send _keyframe_cloud :serialization-length)
    ;; sensor_msgs/PointCloud2 _keyframe_color
    (send _keyframe_color :serialization-length)
    ;; sensor_msgs/PointCloud2 _keyframe_poses
    (send _keyframe_poses :serialization-length)
    ;; sensor_msgs/PointCloud2 _keyframe_map
    (send _keyframe_map :serialization-length)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; std_msgs/Header _header
       (send _header :serialize s)
     ;; int32[] _start_ring_index
     (write-long (length _start_ring_index) s)
     (dotimes (i (length _start_ring_index))
       (write-long (elt _start_ring_index i) s)
       )
     ;; int32[] _end_ring_index
     (write-long (length _end_ring_index) s)
     (dotimes (i (length _end_ring_index))
       (write-long (elt _end_ring_index i) s)
       )
     ;; int32[] _point_column_index
     (write-long (length _point_column_index) s)
     (dotimes (i (length _point_column_index))
       (write-long (elt _point_column_index i) s)
       )
     ;; int32[] _point_range
     (write-long (length _point_range) s)
     (dotimes (i (length _point_range))
       (write-long (elt _point_range i) s)
       )
     ;; int64 _imu_available
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _imu_available (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _imu_available) (= (length (_imu_available . bv)) 2)) ;; bignum
              (write-long (ash (elt (_imu_available . bv) 0) 0) s)
              (write-long (ash (elt (_imu_available . bv) 1) -1) s))
             ((and (class _imu_available) (= (length (_imu_available . bv)) 1)) ;; big1
              (write-long (elt (_imu_available . bv) 0) s)
              (write-long (if (>= _imu_available 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _imu_available s)(write-long (if (>= _imu_available 0) 0 #xffffffff) s)))
     ;; int64 _odom_available
#+(or :alpha :irix6 :x86_64)
       (progn (sys::poke _odom_available (send s :buffer) (send s :count) :long) (incf (stream-count s) 8))
#-(or :alpha :irix6 :x86_64)
       (cond ((and (class _odom_available) (= (length (_odom_available . bv)) 2)) ;; bignum
              (write-long (ash (elt (_odom_available . bv) 0) 0) s)
              (write-long (ash (elt (_odom_available . bv) 1) -1) s))
             ((and (class _odom_available) (= (length (_odom_available . bv)) 1)) ;; big1
              (write-long (elt (_odom_available . bv) 0) s)
              (write-long (if (>= _odom_available 0) 0 #xffffffff) s))
             (t                                         ;; integer
              (write-long _odom_available s)(write-long (if (>= _odom_available 0) 0 #xffffffff) s)))
     ;; float32 _imu_roll_init
       (sys::poke _imu_roll_init (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _imu_pitch_init
       (sys::poke _imu_pitch_init (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _imu_yaw_init
       (sys::poke _imu_yaw_init (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _init_guess_x
       (sys::poke _init_guess_x (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _init_guess_y
       (sys::poke _init_guess_y (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _init_guess_z
       (sys::poke _init_guess_z (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _init_guess_roll
       (sys::poke _init_guess_roll (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _init_guess_pitch
       (sys::poke _init_guess_pitch (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; float32 _init_guess_yaw
       (sys::poke _init_guess_yaw (send s :buffer) (send s :count) :float) (incf (stream-count s) 4)
     ;; sensor_msgs/PointCloud2 _cloud_deskewed
       (send _cloud_deskewed :serialize s)
     ;; sensor_msgs/PointCloud2 _cloud_corner
       (send _cloud_corner :serialize s)
     ;; sensor_msgs/PointCloud2 _cloud_surface
       (send _cloud_surface :serialize s)
     ;; sensor_msgs/PointCloud2 _keyframe_cloud
       (send _keyframe_cloud :serialize s)
     ;; sensor_msgs/PointCloud2 _keyframe_color
       (send _keyframe_color :serialize s)
     ;; sensor_msgs/PointCloud2 _keyframe_poses
       (send _keyframe_poses :serialize s)
     ;; sensor_msgs/PointCloud2 _keyframe_map
       (send _keyframe_map :serialize s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; std_msgs/Header _header
     (send _header :deserialize buf ptr-) (incf ptr- (send _header :serialization-length))
   ;; int32[] _start_ring_index
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _start_ring_index (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _start_ring_index i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _end_ring_index
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _end_ring_index (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _end_ring_index i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _point_column_index
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _point_column_index (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _point_column_index i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int32[] _point_range
   (let (n)
     (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4)
     (setq _point_range (instantiate integer-vector n))
     (dotimes (i n)
     (setf (elt _point_range i) (sys::peek buf ptr- :integer)) (incf ptr- 4)
     ))
   ;; int64 _imu_available
#+(or :alpha :irix6 :x86_64)
      (setf _imu_available (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _imu_available (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; int64 _odom_available
#+(or :alpha :irix6 :x86_64)
      (setf _odom_available (prog1 (sys::peek buf ptr- :long) (incf ptr- 8)))
#-(or :alpha :irix6 :x86_64)
      (setf _odom_available (let ((b0 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4)))
                  (b1 (prog1 (sys::peek buf ptr- :integer) (incf ptr- 4))))
              (cond ((= b1 -1) b0)
                     ((and (= b1  0)
                           (<= lisp::most-negative-fixnum b0 lisp::most-positive-fixnum))
                      b0)
                    ((= b1  0) (make-instance bignum :size 1 :bv (integer-vector b0)))
                    (t (make-instance bignum :size 2 :bv (integer-vector b0 (ash b1 1)))))))
   ;; float32 _imu_roll_init
     (setq _imu_roll_init (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _imu_pitch_init
     (setq _imu_pitch_init (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _imu_yaw_init
     (setq _imu_yaw_init (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _init_guess_x
     (setq _init_guess_x (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _init_guess_y
     (setq _init_guess_y (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _init_guess_z
     (setq _init_guess_z (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _init_guess_roll
     (setq _init_guess_roll (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _init_guess_pitch
     (setq _init_guess_pitch (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; float32 _init_guess_yaw
     (setq _init_guess_yaw (sys::peek buf ptr- :float)) (incf ptr- 4)
   ;; sensor_msgs/PointCloud2 _cloud_deskewed
     (send _cloud_deskewed :deserialize buf ptr-) (incf ptr- (send _cloud_deskewed :serialization-length))
   ;; sensor_msgs/PointCloud2 _cloud_corner
     (send _cloud_corner :deserialize buf ptr-) (incf ptr- (send _cloud_corner :serialization-length))
   ;; sensor_msgs/PointCloud2 _cloud_surface
     (send _cloud_surface :deserialize buf ptr-) (incf ptr- (send _cloud_surface :serialization-length))
   ;; sensor_msgs/PointCloud2 _keyframe_cloud
     (send _keyframe_cloud :deserialize buf ptr-) (incf ptr- (send _keyframe_cloud :serialization-length))
   ;; sensor_msgs/PointCloud2 _keyframe_color
     (send _keyframe_color :deserialize buf ptr-) (incf ptr- (send _keyframe_color :serialization-length))
   ;; sensor_msgs/PointCloud2 _keyframe_poses
     (send _keyframe_poses :deserialize buf ptr-) (incf ptr- (send _keyframe_poses :serialization-length))
   ;; sensor_msgs/PointCloud2 _keyframe_map
     (send _keyframe_map :deserialize buf ptr-) (incf ptr- (send _keyframe_map :serialization-length))
   ;;
   self)
  )

(setf (get lio_sam::cloud_info :md5sum-) "7490d670e17d4a2b9ad7d689d0925e1a")
(setf (get lio_sam::cloud_info :datatype-) "lio_sam/cloud_info")
(setf (get lio_sam::cloud_info :definition-)
      "# Cloud Info
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
================================================================================
MSG: std_msgs/Header
# Standard metadata for higher-level stamped data types.
# This is generally used to communicate timestamped data 
# in a particular coordinate frame.
# 
# sequence ID: consecutively increasing ID 
uint32 seq
#Two-integer timestamp that is expressed as:
# * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
# * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
# time-handling sugar is provided by the client library
time stamp
#Frame this data is associated with
string frame_id

================================================================================
MSG: sensor_msgs/PointCloud2
# This message holds a collection of N-dimensional points, which may
# contain additional information such as normals, intensity, etc. The
# point data is stored as a binary blob, its layout described by the
# contents of the \"fields\" array.

# The point cloud data may be organized 2d (image-like) or 1d
# (unordered). Point clouds organized as 2d images may be produced by
# camera depth sensors such as stereo or time-of-flight.

# Time of sensor data acquisition, and the coordinate frame ID (for 3d
# points).
Header header

# 2D structure of the point cloud. If the cloud is unordered, height is
# 1 and width is the length of the point cloud.
uint32 height
uint32 width

# Describes the channels and their layout in the binary data blob.
PointField[] fields

bool    is_bigendian # Is this data bigendian?
uint32  point_step   # Length of a point in bytes
uint32  row_step     # Length of a row in bytes
uint8[] data         # Actual point data, size is (row_step*height)

bool is_dense        # True if there are no invalid points

================================================================================
MSG: sensor_msgs/PointField
# This message holds the description of one point entry in the
# PointCloud2 message format.
uint8 INT8    = 1
uint8 UINT8   = 2
uint8 INT16   = 3
uint8 UINT16  = 4
uint8 INT32   = 5
uint8 UINT32  = 6
uint8 FLOAT32 = 7
uint8 FLOAT64 = 8

string name      # Name of field
uint32 offset    # Offset from start of point struct
uint8  datatype  # Datatype enumeration, see above
uint32 count     # How many elements in the field

")



(provide :lio_sam/cloud_info "7490d670e17d4a2b9ad7d689d0925e1a")


