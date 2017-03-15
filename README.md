# Visual Odometry Metapackage 

rpl\_vo is a visual odometry metapackage for UAV's in RPL.

## rplvo_mono

Monocular VO package. Parameters;

* odom_frame_id (default /odom)
* base_link_frame_id (default /base_link)
* sensor_frame_id (default /camera)
* publish_tf (default true)
* invert_tf (default false)
* cam_model
* cam_width
* cam_height
* cam_fx
* cam_fy
* cam_cx
* cam_cy
* cam_d0 (default 0.0)
* cam_d1 (default 0.0)
* cam_d2 (default 0.0)
* cam_d3 (default 0.0)
* image_topic
* min_number_of_features
* ransac_threshold
* feature_detector_threshold
* feature_tracker_window_size
* feature_tracker_max_pyramid_level
* feature_tracker_max_iterations
* feature_tracker_epsilon
* feature_tracker_eigen_threshold

