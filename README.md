# Visual Odometry Metapackage 

rpl\_vo is a visual odometry metapackage for UAV's in RPL.

## Needed Packages

rpl_vo uses OpenCV 3.0+ functions, but  the OpenCV version in ROS Indigo is 2.4.8. Therefore it is needed to be installed. [This package](http://wiki.ros.org/opencv3) is used rather than a stand alone installation;

```
sudo apt-get install ros-<distro>-opencv3
```
The library uses functionalities of [vikit](https://github.com/uzh-rpg/rpg_vikit). It is needed to be clone in the catkin workspace and built;
```
cd <catkin_workspace>/src
git clone https://github.com/uzh-rpg/rpg_vikit.git
cd ..
catkin_make
```
The `MonoOdometer` class is derived from `OdometerBase` class in [viso2_ros](http://wiki.ros.org/viso2_ros). However it is not needed to install this package since the necessary header file is included.



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
* rectify_image
* min_number_of_features
* ransac_threshold
* ransac_confidence
* feature_detector_threshold
* feature_tracker_window_size
* feature_tracker_max_pyramid_level
* feature_tracker_max_iterations
* feature_tracker_epsilon
* feature_tracker_eigen_threshold
* visualize_frame_tracking

