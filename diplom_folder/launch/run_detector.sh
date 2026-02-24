#!/bin/bash
ros2 run apriltag_ros tag_detector --ros-args \
  -r /image:=/logi_webcam/image_raw \
  -r /camera_info:=/logi_webcam/camera_info \
  --params-file /home/igsp-01/ros2_ws/src/Arya_ros2_ws_stepik/diplom_folder/config/apriltag_config.yaml
