
Step 1:
  Replay your point cloud bag or real-time pointcloud(The subscribed point cloud topic has been set to /ouster/points,if you use other kinds of lidar,please replace the input topic)
  Command: ros2 bag play your-point-bag -l(optional)
Step 2:
  Start the program
  Command: cd detect_ws && colcon build  
           source install/setup.bash
           ros2 launch foreground_extractor extractor.launch.py
Step 3:
  Launch rviz2 and add topics such as /foreground_points and /human_cluster (remember to set the topic "Reliability Policy" to Best Effort)
