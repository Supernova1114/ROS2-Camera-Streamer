# Camera-Streaming-System
A camera streaming system for ROS 2
Tested using ROS 2 Humble


## How to use:
1. Find the serial ID for the camera devices using the `find_devpath.bash` file within the `resources/` folder.
2. Add the necessary `CameraEncoder` objects to the `encoder.launch.py` file, and give each `CameraDecoder` a **camera name**.
3. Add `CameraDecoder` objects to the `decoder.launch.py` file, and specify the **camera name**.
4. Run `ros2 launch camera_streamer encoder.launch.py` to start the encoder.
5. Run `ros2 launch camera_streamer decoder.launch.py` to start the decoder.
6. Subscribe to `/<camera_name>/out` to view video feed.

TODO - add info about enabling video feed from decoder.
Add info regarding config tester.
Add info regarding being able to use rqt_image_viewer
