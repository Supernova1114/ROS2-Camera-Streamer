# Camera-Streaming-System
A camera streaming system for ROS 2
<br>
Tested using ROS 2 Humble


## How to use:
1. Find the serial ID for the camera devices using the `find_devpath.bash` file within the `resources/` folder.
2. Set the proper host machine architecture using `HOST_MACHINE` within `encoder.launch.py`.
3. Add `CameraEncoder` objects to the `encoder.launch.py` file, and give each `CameraDecoder` a **camera name** and **serial ID**.
4. Add `CameraDecoder` objects to the `decoder.launch.py` file, and specify the **camera name**.
5. Run `ros2 launch camera_streamer encoder.launch.py` to start the encoder.
6. Run `ros2 launch camera_streamer decoder.launch.py` to start the decoder.

## Publishers
- Topic: `<camera_name>/out`, Type: `Image`, QOS: SensorData, // Video feed output.
## Services
- Service: `<camera_name>/toggle_camera`, Type: `std_srvs/SetBool`; Toggle the camera feed on or off. (Will also release the USB device).
- Service: `<camera_name>/set_encoder_config`, Type: `SetEncoderConfig`; Set encoder parameters such as resolution and frame rate.

## Extra Features:
- To use the built-in camera viewer, enable `ENABLE_WINDOW_VIEW` within `decoder.launch.py`.
- JPEG image compression can be set using the `JPEG_COMPRESSION` variable within `decoder.launch.py`.
- Set cameras to auto enable on encoder startup using `AUTO_ENABLE_CAMERAS` within `encoder.launch.py`.
- Package contains a launch file with example code to test camera toggle and encoder config service functionality.

## Notes:
- 

TODO - add info about enabling video feed from decoder.
Add info regarding config tester.
Add info regarding being able to use rqt_image_viewer
Add info on services / subscriptions.
Add info regarding changing compression param.
