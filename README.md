# ROS2-Camera-Streamer
A camera streaming system for ROS 2
<br><br>
Tested using ROS 2 Humble, Ubuntu 22.04, amd64 laptop.
<br>
Tested using ROS 2 Humble, Jetson Linux DP 6.0, Jetson AGX Orin.

## Prerequisites:
1. Install ROS 2 Compressed Image Transport: `sudo apt install ros-humble-compressed-image-transport`
2. Make sure Compressed Image Transport is listed: `ros2 run image_transport list_transports`

## Usage:
1. Find the **serial ID** for the USB camera devices using the `find_devpath.bash` file within the `resources/` folder.
2. Set the proper host machine architecture using `HOST_MACHINE` within `encoder.launch.py`.
3. Add `CameraEncoder` objects to the `encoder.launch.py` file, and give each a **camera name** and **serial ID**.
4. Add `CameraDecoder` objects to the `decoder.launch.py` file, and specify the **camera name**.
5. Build the package using `colcon build`
6. Run `ros2 launch camera_streamer encoder.launch.py` to start the encoder.
7. Run `ros2 launch camera_streamer decoder.launch.py` to start the decoder.

## Publishers
- Topic: `<camera_name>/out`, Type: `Image`, QoS: `SensorData`; Video feed output.
## Services
- Service: `<camera_name>/toggle_camera`, Type: `std_srvs/SetBool`; Toggle the camera feed on or off. (Will also release the USB device).
- Service: `<camera_name>/set_encoder_config`, Type: `SetEncoderConfig`; Set encoder parameters such as resolution and frame rate.

## Extra Features:
- To use the built-in camera viewer, enable `ENABLE_WINDOW_VIEW` within `decoder.launch.py`.
- JPEG image compression can be set using the `JPEG_COMPRESSION` variable within `encoder.launch.py`.
- Set cameras to auto enable on encoder startup using `AUTO_ENABLE_CAMERAS` within `encoder.launch.py`.
- Test encoder services using `ros2 launch camera_streamer config_test.launch.py`. Make sure the **camera name** is specified.

## Notes:
- Seem to only be able to use two cameras per physical USB controller. May need to update GStreamer API configuration to allocate less memory to the device?
- Other ROS 2 image transports can potentially be used. Would need to change `IMAGE_TRANSPORT` within `decoder.launch.py` to the name within `ros2 run image_transport list_transports` list, and then add the proper ROS 2 parameters for the transport into `encoder.launch.py`:
```
Example from encoder.launch.py for compressed image transport:
{self.camera_name + '.transport.format': 'jpeg'},
{self.camera_name + '.transport.jpeg_quality': self.jpeg_quality}

