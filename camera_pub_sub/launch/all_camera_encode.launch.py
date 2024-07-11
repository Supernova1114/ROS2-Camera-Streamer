from launch import LaunchDescription
from launch_ros.actions import Node
from dataclasses import dataclass

# GLOBAL PARAMS (Can be overriden in Camera class)

IMAGE_SEND_WIDTH = 1280
IMAGE_SEND_HEIGHT = 720
IMAGE_SEND_FPS = 15
JPEG_QUALITY = 10

CAMERA_CAP_WIDTH = 1280
CAMERA_CAP_HEIGHT = 720
CAMERA_CAP_FPS = 30

HOST_MACHINE = "amd64"

@dataclass
class CameraEncoder():

    camera_name: str = ""
    serial_ID: str = ""

    send_width: int = IMAGE_SEND_WIDTH
    send_height: int = IMAGE_SEND_HEIGHT
    send_framerate: int = IMAGE_SEND_FPS
    jpeg_quality: int = JPEG_QUALITY # 1 - 100   

    cap_width: int = CAMERA_CAP_WIDTH
    cap_height: int = CAMERA_CAP_HEIGHT
    cap_framerate: int = CAMERA_CAP_FPS

    def get_node(self) -> Node:
        
        camera_encoder_node = Node(
            package = 'camera_pub_sub',
            executable = 'camera_encoder',
            name = self.camera_name + "_enc",
            output='screen',
            parameters=[
                {'camera_name': self.camera_name},
                {'serial_ID': self.serial_ID},
                {'camera_cap_width': self.cap_width},
                {'camera_cap_height': self.cap_height},
                {'camera_cap_fps': self.cap_framerate},
                {'image_send_width': self.send_width},
                {'image_send_height': self.send_height},
                {'image_send_fps': self.send_framerate},
                {'auto_enable_camera': False},
                {'host_machine': HOST_MACHINE},
                {self.camera_name + '.transport.format': 'jpeg'},
                {self.camera_name + '.transport.jpeg_quality': self.jpeg_quality}
            ],
        )  

        return camera_encoder_node  


def generate_launch_description():

    camera_list = (
        CameraEncoder("camera_forearm", "Arducam_Technology_Co.__Ltd._Arducam_IMX179_Camera_0010"),
        CameraEncoder("camera_ee_down", "Arducam_Technology_Co.__Ltd._Arducam_IMX179_Camera_0004"),
        CameraEncoder("camera_ee_forward", "Arducam_Technology_Co.__Ltd._Arducam_IMX179_Camera_0001"),
        CameraEncoder("camera_front_right", "Arducam_Technology_Co.__Ltd._Arducam_IMX179_Camera_0007"),
        CameraEncoder("camera_front_left", "Arducam_Technology_Co.__Ltd._Arducam_IMX179_Camera_0009"),
        CameraEncoder("camera_back_right", "Arducam_Technology_Co.__Ltd._0b12_0006"),
        CameraEncoder("camera_back_left", "Arducam_Technology_Co.__Ltd._0b12_0005"),
        CameraEncoder("camera_mast", "16MP_Camera_Mamufacture_16MP_USB_Camera_2022050701"),

    )
    
    node_list = []

    for camera in camera_list:
        node_list.append(camera.get_node())

    return LaunchDescription(node_list)

