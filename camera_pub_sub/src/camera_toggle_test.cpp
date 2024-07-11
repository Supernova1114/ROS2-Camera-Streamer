#include <string.h>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);

    rclcpp::NodeOptions options;
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("camera_toggle_test", options);

    node->declare_parameter("camera_name", "camera1");
    node->declare_parameter("toggle_camera", true);

    std::string cameraName = node->get_parameter("camera_name").as_string();
    bool enableCamera = node->get_parameter("toggle_camera").as_bool();

    std::string toggleServiceName = cameraName + "/toggle_camera";

    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr client =
    node->create_client<std_srvs::srv::SetBool>(toggleServiceName);

    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = enableCamera;

    if (client->service_is_ready())
    {
        std::cout << "Service is available, sending request!" << std::endl;
    }
    else
    {
        std::cout << "Error: Service is not available!" << std::endl;
        rclcpp::shutdown();
        return 1;
    }

    auto result = client->async_send_request(request);

    // Wait for the result.
    if (rclcpp::spin_until_future_complete(node, result) ==
        rclcpp::FutureReturnCode::SUCCESS)
    {
        std::cout << "Camera toggle {" << enableCamera << "} ";

        if (result.get()->success)
            std::cout << "successful!";
        else
            std::cout << "failed!";
        
        std::cout << std::endl;
    }

    rclcpp::shutdown();
    return 0;
}
