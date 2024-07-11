#include "usb_device.h"

std::string get_usb_devices() {

    std::string package_share_directory = ament_index_cpp::get_package_share_directory("camera_pub_sub");
    std::string script_path = package_share_directory + "/resources/find_devpath.bash";

    std::string command = "chmod +x " + script_path;
    
    system(command.c_str());
    
    return exec(script_path.c_str());
}

std::string get_device_path(std::string serial_ID)
{
    std::string device_list = get_usb_devices();

    std::istringstream stream(device_list);
    std::string line;
    std::string delimiter = " - ";
    std::string keyword = "video";

    std::string device_path_found = "";

    while (std::getline(stream, line)) 
    {
        int delimiter_index = line.find(delimiter);

        std::string device_path = line.substr(0, delimiter_index);

        size_t keyword_index = device_path.find(keyword);

        // Get rid of device paths that are not video feeds
        if (keyword_index == std::string::npos)
            continue;
        
        // Get rid of odd numbered dev paths for /dev/video#
        // odd numbers are for camera control rather than image output
        if ((int(device_path[device_path.length() - 1]) - 48) % 2 == 1) {
            continue;
        }
        
        std::string serial_number = line.substr(delimiter_index + delimiter.length());

        if (serial_number == serial_ID) 
        {
            device_path_found = device_path;
        }
    }

    return device_path_found;
}