#pragma once

#include <cstdio>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <array>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sstream>
#include <cstring>
#include "helper_functions.h"

std::string get_usb_devices();

std::string get_device_path(std::string serial_ID);