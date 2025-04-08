################################################################################
# * Copyright 2023 ModalAI Inc.
# *
# * Redistribution and use in source and binary forms, with or without
# * modification, are permitted provided that the following conditions are met:
# *
# * 1. Redistributions of source code must retain the above copyright notice,
# *    this list of conditions and the following disclaimer.
# *
# * 2. Redistributions in binary form must reproduce the above copyright notice,
# *    this list of conditions and the following disclaimer in the documentation
# *    and/or other materials provided with the distribution.
# *
# * 3. Neither the name of the copyright holder nor the names of its contributors
# *    may be used to endorse or promote products derived from this software
# *    without specific prior written permission.
# *
# * 4. The Software is used solely in conjunction with devices provided by
# *    ModalAI Inc.
# *
# * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# * POSSIBILITY OF SUCH DAMAGE.
################################################################################

cmake_minimum_required(VERSION 3.5)
project(voxl_mpa_to_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(px4_ros_com REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(voxl_msgs REQUIRED)
find_package(nlohmann_json REQUIRED
    CONFIG
    PATHS "/usr/lib/cmake/"
)

include_directories(include) 

set(dependencies
  rclcpp
  sensor_msgs
  Eigen3
  px4_ros_com
  rosidl_default_generators
  geometry_msgs
  sensor_msgs
  image_transport
  voxl_msgs
  nav_msgs
  tf2
  tf2_ros
)

add_executable(${PROJECT_NAME}_node
    src/interfaces/imu_interface.cpp
    src/interfaces/pose_vel_6dof_interface.cpp
    src/interfaces/camera_interface.cpp
    src/interfaces/point_cloud_interface.cpp
    src/interfaces/stereo_interface.cpp
    src/interfaces/qvio_interface.cpp
    src/interfaces/ai_detection_interface.cpp
    src/utils/common_utils.cpp
    src/utils/camera_helpers.cpp
    src/interface_manager.cpp
    src/mpa_ros_node.cpp
    src/interfaces/tag_interface.cpp
)
ament_target_dependencies(${PROJECT_NAME}_node ${dependencies})
target_link_libraries(${PROJECT_NAME}_node
	/usr/lib64/libmodal_pipe.so
	/usr/lib64/libmodal_json.so
  nlohmann_json
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})

ament_export_dependencies(${dependencies})
ament_package()

