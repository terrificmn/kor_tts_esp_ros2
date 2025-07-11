cmake_minimum_required(VERSION 3.5)
project(kor_tts_esp_ros2)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(Python3_include_d "/usr/include/python3.10") # e.g., /usr/local/opt/python@3.X or /opt/anaconda3/envs/my_env
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) # If you use this message type


set(PortAudio_lib "/usr/lib/x86_64-linux-gnu/libportaudio.so")
# find_package(PortAudio REQUIRED)
#include(${Open3D_CONFIG})

# if (Python3_FOUND)
#     # message(FATAL_ERROR "Found Python3: ${Python3_EXECUTABLE}")
#     # message(FATAL_ERROR "Python3 Version: ${Python3_VERSION}")
#     if (Python3_Development_FOUND)
#         # message(FATAL_ERROR "Python3 Development Headers FOUND")
#         # message(FATAL_ERROR "Python3 Include Dirs: ${Python3_INCLUDE_DIRS}")  #  /usr/include/python3.10
#         message(FATAL_ERROR "Python3 Libraries: ${Python3_LIBRARIES}")
#     else()
#         message(FATAL_ERROR "Python3 Development headers NOT found, even though interpreter was located.")
#     endif()
# else()
#     message(FATAL_ERROR "Python3 not found at all!")
# endif()

include_directories(
  include
  ${Python3_include_d}
)

add_executable(${PROJECT_NAME}_node 
  src/main.cpp
  src/tts/sp.cpp
  src/tts/cin_publisher.cpp
)

ament_target_dependencies(${PROJECT_NAME}_node 
  std_msgs
)

target_link_libraries(${PROJECT_NAME}_node 
  rclcpp::rclcpp
  ${Python3_LIBRARIES}
  ${PortAudio_lib}
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
    ${Python3_include_d}
    # ${Python3_INCLUDE_DIRS}
)

install(TARGETS
${PROJECT_NAME}_node
RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
DESTINATION include/
)
install(DIRECTORY script-py DESTINATION share/${PROJECT_NAME}) # Copies the 'python' folder with your script

# ament_export_include_directories(include)
# ament_export_libraries(${PROJECT_NAME})
# ament_export_dependencies(${dependencies})
ament_package()
