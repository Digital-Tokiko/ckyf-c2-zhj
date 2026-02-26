cmake_minimum_required(VERSION 3.8)
project(auto_aim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include msg_serializer thirdparty/serialib MyClasses/Fort MyClasses/ImgProcess MyClasses/MyKalmanFilter)
add_subdirectory(thirdparty/serialib)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(auto_aim src/main.cpp MyClasses/ImgProcess/RawReceiver.cpp)
ament_target_dependencies(auto_aim rclcpp cv_bridge)
target_link_libraries(auto_aim ${OpenCV_LIBS} serialib)

install(TARGETS
  auto_aim
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
