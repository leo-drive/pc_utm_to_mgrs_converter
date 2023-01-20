cmake_minimum_required(VERSION 3.14)
project(pc_utm_to_mgrs_converter)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_C_STANDARD 17)

# find dependencies
#find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS io)
find_package(pcl_conversions REQUIRED)


include(cmake/macros.cmake)
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")
find_package(GeographicLib REQUIRED)

find_path(GeographicLib_INCLUDE_DIR GeographicLib/Config.h
        PATH_SUFFIXES GeographicLib)
set(GeographicLib_INCLUDE_DIRS ${GeographicLib_INCLUDE_DIR})

find_library(GeographicLib_LIBRARIES
        NAMES Geographic)

add_definitions(${GeographicLib_DEFINITIONS})

add_executable(${PROJECT_NAME}  src/pc_utm_to_mgrs_converter.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs PCL pcl_conversions)
install(TARGETS
        ${PROJECT_NAME}
        DESTINATION lib/${PROJECT_NAME}
        )

include_directories(include/)



target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES} ${GeographicLib_LIBRARIES})

target_include_directories(pc_utm_to_mgrs_converter
        SYSTEM PUBLIC
        ${PCL_INCLUDE_DIRS}
        ${GeographicLib_INCLUDE_DIRS}
        )

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

ament_package()
