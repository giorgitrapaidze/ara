cmake_minimum_required(VERSION 3.5)
project(sd_localization_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sd_map_msgs REQUIRED)

file(GLOB MSG_FILES_PATH CONFIGURE_DEPENDS "msg/*.msg")

foreach (TMP_PATH ${MSG_FILES_PATH})
    file(RELATIVE_PATH MSG_FILE_NAME "${CMAKE_SOURCE_DIR}/" "${TMP_PATH}")
    list(APPEND MSG_FILE_NAMES ${MSG_FILE_NAME})
endforeach (TMP_PATH)

rosidl_generate_interfaces(${PROJECT_NAME}
        ${MSG_FILE_NAMES}
        DEPENDENCIES builtin_interfaces std_msgs sd_map_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
