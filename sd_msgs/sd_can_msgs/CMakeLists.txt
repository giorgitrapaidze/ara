cmake_minimum_required(VERSION 3.5)
project(sd_can_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

file(GLOB_RECURSE MSG_FILES_PATH CONFIGURE_DEPENDS "msg/*.msg")

foreach (TMP_PATH ${MSG_FILES_PATH})
    file(RELATIVE_PATH MSG_FILE_NAME "${CMAKE_SOURCE_DIR}/" "${TMP_PATH}")
    list(APPEND MSG_FILE_NAMES ${MSG_FILE_NAME})
endforeach (TMP_PATH)

# Choose all msgs
set(MSG_COPY_FILES_PATH)
foreach (TMP_PATH ${MSG_FILES_PATH})
    file(RELATIVE_PATH MSG_FILE_NAME "${CMAKE_SOURCE_DIR}/msg" "${TMP_PATH}")
    list(APPEND MSG_COPY_FILES_PATH ${MSG_FILE_NAME})
endforeach()

# Choose copy msgs to build folder
foreach (msg_file ${MSG_COPY_FILES_PATH})
    get_filename_component(msg_file_name ${msg_file} NAME)
    configure_file(${CMAKE_SOURCE_DIR}/msg/${msg_file} ${CMAKE_CURRENT_BINARY_DIR}/msg/${msg_file_name} COPYONLY)
endforeach()
# Choose just filenames from the path
set(FLAT_MSG_FILES_PATH)
foreach (TMP_PATH ${MSG_FILES_PATH})
    # Take just filenames from the path
    get_filename_component(MSG_FILE_NAME "${TMP_PATH}" NAME)
    list(APPEND FLAT_MSG_FILES_PATH ${MSG_FILE_NAME})
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
        ${MSG_FILE_NAMES}
        DEPENDENCIES builtin_interfaces std_msgs geometry_msgs)
#Copy msgs to install folder
foreach (msg_file ${FLAT_MSG_FILES_PATH})
        install(FILES ${CMAKE_CURRENT_BINARY_DIR}/msg/${msg_file}
            DESTINATION share/${PROJECT_NAME}/msg)
endforeach()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
