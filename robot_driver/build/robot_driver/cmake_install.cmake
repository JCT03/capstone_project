# Install script for directory: /home/robotics/capstone_project/robot_driver

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/robotics/capstone_project/robot_driver/install/robot_driver")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_driver/drive" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_driver/drive")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_driver/drive"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_driver" TYPE EXECUTABLE FILES "/home/robotics/capstone_project/robot_driver/build/robot_driver/drive")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_driver/drive" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_driver/drive")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_driver/drive"
         OLD_RPATH "/home/robotics/ros2_iron/install/rclcpp/lib:/home/robotics/ros2_iron/install/nav_msgs/lib:/home/robotics/ros2_iron/install/tf2/lib:/home/robotics/ros2_iron/install/libstatistics_collector/lib:/home/robotics/ros2_iron/install/rcl/lib:/home/robotics/ros2_iron/install/rcl_logging_interface/lib:/home/robotics/ros2_iron/install/rmw_implementation/lib:/home/robotics/ros2_iron/install/ament_index_cpp/lib:/home/robotics/ros2_iron/install/type_description_interfaces/lib:/home/robotics/ros2_iron/install/rcl_interfaces/lib:/home/robotics/ros2_iron/install/rcl_yaml_param_parser/lib:/home/robotics/ros2_iron/install/rosgraph_msgs/lib:/home/robotics/ros2_iron/install/statistics_msgs/lib:/home/robotics/ros2_iron/install/tracetools/lib:/home/robotics/ros2_iron/install/service_msgs/lib:/home/robotics/ros2_iron/install/geometry_msgs/lib:/home/robotics/ros2_iron/install/std_msgs/lib:/home/robotics/ros2_iron/install/builtin_interfaces/lib:/home/robotics/ros2_iron/install/rosidl_typesupport_fastrtps_c/lib:/home/robotics/ros2_iron/install/rosidl_typesupport_introspection_cpp/lib:/home/robotics/ros2_iron/install/rosidl_typesupport_introspection_c/lib:/home/robotics/ros2_iron/install/rosidl_typesupport_fastrtps_cpp/lib:/home/robotics/ros2_iron/install/fastcdr/lib:/home/robotics/ros2_iron/install/rmw/lib:/home/robotics/ros2_iron/install/rosidl_dynamic_typesupport/lib:/home/robotics/ros2_iron/install/rosidl_typesupport_cpp/lib:/home/robotics/ros2_iron/install/rosidl_typesupport_c/lib:/home/robotics/ros2_iron/install/rcpputils/lib:/home/robotics/ros2_iron/install/rosidl_runtime_c/lib:/home/robotics/ros2_iron/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_driver/drive")
    endif()
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/robotics/capstone_project/robot_driver/build/robot_driver/CMakeFiles/drive.dir/install-cxx-module-bmi-noconfig.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/robotics/capstone_project/robot_driver/build/robot_driver/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/robot_driver")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/robotics/capstone_project/robot_driver/build/robot_driver/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/robot_driver")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_driver/environment" TYPE FILE FILES "/home/robotics/ros2_iron/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_driver/environment" TYPE FILE FILES "/home/robotics/capstone_project/robot_driver/build/robot_driver/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_driver/environment" TYPE FILE FILES "/home/robotics/ros2_iron/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_driver/environment" TYPE FILE FILES "/home/robotics/capstone_project/robot_driver/build/robot_driver/ament_cmake_environment_hooks/path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_driver" TYPE FILE FILES "/home/robotics/capstone_project/robot_driver/build/robot_driver/ament_cmake_environment_hooks/local_setup.bash")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_driver" TYPE FILE FILES "/home/robotics/capstone_project/robot_driver/build/robot_driver/ament_cmake_environment_hooks/local_setup.sh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_driver" TYPE FILE FILES "/home/robotics/capstone_project/robot_driver/build/robot_driver/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_driver" TYPE FILE FILES "/home/robotics/capstone_project/robot_driver/build/robot_driver/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_driver" TYPE FILE FILES "/home/robotics/capstone_project/robot_driver/build/robot_driver/ament_cmake_environment_hooks/package.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/robotics/capstone_project/robot_driver/build/robot_driver/ament_cmake_index/share/ament_index/resource_index/packages/robot_driver")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_driver/cmake" TYPE FILE FILES
    "/home/robotics/capstone_project/robot_driver/build/robot_driver/ament_cmake_core/robot_driverConfig.cmake"
    "/home/robotics/capstone_project/robot_driver/build/robot_driver/ament_cmake_core/robot_driverConfig-version.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_driver" TYPE FILE FILES "/home/robotics/capstone_project/robot_driver/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  if(CMAKE_INSTALL_COMPONENT MATCHES "^[a-zA-Z0-9_.+-]+$")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
  else()
    string(MD5 CMAKE_INST_COMP_HASH "${CMAKE_INSTALL_COMPONENT}")
    set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INST_COMP_HASH}.txt")
    unset(CMAKE_INST_COMP_HASH)
  endif()
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
  file(WRITE "/home/robotics/capstone_project/robot_driver/build/robot_driver/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
