# Install script for directory: /home/dvtnguyen/devel/hpp/src/hpp-practicals

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/dvtnguyen/devel/hpp/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
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

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/hpp/practicals" TYPE FILE PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE FILES "/home/dvtnguyen/devel/hpp/src/hpp-practicals/build-rel/include/hpp/practicals/config.hh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/hpp/practicals" TYPE FILE PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE FILES "/home/dvtnguyen/devel/hpp/src/hpp-practicals/build-rel/include/hpp/practicals/deprecated.hh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/hpp/practicals" TYPE FILE PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE FILES "/home/dvtnguyen/devel/hpp/src/hpp-practicals/build-rel/include/hpp/practicals/warning.hh")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  EXECUTE_PROCESS(COMMAND /usr/bin/gmake hpp_practicals-doc)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/home/dvtnguyen/devel/hpp/install/share/doc/hpp_practicals/doxygen-html")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/home/dvtnguyen/devel/hpp/install/share/doc/hpp_practicals" TYPE DIRECTORY FILES "/home/dvtnguyen/devel/hpp/src/hpp-practicals/build-rel/doc/doxygen-html")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hpp_practicals/urdf/ur_benchmark" TYPE FILE FILES
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/urdf/ur_benchmark/obstacles.urdf"
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/urdf/ur_benchmark/table.urdf"
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/urdf/ur_benchmark/wall.urdf"
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/urdf/ur_benchmark/box.urdf"
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/urdf/ur_benchmark/ground.urdf"
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/urdf/ur_benchmark/ground_with_obstacles.urdf"
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/urdf/ur_benchmark/pokeball.urdf"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hpp_practicals/srdf/ur_benchmark" TYPE FILE FILES
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/srdf/ur_benchmark/box.srdf"
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/srdf/ur_benchmark/ground.srdf"
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/srdf/ur_benchmark/pokeball.srdf"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hpp_practicals/urdf" TYPE FILE FILES
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/urdf/ur5_gripper.urdf"
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/urdf/ur5_joint_limited_robot.urdf"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hpp_practicals/srdf" TYPE FILE FILES
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/srdf/ur5_gripper.srdf"
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/srdf/ur5_joint_limited_robot.srdf"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hpp_practicals" TYPE DIRECTORY FILES "/home/dvtnguyen/devel/hpp/src/hpp-practicals/meshes")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/hpp/corbaserver/practicals/ur5" TYPE FILE FILES
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/src/hpp/corbaserver/practicals/ur5/robot.py"
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/src/hpp/corbaserver/practicals/ur5/__init__.py"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/hpp/corbaserver/practicals" TYPE FILE FILES "/home/dvtnguyen/devel/hpp/src/hpp-practicals/src/hpp/corbaserver/practicals/__init__.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/hpp/corbaserver/practicals/manipulation" TYPE FILE FILES "/home/dvtnguyen/devel/hpp/src/hpp-practicals/src/hpp/corbaserver/practicals/manipulation/__init__.py")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/python3.12/site-packages/hpp/corbaserver/practicals/manipulation/ur5" TYPE FILE FILES
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/src/hpp/corbaserver/practicals/manipulation/ur5/robot.py"
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/src/hpp/corbaserver/practicals/manipulation/ur5/__init__.py"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  include("/home/dvtnguyen/devel/hpp/src/hpp-practicals/build-rel/CMakeFiles/hpp_practicals.dir/install-cxx-module-bmi-Release.cmake" OPTIONAL)
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE FILES "/home/dvtnguyen/devel/hpp/src/hpp-practicals/build-rel/hpp_practicals.pc")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/dvtnguyen/devel/hpp/src/hpp-practicals/build-rel/share/ament_index/resource_index/packages/hpp_practicals")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hpp_practicals/hook" TYPE FILE FILES "/home/dvtnguyen/devel/hpp/src/hpp-practicals/build-rel/share/hpp_practicals/hook/ament_prefix_path.dsv")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/hpp_practicals" TYPE FILE FILES "/home/dvtnguyen/devel/hpp/src/hpp-practicals/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/hpp_practicals" TYPE FILE FILES "/home/dvtnguyen/devel/hpp/install/share/jrl-cmakemodules/cxx-standard.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/hpp_practicals" TYPE FILE FILES
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/build-rel/generated/hpp_practicalsConfig.cmake"
    "/home/dvtnguyen/devel/hpp/src/hpp-practicals/build-rel/generated/hpp_practicalsConfigVersion.cmake"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/hpp_practicals/hpp_practicalsTargets.cmake")
    file(DIFFERENT _cmake_export_file_changed FILES
         "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/hpp_practicals/hpp_practicalsTargets.cmake"
         "/home/dvtnguyen/devel/hpp/src/hpp-practicals/build-rel/CMakeFiles/Export/17ffb241596288599e5ae36b6d1386cf/hpp_practicalsTargets.cmake")
    if(_cmake_export_file_changed)
      file(GLOB _cmake_old_config_files "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/hpp_practicals/hpp_practicalsTargets-*.cmake")
      if(_cmake_old_config_files)
        string(REPLACE ";" ", " _cmake_old_config_files_text "${_cmake_old_config_files}")
        message(STATUS "Old export file \"$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/cmake/hpp_practicals/hpp_practicalsTargets.cmake\" will be replaced.  Removing files [${_cmake_old_config_files_text}].")
        unset(_cmake_old_config_files_text)
        file(REMOVE ${_cmake_old_config_files})
      endif()
      unset(_cmake_old_config_files)
    endif()
    unset(_cmake_export_file_changed)
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/cmake/hpp_practicals" TYPE FILE FILES "/home/dvtnguyen/devel/hpp/src/hpp-practicals/build-rel/CMakeFiles/Export/17ffb241596288599e5ae36b6d1386cf/hpp_practicalsTargets.cmake")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/dvtnguyen/devel/hpp/src/hpp-practicals/build-rel/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
