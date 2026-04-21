# Install script for directory: /Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set path to fallback-tool for dependency-resolution.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/bullet/btBulletCollisionCommon.h;/usr/local/include/bullet/btBulletDynamicsCommon.h;/usr/local/include/bullet/Bullet-C-Api.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/include/bullet" TYPE FILE FILES
    "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src/btBulletCollisionCommon.h"
    "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src/btBulletDynamicsCommon.h"
    "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src/Bullet-C-Api.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/bullet/vectormath/vmInclude.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/include/bullet/vectormath" TYPE FILE FILES "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src/vectormath/vmInclude.h")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/bullet/vectormath/scalar/boolInVec.h;/usr/local/include/bullet/vectormath/scalar/floatInVec.h;/usr/local/include/bullet/vectormath/scalar/mat_aos.h;/usr/local/include/bullet/vectormath/scalar/quat_aos.h;/usr/local/include/bullet/vectormath/scalar/vec_aos.h;/usr/local/include/bullet/vectormath/scalar/vectormath_aos.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/include/bullet/vectormath/scalar" TYPE FILE FILES
    "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src/vectormath/scalar/boolInVec.h"
    "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src/vectormath/scalar/floatInVec.h"
    "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src/vectormath/scalar/mat_aos.h"
    "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src/vectormath/scalar/quat_aos.h"
    "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src/vectormath/scalar/vec_aos.h"
    "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src/vectormath/scalar/vectormath_aos.h"
    )
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/bullet/vectormath/sse/boolInVec.h;/usr/local/include/bullet/vectormath/sse/floatInVec.h;/usr/local/include/bullet/vectormath/sse/mat_aos.h;/usr/local/include/bullet/vectormath/sse/quat_aos.h;/usr/local/include/bullet/vectormath/sse/vec_aos.h;/usr/local/include/bullet/vectormath/sse/vecidx_aos.h;/usr/local/include/bullet/vectormath/sse/vectormath_aos.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/include/bullet/vectormath/sse" TYPE FILE FILES
    "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src/vectormath/sse/boolInVec.h"
    "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src/vectormath/sse/floatInVec.h"
    "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src/vectormath/sse/mat_aos.h"
    "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src/vectormath/sse/quat_aos.h"
    "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src/vectormath/sse/vec_aos.h"
    "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src/vectormath/sse/vecidx_aos.h"
    "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-2.82-r2704/src/vectormath/sse/vectormath_aos.h"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-build/src/BulletSoftBody/cmake_install.cmake")
  include("/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-build/src/BulletCollision/cmake_install.cmake")
  include("/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-build/src/BulletDynamics/cmake_install.cmake")
  include("/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-build/src/LinearMath/cmake_install.cmake")

endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
if(CMAKE_INSTALL_LOCAL_ONLY)
  file(WRITE "/Users/jameshatch/Documents/GitHub/EvoDevo-Modeling/bullet-build/src/install_local_manifest.txt"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
endif()
