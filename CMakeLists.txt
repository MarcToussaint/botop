cmake_minimum_required(VERSION 3.5)

project(LISRobotOperation)

## first time call: generate unity sources
if(NOT DEFINED CMAKE_CACHE_MAJOR_VERSION)
  execute_process(COMMAND make -j1 -C ${CMAKE_CURRENT_SOURCE_DIR}/rai unityAll)
endif()

## variable default values
set(CMAKE_CXX_STANDARD 14 CACHE STRING "C++ version selection")
set(PYBIND11_PYTHON_VERSION 3.8 CACHE STRING "python version")
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/.local" CACHE PATH "..." FORCE)
endif()
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")

## options
option(USE_PYBIND "build python bindings" ON)
option(USE_BULLET "link to bullet" OFF)
option(USE_PHYSX "link to physx" ON)
option(USE_OPENCV "link to opencv" ON)
option(USE_OPENVR "link to openvr" OFF)
option(USE_OPENCV_LOCAL "link to opencv" OFF)
option(USE_REALSENSE "link to realsense2 driver library" ON)
option(USE_LIBFRANKA "link to libfranka" ON)
option(USE_PCL "link to pcl library" OFF)
option(USE_QHULL8 "use (newer) version 8 of qhull" OFF)
option(MODULE_Vive "compile the ViveController module" OFF)

## compile options
add_compile_options(
  -O3 -g -Wall -Wpedantic -fPIC
  $<$<COMPILE_LANGUAGE:CXX>:-std=c++14>
  $<$<COMPILE_LANGUAGE:CXX>:-Wno-terminate>
  )

################################################################################

add_definitions(
  -DRAI_ROOT_PATH=\"${CMAKE_SOURCE_DIR}/rai\"
  -DRAI_X11 -DRAI_LAPACK -DRAI_EIGEN -DRAI_PTHREAD
  -DRAI_GL
  -DRAI_GLFW -DRAI_ANN -DRAI_QHULL8 -DRAI_ASSIMP -DNO_CYCLE_DETECTION -DRAI_PNG -DRAI_PLY
  -DRAI_GJK -DRAI_CCD -DRAI_FCL -DRAI_Lewiner
  -DRAI_PORTAUDIO
  -BOOST_ERROR_CODE_HEADER_ONLY
  -DRAI_BotOp
  -DRAI_ROBOTIQ
  )

if(USE_QHULL8)
  add_definitions( -DRAI_QHULL8 )
endif()

################################################################################

include_directories(
  rai/rai
  src/
  /usr/include/eigen3
  $ENV{HOME}/.local/include
  )

link_directories(
  $ENV{HOME}/.local/lib
  $ENV{HOME}/.local/lib/PhysX
  )

file(GLOB SRC_GeoGJK rai/rai/Geo/GJK/*.c)
file(GLOB SRC_GeoPly rai/rai/Geo/ply/*.c)
file(GLOB SRC_GeoCcd rai/rai/Geo/ccd_rai/*.c)
file(GLOB SRC_GeoLewiner rai/rai/Geo/Lewiner/*.cpp)

file(GLOB SRC_BotOp src/BotOp/*.cpp)
file(GLOB SRC_Franka src/Franka/*.cpp)
file(GLOB SRC_OptiTrack src/OptiTrack/*.cpp)
file(GLOB SRC_RealSense src/RealSense/*.cpp)
file(GLOB SRC_Robotiq src/Robotiq/*.cpp)
file(GLOB SRC_Audio src/Audio/*.cpp)
file(GLOB SRC_MarkerVision src/MarkerVision/*.cpp)

add_library(rai SHARED
  rai/rai/Core/unity.cxx
  rai/rai/Optim/unity.cxx
  rai/rai/Geo/unity.cxx
  ${SRC_GeoGJK}
  ${SRC_GeoPly}
  ${SRC_GeoCcd}
  ${SRC_GeoLewiner}
  rai/rai/Algo/unity.cxx
  rai/rai/Search/unity.cxx
  rai/rai/PathAlgos/unity.cxx
  rai/rai/Gui/unity.cxx
  rai/rai/Kin/unity.cxx
  rai/rai/KOMO/unity.cxx
  rai/rai/Logic/unity.cxx
  rai/rai/Control/unity.cxx
  rai/rai/Perception/unity.cxx
  ${SRC_BotOp}
  ${SRC_Franka}
  ${SRC_OptiTrack}
  ${SRC_RealSense}
  ${SRC_Robotiq}
  ${SRC_Audio}
  ${SRC_MarkerVision}
  )

target_link_libraries(rai
  jsoncpp lapack blas pthread rt X11
  assimp ccd fcl qhull ann glfw GLEW glut GLU GL png
  portaudio
  boost_system 
  )

################################################################################
#
# compile extra modules

if(MODULE_Vive)
  add_definitions(-DRAI_Vive)
  target_sources(rai PRIVATE src/ViveController/vivecontroller.cpp)
  message(STATUS "[rai] compiling module Vive")
endif()
  
################################################################################
#
# include external libraries

#find_package (Eigen3 3.3 REQUIRED NO_MODULE)
#target_link_libraries (rai Eigen3::Eigen)
find_package(PkgConfig REQUIRED)

if(USE_BULLET)
  find_package(Bullet REQUIRED)
  add_definitions(-DRAI_BULLET)
  include_directories(${BULLET_INCLUDE_DIRS})
  target_link_libraries(rai ${BULLET_LIBRARIES})
  message(STATUS "[rai] using bullet libs: " ${BULLET_LIBRARIES})
endif()

if(USE_PHYSX)
  add_definitions(-DRAI_PHYSX -DNDEBUG -DPX_DISABLE_FLUIDS -DCORELIB -DPX32 -DLINUX)
  target_link_libraries(rai PhysXExtensions_static_64 PhysX_static_64 PhysXCooking_static_64 PhysXCommon_static_64 PhysXFoundation_static_64)
  message(STATUS "[rai] using physx libs from ~/.local")
endif()

if(USE_OPENCV)
  find_package(OpenCV REQUIRED)
  add_definitions(-DRAI_OPENCV)
  target_link_libraries(rai ${OpenCV_LIBS})
  message(STATUS "[rai] using OpenCV libs:" ${OpenCV_LIBS})
endif()

if(USE_OPENCV_LOCAL)
  add_definitions(-DRAI_OPENCV)
  include_directories($ENV{HOME}/.local/include/opencv4)
  target_link_libraries(rai opencv_core opencv_highgui opencv_imgproc opencv_videoio)
  message(STATUS "[rai] using OpenCV libs from ~/.local")
endif()

if(USE_OPENVR)
  pkg_check_modules(OpenVR REQUIRED openvr)
  add_definitions(-DRAI_OPENVR)
  target_compile_options(rai PUBLIC ${OpenVR_CFLAGS_OTHER})
  target_include_directories(rai PUBLIC ${OpenVR_INCLUDE_DIRS})
  target_link_libraries(rai ${OpenVR_LIBRARIES})
  message(STATUS "[rai] using OpenVR libs: " ${OpenVR_LIBRARIES})
  # -I/usr/include/openvr -L/usr/lib/openvr -lopenvr_apis
  #target_link_libraries(rai openvr_api)
  #link_directories(
  #$ENV{HOME}/.local/lib/vrqtlibs
  #)
endif()

if(USE_REALSENSE)
  add_definitions(-DRAI_REALSENSE)
  target_link_libraries(rai realsense2)
  message(STATUS "[rai] using realsense libs")
endif()

if(USE_LIBFRANKA)
  add_definitions(-DRAI_FRANKA)
  target_link_libraries(rai franka PocoNet PocoFoundation)
  message(STATUS "[rai] using libfranka")
endif()

################################################################################

if(USE_PYBIND)
  set(PYTHON_EXECUTABLE "python${PYBIND11_PYTHON_VERSION}")
  message(STATUS "[rai] using python executable " ${PYTHON_EXECUTABLE} )
  execute_process(
    COMMAND bash "-c" "${PYTHON_EXECUTABLE} -m pybind11 --cmakedir" 
    OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE pybind11_DIR
  )
  message(STATUS "[rai] using pybind11_DIR " ${pybind11_DIR} )
  
  find_package(pybind11)
  add_definitions(-DRAI_PYBIND)
  include_directories(${pybind11_INCLUDE_DIRS})
  
  pybind11_add_module(ry SHARED
    rai/rai/ry/unity.cxx
    src/BotOp/pyBot.cxx
  )

  target_link_libraries(ry PRIVATE
    rai
  )
endif()

################################################################################

get_property(_defs DIRECTORY PROPERTY COMPILE_DEFINITIONS)
message(STATUS "[rai] compiler flags: " "${_defs}")

################################################################################

add_executable(kinEdit rai/bin/src_kinEdit/main.cpp)
target_link_libraries(kinEdit rai)

add_executable(bot bin/bot/main.cpp)
target_link_libraries(bot rai)

################################################################################

set(PYTHON_SITE "lib/python${PYBIND11_PYTHON_VERSION}/site-packages")

message(STATUS "[rai] installing python packages in " ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE}/robotic )

install(
  DIRECTORY rai/rai
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include
  FILES_MATCHING
  PATTERN "*.h" PATTERN "*.hxx" PATTERN "*.ipp"
  PATTERN "retired" EXCLUDE
  )

file(GLOB SRC_botopSrc src/*)

install(
  DIRECTORY ${SRC_botopSrc}
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/rai
  FILES_MATCHING
  PATTERN "*.h" PATTERN "*.hxx" PATTERN "*.ipp"
  PATTERN "retired" EXCLUDE
  )
  
install(
  TARGETS rai
  DESTINATION ${CMAKE_INSTALL_PREFIX}/lib
  COMPONENT pkgconfig)

install(
  TARGETS kinEdit bot
  DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)

install(
  TARGETS ry
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${PYTHON_SITE}/robotic)


