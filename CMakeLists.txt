cmake_minimum_required(VERSION 3.4)

project(LISRobotOperation)

set(CMAKE_CXX_STANDARD 14 CACHE STRING "C++ version selection")

option(USE_PYBIND "build python bindings" ON)
option(USE_OPENCV "link to opencv" OFF)
option(USE_REALSENSE "link to realsense2 driver library" OFF)

add_compile_options(
  -O3 -g -Wall -fPIC -std=c++14 -Wno-terminate -Wno-pragmas -lboost_system
  )

################################################################################

add_definitions(
  -DRAI_ROOT_PATH=\"${CMAKE_SOURCE_DIR}/rai\"
  -DRAI_X11 -DRAI_LAPACK -DRAI_EIGEN -DRAI_PTHREAD
  -DRAI_GL
  -DRAI_GLFW -DRAI_ANN -DRAI_QHULL -DRAI_ASSIMP -DNO_CYCLE_DETECTION -DRAI_PNG -DRAI_PLY
  -DRAI_SWIFT -DRAI_GJK -DRAI_FCL -DRAI_CCD
  -DRAI_FRANKA
  -BOOST_ERROR_CODE_HEADER_ONLY
  )

################################################################################

include_directories(
  src/
  rai/rai
  /usr/include/eigen3
  $ENV{HOME}/opt/include
  )

link_directories(
  $ENV{HOME}/opt/lib
  )

file(GLOB SRC_Core rai/rai/Core/*.cpp)
file(GLOB SRC_Optim rai/rai/Optim/*.cpp)
file(GLOB SRC_Geo rai/rai/Geo/*.cpp)
file(GLOB SRC_GeoGJK rai/rai/Geo/GJK/*.c)
file(GLOB SRC_GeoPly rai/rai/Geo/ply/*.c)
file(GLOB SRC_Algo rai/rai/Algo/*.cpp)
file(GLOB SRC_Plot rai/rai/Plot/*.cpp)
file(GLOB SRC_Gui rai/rai/Gui/*.cpp)
file(GLOB SRC_Kin rai/rai/Kin/*.cpp)
file(GLOB SRC_KinSwift rai/rai/Kin/SWIFT/*.cpp)
file(GLOB SRC_KOMO rai/rai/KOMO/*.cpp)
file(GLOB SRC_Logic rai/rai/Logic/*.cpp)
file(GLOB SRC_LGP rai/rai/LGP/*.cpp)
file(GLOB SRC_RosCom rai/rai/RosCom/*.cpp)
file(GLOB SRC_Perception rai/rai/Perception/*.cpp)
file(GLOB SRC_Control rai/rai/Control/*.cpp)
file(GLOB SRC_Operate rai/rai/Operate/*.cpp)
file(GLOB SRC_ry rai/rai/ry/*.cpp)

file(GLOB SRC_Franka src/Franka/*.cpp)
file(GLOB SRC_RealSense src/RealSense/*.cpp)
file(GLOB SRC_BotOp src/BotOp/*.cpp)
file(GLOB SRC_Robotiq src/Robotiq/*.cpp)

add_library(rai SHARED
  ${SRC_Core}
  ${SRC_Optim}
  ${SRC_Geo}
  ${SRC_GeoGJK}
  ${SRC_GeoPly}
  ${SRC_Algo}
  ${SRC_Plot}
  ${SRC_Gui}
  ${SRC_Kin}
  ${SRC_KinSwift}
  ${SRC_KOMO}
  ${SRC_Logic}
  #${SRC_LGP}
  #${SRC_RosCom}
  ${SRC_Perception}
  ${SRC_Control}
  #${SRC_Operate}
  ${SRC_Franka}
  ${SRC_Robotiq}
  ${SRC_BotOp}
  ${SRC_RealSense}
  )

target_link_libraries(rai
  jsoncpp lapack blas pthread rt X11
  assimp fcl ann glfw GLEW glut GLU GL png qhull
  boost_system
  franka
  )

################################################################################

if(USE_PYBIND)
  execute_process(
    COMMAND bash "-c" "python3 -m pybind11 --cmakedir" 
    OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE pybind11_DIR
  )
  find_package(pybind11)
  add_definitions(-DRAI_PYBIND)
endif()

if(USE_OPENCV)
  find_package(OpenCV REQUIRED)
  add_definitions(-DRAI_OPENCV)
  target_link_libraries(rai ${OpenCV_LIBS})
endif()

if(USE_REALSENSE)
  add_definitions(-DRAI_REALSENSE)
  target_link_libraries(rai realsense2)
endif()

################################################################################

pybind11_add_module(libry SHARED
  ${SRC_ry}
  )
target_link_libraries(libry PRIVATE
  rai
  )

pybind11_add_module(libpybot SHARED
  src/BotOp/pyBot.cxx
  )
target_link_libraries(libpybot PRIVATE
  rai
  libry
  )

################################################################################

add_executable(kinEdit rai/bin/src_kinEdit/main.cpp)
target_link_libraries(kinEdit rai)

add_executable(bot bin/bot/main.cpp)
target_link_libraries(bot rai)


