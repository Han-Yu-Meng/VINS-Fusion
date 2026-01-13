cmake_minimum_required(VERSION 3.16)
project(vins_estimator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4") 
find_package(OpenCV 4.5.4 EXACT REQUIRED)

message(STATUS "OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBS: ${OpenCV_LIBS}")

find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(CAMERA_MODELS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../camera_models)

add_library(camera_models STATIC
    ${CAMERA_MODELS_DIR}/src/chessboard/Chessboard.cc
    ${CAMERA_MODELS_DIR}/src/calib/CameraCalibration.cc
    ${CAMERA_MODELS_DIR}/src/camera_models/CameraFactory.cc
    ${CAMERA_MODELS_DIR}/src/camera_models/Camera.cc
    ${CAMERA_MODELS_DIR}/src/camera_models/PinholeCamera.cc
    ${CAMERA_MODELS_DIR}/src/camera_models/PinholeFullCamera.cc
    ${CAMERA_MODELS_DIR}/src/camera_models/CataCamera.cc
    ${CAMERA_MODELS_DIR}/src/camera_models/EquidistantCamera.cc
    ${CAMERA_MODELS_DIR}/src/camera_models/ScaramuzzaCamera.cc
    ${CAMERA_MODELS_DIR}/src/camera_models/CostFunctionFactory.cc
    ${CAMERA_MODELS_DIR}/src/gpl/gpl.cc
    ${CAMERA_MODELS_DIR}/src/gpl/EigenQuaternionParameterization.cc
)

target_include_directories(camera_models PUBLIC 
    ${CAMERA_MODELS_DIR}/include
    ${EIGEN3_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

add_library(vins_algo_lib STATIC
    src/estimator/parameters.cpp
    src/estimator/estimator.cpp
    src/estimator/feature_manager.cpp
    src/factor/pose_local_parameterization.cpp
    src/factor/projectionTwoFrameOneCamFactor.cpp
    src/factor/projectionTwoFrameTwoCamFactor.cpp
    src/factor/projectionOneFrameTwoCamFactor.cpp
    src/factor/marginalization_factor.cpp
    src/utility/utility.cpp
    src/initial/solve_5pts.cpp
    src/initial/initial_aligment.cpp
    src/initial/initial_sfm.cpp
    src/initial/initial_ex_rotation.cpp
    src/featureTracker/feature_tracker.cpp
)

target_include_directories(vins_algo_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CAMERA_MODELS_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${CERES_INCLUDE_DIRS}
)

target_link_libraries(vins_algo_lib
    PUBLIC
    camera_models
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
    glog::glog
    gflags
    Eigen3::Eigen
    Sophus::Sophus
    OpenMP::OpenMP_CXX
    ${PCL_LIBRARIES}
)

add_library(vins_estimator SHARED 
    src/nodes.cpp
)

target_link_libraries(vins_estimator 
    PRIVATE 
    vins_algo_lib
    fins_sdk
)

foreach(msg_pkg sensor_msgs nav_msgs geometry_msgs)
    if(TARGET ${msg_pkg}::${msg_pkg}__rosidl_typesupport_cpp)
        target_link_libraries(vins_estimator PRIVATE ${msg_pkg}::${msg_pkg}__rosidl_typesupport_cpp)
    else()
        target_link_libraries(vins_estimator PRIVATE ${${msg_pkg}_LIBRARIES})
        target_include_directories(vins_estimator PRIVATE ${${msg_pkg}_INCLUDE_DIRS})
    endif()
endforeach()

set_target_properties(vins_estimator PROPERTIES 
    LIBRARY_OUTPUT_NAME "workspace_vins_estimator"
    POSITION_INDEPENDENT_CODE ON
)