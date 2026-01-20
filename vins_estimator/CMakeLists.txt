cmake_minimum_required(VERSION 3.16)
project(vins_estimator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(OpenCV_DIR "/usr/lib/x86_64-linux-gnu/cmake/opencv4") 
find_package(OpenCV 4.5.4 EXACT REQUIRED)
find_package(glog REQUIRED)
find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(OpenMP REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)

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

fins_add_node(${PROJECT_NAME} src/nodes.cpp)
target_link_libraries(${PROJECT_NAME} PRIVATE vins_algo_lib)

fins_link_ros_dependencies(${PROJECT_NAME}
    nav_msgs
    sensor_msgs
    geometry_msgs
)

set_target_properties(${PROJECT_NAME} PROPERTIES 
    LIBRARY_OUTPUT_NAME "workspace_vins_estimator"
    POSITION_INDEPENDENT_CODE ON
)