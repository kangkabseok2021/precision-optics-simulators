cmake_minimum_required(VERSION 3.25)
project(AMRStackLibraries CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# Dependency Resolution (Optional for top-level monorepo compilation compatibility)
# -----------------------------------------------------------------------------

find_package(OpenCV COMPONENTS core imgproc calib3d QUIET)
find_package(PCL COMPONENTS common segmentation filters kdtree search QUIET)
find_package(paho-mqtt-cpp QUIET)
find_package(nlohmann_json QUIET)

if(OpenCV_FOUND AND PCL_FOUND AND paho-mqtt-cpp_FOUND AND nlohmann_json_FOUND)
    message(STATUS "ros2_amr_stack dependencies (OpenCV, PCL, Paho MQTT, nlohmann_json) found. Building libraries...")
    
    # -----------------------------------------------------------------------------
    # Standalone Subprojects (Libraries)
    # -----------------------------------------------------------------------------
    # Note: Added to the build sequence once their source skeletons are populated.
    # add_subdirectory(can_protocol)
    # add_subdirectory(world_sim_model)
    # add_subdirectory(perception_pipeline)
    # add_subdirectory(pose_fusion_ekf)
    # add_subdirectory(goal_controller)
    # add_subdirectory(cloud_telemetry)
else()
    message(WARNING "ros2_amr_stack dependencies (OpenCV, PCL, Paho MQTT, nlohmann_json) not found. Standalone AMR libraries will be skipped in local build.")
endif()
