cmake_minimum_required(VERSION 3.8)
project(temporis_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# ---- Temporis core library ----
# Header-only models + compiled sources from the Temporis project.
# Adjust TEMPORIS_CORE_DIR to point to your Temporis source tree.
set(TEMPORIS_CORE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/temporis_core"
    CACHE PATH "Path to Temporis core source (contains *.hpp and *.cpp)")

add_library(temporis_core STATIC
  ${TEMPORIS_CORE_DIR}/QueueLatencyModel.cpp
  ${TEMPORIS_CORE_DIR}/ZenohQueueModel.cpp
)
target_include_directories(temporis_core PUBLIC
  ${TEMPORIS_CORE_DIR}
)
target_compile_features(temporis_core PUBLIC cxx_std_17)

# ---- Shaper node ----
add_executable(temporis_shaper
  src/temporis_shaper_node.cpp
)
target_link_libraries(temporis_shaper temporis_core)
ament_target_dependencies(temporis_shaper rclcpp std_msgs)

install(TARGETS temporis_shaper
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(test_agent
  src/test_agent_node.cpp
)

target_link_libraries(test_agent temporis_core)
ament_target_dependencies(test_agent rclcpp std_msgs)

install(TARGETS test_agent
  DESTINATION lib/${PROJECT_NAME}
)

add_executable(temporis_per_agent_shaper
    src/temporis_per_agent_shaper_node.cpp
    temporis_core/QueueLatencyModel.cpp
    temporis_core/ZenohQueueModel.cpp
    # Topology.hpp is header-only
)
 
target_include_directories(temporis_per_agent_shaper PRIVATE
    include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/temporis_core
)
 
ament_target_dependencies(temporis_per_agent_shaper
    rclcpp
    std_msgs
    rcl_interfaces
)
 
install(TARGETS temporis_per_agent_shaper
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(consensus_agent src/consensus_agent_node.cpp)
ament_target_dependencies(consensus_agent rclcpp std_msgs)

install(TARGETS
  temporis_shaper
  test_agent
  consensus_agent  
  temporis_per_agent_shaper       
  DESTINATION lib/${PROJECT_NAME}
)

# ---- Launch files ----
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

# ---- Config files ----
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

ament_package()
