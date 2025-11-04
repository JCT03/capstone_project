cmake_minimum_required(VERSION 3.18 FATAL_ERROR)
project(neural_network)

set(CMAKE_PREFIX_PATH "/usr/local/Cellar/pytorch/2.8.0_2")

set(TORCH_WARN_ON_MISSING_LIB_DEPS OFF)

find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

add_executable(collision_network_builder collision_network_builder.cpp)

target_link_libraries(collision_network_builder "${TORCH_LIBRARIES}")

set_property(TARGET collision_network_builder PROPERTY CXX_STANDARD 17)