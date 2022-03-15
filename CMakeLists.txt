cmake_minimum_required(VERSION 3.10)

# set the project name
project(Connected_Components)

# add the executable
add_executable(graph
                Graph/Graph.cpp
                main.cpp)

include_directories(
    Graph
)

file(GLOB MY_FILES "*.txt")

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Graph/in.txt
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/Graph/in2.txt
     DESTINATION ${CMAKE_CURRENT_BINARY_DIR})