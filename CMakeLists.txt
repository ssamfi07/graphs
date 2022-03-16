cmake_minimum_required(VERSION 3.10)

# set the project name
project(Connected_Components)

# add definitions
add_definitions(-lboost_log -DBOOST_ALL_DYN_LINK -lpthread -lboost_date_time)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -lboost_log -DBOOST_ALL_DYN_LINK -lpthread -lboost_date_time")

find_package(Boost COMPONENTS program_options log log_setup REQUIRED)
set(PROJECT_LIB ${PROJECT_LIB} ${Boost_LOG_LIBRARY} ${Boost_LOG_SETUP_LIBRARY})

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

target_link_libraries(graph pthread)
target_link_libraries(graph ${PROJECT_LIB})