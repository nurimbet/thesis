project(ompl_custom)
cmake_minimum_required(VERSION 2.8)

file(GLOB_RECURSE OMPLDRRTSFN_SOURCE_CODE base/*.cpp geometric/*.cpp)
add_library(${PROJECT_NAME} ${OMPLDRRTSFN_SOURCE_CODE})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
