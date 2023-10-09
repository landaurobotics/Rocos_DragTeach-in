cmake_minimum_required(VERSION 3.10)

project(rocos_test)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
list(APPEND CMAKE_PREFIX_PATH /opt/rocos/app)
list(APPEND CMAKE_PREFIX_PATH /opt/grpc)
include_directories(/opt/grpc/include)

find_package(rocos_app REQUIRED)
find_package(Eigen3 REQUIRED)
add_subdirectory(3rdparty/gflags)
add_subdirectory(3rdparty/filter_iir)
add_subdirectory(3rdparty/kdl_parser)

set( List_Cpp
demo 
# demo_teach_6
# demo_torque_7joint
# demo_teach_246
demo_teach_all
demo_teach_all_stop
demo_teach_stop


)
set(LIST ${List_Cpp} )
foreach(list ${LIST})

  add_executable(${list} src/${list}.cpp  )
  target_link_libraries(${list} PUBLIC rocos::hardware rocos::kinematics rocos::drive rocos::protocol rocos::robot  Eigen3::Eigen gflags::gflags filter_iir kdl_parser)

endforeach()


# add_executable(demo src/demo.cpp)
# target_link_libraries(demo PUBLIC rocos::hardware rocos::kinematics rocos::drive rocos::protocol rocos::robot  Eigen3::Eigen gflags::gflags )



#file(GLOB_RECURSE ${CMAKE_SOURCE_DIR}/config)
file(GLOB_RECURSE CONFIG_FILE config/*.urdf config/public.key config/license)

file(COPY ${CONFIG_FILE} DESTINATION ${CMAKE_BINARY_DIR}/bin/config)

#file(COPY ${CMAKE_SOURCE_DIR}/config DESTINATION ${CMAKE_BINARY_DIR}/bin)

