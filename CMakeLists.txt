cmake_minimum_required(VERSION 3.5)
project(kimera_multi_lcd)

add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin_simple REQUIRED)

find_package(Boost REQUIRED
  date_time
  serialization
  thread
  filesystem
  system
  regex
  timer
  chrono
)

find_package(GTSAM REQUIRED)
find_package(opengv REQUIRED)
find_package(DBoW2 REQUIRED)
if(NOT TARGET DBoW2::DBoW2)
  add_library(DBoW2::DBoW2 INTERFACE IMPORTED)
  set_target_properties(DBoW2::DBoW2 PROPERTIES
  INTERFACE_LINK_LIBRARIES "${DBoW2_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${DBoW2_INCLUDE_DIRS}")
endif()
find_package(PCL REQUIRED)

configure_file(cmake/json.CMakeLists.txt.in json-download/CMakeLists.txt)
execute_process(
  COMMAND "${CMAKE_COMMAND}" -G "${CMAKE_GENERATOR}" .
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/json-download" OUTPUT_QUIET
)
execute_process(
  COMMAND "${CMAKE_COMMAND}" --build .
  WORKING_DIRECTORY "${CMAKE_BINARY_DIR}/json-download" OUTPUT_QUIET
)

set(JSON_BuildTests OFF CACHE INTERNAL "")
set(JSON_Install OFF CACHE INTERNAL "")
add_subdirectory(${CMAKE_BINARY_DIR}/json-src ${CMAKE_BINARY_DIR}/json-build)

catkin_simple()


cs_add_library(${PROJECT_NAME} 
  src/loop_closure_detector.cpp
  src/lcd_third_party.cpp
  src/types.cpp
  src/utils.cpp
  src/serializer.cpp
  src/io.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gtsam
  opengv
  DBoW2::DBoW2
  tbb
  glog
  nlohmann_json::nlohmann_json
)

include(cmake/vocab_download.cmake)

# Unit tests 
# set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/data")
set(TEST_VOCAB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/vocab")
configure_file(test/test_config.h.in test/test_config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/test)

catkin_add_gtest(${PROJECT_NAME}-test_lcd test/test_lcd.cpp)
target_link_libraries(${PROJECT_NAME}-test_lcd ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}-test_utils test/test_utils.cpp)
target_link_libraries(${PROJECT_NAME}-test_utils ${PROJECT_NAME})

cs_install()

cs_export()
