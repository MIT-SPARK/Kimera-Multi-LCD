cmake_minimum_required(VERSION 3.5)
project(kimera_multi_lcd)

SET(CMAKE_BUILD_TYPE Release)

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

catkin_simple()


cs_add_library(${PROJECT_NAME} 
  src/LoopClosureDetector.cpp
  src/LcdThirdPartyWrapper.cpp
  src/types.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  gtsam
  opengv
  DBoW2::DBoW2
)

catkin_add_gtest(test_lcd test/test_lcd.cpp)
target_link_libraries(test_lcd ${PROJECT_NAME} -ltbb)

cs_install()

cs_export()
