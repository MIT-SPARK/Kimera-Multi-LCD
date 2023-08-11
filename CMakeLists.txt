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

catkin_simple()


cs_add_library(${PROJECT_NAME} 
  src/LoopClosureDetector.cpp
  src/LcdThirdPartyWrapper.cpp
  src/types.cpp
  src/utils.cpp
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
)

### Download and unzip the vocabularly file
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/vocab/mit_voc.yml)
  message(STATUS "Downloading vocabulary file from drive.")
  file(DOWNLOAD
       https://drive.google.com/uc?export=download&confirm=9iBg&id=1N4y0HbgA3PHQ73ZxFJvy5dgvV_0cTBYF
       ${CMAKE_CURRENT_SOURCE_DIR}/vocab.zip
       SHOW_PROGRESS
       STATUS voc_download_success
       TIMEOUT 60)
  if(voc_download_success)
    message(STATUS "Unzipping vocabulary file.")

    execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf ${CMAKE_CURRENT_SOURCE_DIR}/vocab.zip
                            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/)
    execute_process(COMMAND ${CMAKE_COMMAND} -E remove ${CMAKE_CURRENT_SOURCE_DIR}/vocab.zip)
  else(voc_download_success)
    message(STATUS "Failed to download vocabulary file. Please download manually.")
  endif(voc_download_success)
else()
  message(STATUS "Vocabulary file exists, will not download.")
endif()

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
