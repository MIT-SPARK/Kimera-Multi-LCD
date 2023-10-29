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