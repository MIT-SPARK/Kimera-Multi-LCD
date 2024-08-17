### Download and unzip the vocabulary file
message("[Kimera-Multi-LCD] Trying to download and unzip the vocabulary file...")
if(NOT EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/vocab/mit_voc.yml)
  message("[Kimera-Multi-LCD] Downloading vocabulary file from drive.")
  file(DOWNLOAD
       https://github.com/LimHyungTae/kimera-multi-vocab/raw/master/vocab.zip
       ${CMAKE_CURRENT_SOURCE_DIR}/vocab.zip
       STATUS voc_download_success
       )
  if(voc_download_success)
    message("[Kimera-Multi-LCD] Unzipping vocabulary file...")

    execute_process(COMMAND ${CMAKE_COMMAND} -E tar xzf ${CMAKE_CURRENT_SOURCE_DIR}/vocab.zip
                            WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/)
    execute_process(COMMAND ${CMAKE_COMMAND} -E rm ${CMAKE_CURRENT_SOURCE_DIR}/vocab.zip)
    message("[Kimera-Multi-LCD] Complete! Check your `${Kimera-Multi-LCD}/vocab/mit_voc.yml.")
  else()
    message(FATAL_ERROR "[Kimera-Multi-LCD] Failed to download vocabulary file. Please download manually.")
  endif()
else()
  message("[Kimera-Multi-LCD] Vocabulary file exists, will not download.")
endif()
