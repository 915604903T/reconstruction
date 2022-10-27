##############################
# SetScratchTestTarget.cmake #
##############################

INCLUDE(${PROJECT_SOURCE_DIR}/cmake/Flags.cmake)

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/bin/tests/scratch/${targetname})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_DEBUG ${PROJECT_BINARY_DIR}/bin/tests/scratch/${targetname})
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_RELEASE ${PROJECT_BINARY_DIR}/bin/tests/scratch/${targetname})
ADD_EXECUTABLE(${targetname} ${sources} ${headers} ${templates})
INCLUDE(${PROJECT_SOURCE_DIR}/cmake/VCLibraryHack.cmake)

IF(MSVC_IDE)
  SET_TARGET_PROPERTIES(${targetname} PROPERTIES LINK_FLAGS_DEBUG "/DEBUG")
ENDIF()
