# Use the ZMQ vars and include dirs inherited from the parent scratch/CMakeLists

# Collect all sources from Attacks/
file(GLOB ATTACKS_SRC CONFIGURE_DEPENDS
  ${CMAKE_CURRENT_SOURCE_DIR}/Attacks/*.cc)

# Single executable from main.cc + Attacks/*.cc
set(SRC
  ${CMAKE_CURRENT_SOURCE_DIR}/main.cc
  ${ATTACKS_SRC}
)

# Mirror ns-3's output dir convention like the parent does
get_filename_component(absdir ${CMAKE_CURRENT_SOURCE_DIR} ABSOLUTE)
string(REPLACE "${PROJECT_SOURCE_DIR}" "${CMAKE_OUTPUT_DIRECTORY}" scratch_directory ${absdir})

# Find ZeroMQ library
find_library(ZMQ_LIB zmq)
if(NOT ZMQ_LIB)
    message(FATAL_ERROR "ZeroMQ library not found")
endif()

build_exec(
  EXECNAME drone_mesh
  EXECNAME_PREFIX scratch_drone-mesh_
  SOURCE_FILES "${SRC}"
  LIBRARIES_TO_LINK "${ns3-libs}" "${ns3-contrib-libs}" ${ZMQ_LIB}
  EXECUTABLE_DIRECTORY_PATH ${scratch_directory}/
)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${ZMQ_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/c_library_v2
)

target_compile_options(scratch_drone-mesh_drone_mesh PRIVATE
  -Wno-pedantic
  -Wno-address-of-packed-member
  -Wno-sign-compare
)
