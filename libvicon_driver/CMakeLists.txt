cmake_minimum_required(VERSION 2.8.3)
project(vicon_driver)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/../cmake")
find_package(Eigen3 REQUIRED)

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)
add_definitions("-Wall -Wextra -O2 -fPIC")

# Default to use the old YAML-CPP API
set(USE_OLD_YAML_CPP_API TRUE CACHE BOOL "Use the old YAML-CPP API")
if(USE_OLD_YAML_CPP_API)
  message(STATUS "${PROJECT_NAME}: Using old YAML-CPP API")
  add_definitions( -DUSE_OLD_YAML_CPP_API )
endif(USE_OLD_YAML_CPP_API)

if(APPLE)
  set(SO_EXT ".dylib")
  set(VICON_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/vicon_sdk/MacOSX")
else(APPLE)
  set(SO_EXT ".so")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-O1 -Wl,--as-needed")

  if(CMAKE_SIZEOF_VOID_P MATCHES 8)
    set(VICON_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/vicon_sdk/Linux64")
  else(CMAKE_SIZEOF_VOID_P MATCHES 8)
    set(VICON_SDK_DIR "${CMAKE_CURRENT_LIST_DIR}/vicon_sdk/Linux32")
  endif(CMAKE_SIZEOF_VOID_P MATCHES 8)
endif(APPLE)

include_directories(${VICON_SDK_DIR})
link_directories(${VICON_SDK_DIR})

add_library(ViconDataStreamSDK_CPP SHARED IMPORTED)
set_target_properties(ViconDataStreamSDK_CPP PROPERTIES
  IMPORTED_LOCATION "${VICON_SDK_DIR}/libViconDataStreamSDK_CPP${SO_EXT}"
)

add_library(vicon_driver
  src/vicon_driver.cpp
  src/vicon_calib.cpp
)

target_link_libraries(vicon_driver
  yaml-cpp
  ViconDataStreamSDK_CPP
)

install(TARGETS vicon_driver DESTINATION lib)
install(FILES "${VICON_SDK_DIR}/libViconDataStreamSDK_CPP${SO_EXT}" DESTINATION lib)
