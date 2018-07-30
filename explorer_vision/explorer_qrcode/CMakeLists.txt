cmake_minimum_required(VERSION 2.8.3)
project(explorer_qrcode)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  hector_worldmodel_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES explorer_qrcode
 CATKIN_DEPENDS hector_worldmodel_msgs cv_bridge image_transport message_generation roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)
option(BUILD_TESTING "Enable generation of test targets" OFF)

set(CMAKE_LIBRARY_PATH /opt/local/lib ${CMAKE_LIBRARY_PATH})


# Adjust CMake's module path.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/FindModules)

# Suppress MSVC CRT warnings.
if(MSVC)
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    add_definitions(/Za)
    add_definitions(/MP) # multi-core builds
endif()

include(source_files.cmake)

if(WIN32)
    include_directories(core/lib/win32)
    set(CMAKE_DEBUG_POSTFIX -debug)
endif()

# OpenCV classes
find_package(OpenCV)
if(OpenCV_FOUND)
    list(APPEND LIBZXING_FILES
        opencv/src/zxing/MatSource.cpp
        opencv/src/zxing/MatSource.h
    )

    include_directories(${OpenCV_INCLUDE_DIRS})
    include_directories(opencv/src)
endif()

include_directories(core/src)
add_library(libzxing ${LIBZXING_FILES})
set_target_properties(libzxing PROPERTIES PREFIX "")

find_package(Iconv)
if(ICONV_FOUND)
    include_directories(${ICONV_INCLUDE_DIR})
    target_link_libraries(libzxing ${ICONV_LIBRARIES})
else()
    add_definitions(-DNO_ICONV=1)
endif()

# Add cli executable.
file(GLOB_RECURSE ZXING_FILES
    "./src/*.cpp"
    "./src/*.h"
)


include_directories(include include/qrcode_detection)
include_directories(include 3rdparty/zbar)
include_directories(
  ${catkin_INCLUDE_DIRS}
)



## Declare a cpp executable
 add_executable(qrcode_detection_node ${ZXING_FILES} )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
 add_dependencies(qrcode_detection_node qrcode_detection_generate_messages_cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(qrcode_detection_node
   ${catkin_LIBRARIES}
   ${ZBAR_LIBRARIES} zbar
   ${OpenCV_LIBS}
   libzxing
 )

 install(TARGETS qrcode_detection_node libzxing EXPORT zxing-targets
 LIBRARY DESTINATION lib
 RUNTIME DESTINATION bin
 ARCHIVE DESTINATION lib
   INCLUDES DESTINATION include
)

install(EXPORT zxing-targets DESTINATION lib/zxing/cmake NAMESPACE zxing::)

install(
   DIRECTORY core/src/zxing/
   DESTINATION include/zxing
   FILES_MATCHING PATTERN "*.h"
)

if(OpenCV_FOUND)
   install(
       DIRECTORY opencv/src/zxing/
       DESTINATION include/zxing
       FILES_MATCHING PATTERN "*.h"
   )
endif()

configure_file(cmake/zxing-config.cmake.in zxing-config.cmake @ONLY)
install(FILES ${CMAKE_BINARY_DIR}/zxing-config.cmake DESTINATION lib/zxing/cmake)

if(BUILD_TESTING)
   # Add testrunner executable.
   find_package(CPPUNIT)
   if(CPPUNIT_FOUND)
       file(GLOB_RECURSE TESTRUNNER_FILES
           "./core/tests/src/*.cpp"
           "./core/tests/src/*.h"
       )
       add_executable(testrunner ${TESTRUNNER_FILES})
       include_directories(${CPPUNIT_INCLUDE_DIR})
       target_link_libraries(testrunner libzxing  ${CPPUNIT_LIBRARIES})
   endif()
endif()


