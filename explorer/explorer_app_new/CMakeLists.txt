cmake_minimum_required(VERSION 2.8.3)
project(explorer_app_new)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rqt_gui
  rqt_gui_cpp
  std_msgs
  sensor_msgs
  image_transport
  sensor_msgs
  cv_bridge
  explorer_msgs
)



find_package(Boost REQUIRED COMPONENTS system)
find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
find_package(class_loader)

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITION})

catkin_python_setup()

set(explorer_app_new_SRCS
    src/explorer_app_new.cpp
    src/light_warn.cpp
    src/victim_choose.cpp
    src/widget.cpp
    )

set(explorer_app_new_HDRS
    include/explorer_app_new/ui_widget.h
    include/explorer_app_new/explorer_app_new.h
    include/explorer_app_new/light_warn.h
    include/explorer_app_new/ui_victim_choose.h
    include/explorer_app_new/victim_choose.h
    include/explorer_app_new/widget.h
)

set(explorer_app_new_UIS
    ui/victim_choose.ui
    ui/widget.ui
)

set(explorer_app_new_INCLUDE_DIRECTORIES
    include
    ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy rqt_gui rqt_gui_cpp sensor_msgs std_msgs
  DEPENDS system_lib
)


file(GLOB QT_RESOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} resources/*.qrc)

QT4_ADD_RESOURCES(QT_RESOURCES_CPP ${QT_RESOURCES})
QT4_WRAP_CPP(explorer_app_new_MOCS ${explorer_app_new_HDRS})
QT4_WRAP_UI(explorer_app_new_UIS_H ${explorer_app_new_UIS})

include_directories(${explorer_app_new_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${explorer_app_new_SRCS} ${explorer_app_new_MOCS} ${explorer_app_new_UIS_H} ${QT_RESOURCES_CPP})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})

add_dependencies(${PROJECT_NAME} explorer_msgs_generate_messages)
