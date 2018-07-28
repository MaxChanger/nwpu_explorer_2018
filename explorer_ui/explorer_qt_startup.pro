#-------------------------------------------------
#
# Project created by QtCreator 2018-07-19T16:11:53
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = explorer_qt_startup
TEMPLATE = app


SOURCES += \
            main.cpp\
            mainwindow.cpp \
    login.cpp \
    qterminal.cpp \
    qroscam.cpp \
    explorer_ui_control/src/explorer_thread.cpp \
    explorer_ui_control/src/explorer_ui_control.cpp \
    explorer_thread.cpp \
    explorer_ui_control.cpp \
    explorer_ui_ability.cpp

HEADERS  +=\
            mainwindow.h \
    login.h \
    qterminal.h \
    qroscam.h \
    explorer_ui_control/src/explorer_thread.h \
    explorer_ui_control/src/explorer_ui_control.h \
    explorer_thread.h \
    explorer_ui_control.h \
    explorer_ui_ability.h

FORMS    += \
            mainwindow.ui \
    login.ui \
    qterminal.ui \
    qroscam.ui \
    explorer_ui_control/src/explorer_ui_control.ui \
    explorer_ui_control.ui \
    explorer_ui_ability.ui

RESOURCES += \
    explorer_resource.qrc

SUBDIRS += \
    explorer_ui_control/src/explorer_ui_control.pro


INCLUDEPATH += /opt/ros/kinetic/include
DEPENDPATH +=   /opt/ros/kinetic/include
LIBS += -L/opt/ros/kinetic/lib -lroscpp -lrospack -lpthread -lrosconsole -lrosconsole_log4cxx -lrosconsole_backend_interface -lxmlrpcpp -lroscpp_serialization -lrostime  -lcpp_common  -lroslib -ltf  -lyaml-cpp -lkdl_conversions
LIBS += -l:/opt/ros/kinetic/lib/libroslib.so
