#-------------------------------------------------
#
# Project created by QtCreator 2013-09-11T14:21:21
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Smog
TEMPLATE = app

INCLUDEPATH += /usr/include/vtk-5.8

LIBS += -lQVTK \
    -lboost_system \
    -lpcl_visualization \
    -lvtkCommon \
    -lvtkRendering \
    -lvtkFiltering \
    -lpcl_common \
    -lpcl_io

QMAKE_CXXFLAGS += -std=c++11

SOURCES += main.cpp \
    SmogMainWindow.cpp \
    SmogController.cpp \
    QPointCloudVisualizer.cpp

HEADERS  += \
    SmogMainWindow.hpp \
    SmogController.hpp \
    QPointCloudVisualizer.hpp

FORMS    += SmogMainWindow.ui

RESOURCES += icons.qrc
