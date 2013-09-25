#-------------------------------------------------
#
# Project created by QtCreator 2013-09-11T14:21:21
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = Smog
TEMPLATE = app

INCLUDEPATH += /usr/include/vtk-5.8 \
    /usr/include/qt4 \
    /usr/include/pcl-1.7 \
    /usr/include/eigen3

LIBS += -lQVTK \
    -lboost_system \
    -lpcl_visualization \
    -lvtkCommon \
    -lvtkRendering \
    -lvtkFiltering \
    -lpcl_common \
    -lpcl_io \
    -lpcl_features \
    -llas

QMAKE_CXXFLAGS += -std=c++11

include(main.pri)
include(qtpcl.pri)
include(backend.pri)
include(tools.pri)
