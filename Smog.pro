#-------------------------------------------------
#
# Project created by QtCreator 2013-09-11T14:21:21
#
#-------------------------------------------------

QT       += core gui sql opengl

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
    -lpcl_octree \
    -llas \
    -lGLU

QMAKE_CXXFLAGS += -std=c++11 -Wno-deprecated -Wno-sign-compare

include(main.pri)
include(qtext.pri)
include(backend.pri)
include(tools.pri)
include(math.pri)
include(data.pri)
