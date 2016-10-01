#-------------------------------------------------
#
# Project created by QtCreator 2014-08-12T10:47:31
#
#-------------------------------------------------

QT       += core
#QT       += opengl

QT       -= gui

TARGET = SFM_OCV
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += main.cpp \
    #myGLWidget.cpp \
    #myWindow.cpp \
    #camera.cpp

unix {
    CONFIG += link_pkgconfig
    PKGCONFIG += opencv
}

INCLUDEPATH += /usr/include/opencv2/

LIBS += -L /usr/lib/
-libopencv_core
-libopencv_highgui
-libopencv_features2d
-libopencv_legacy
-libopencv_imgproc
-libopencv_calib3d

LIBS = -lglut -lGLU


HEADERS += \
    SFM.h \
    GNA.h \
    LF.h \
    histogram.h \
    ../../MAT/Mat.h \
    ../../ONSC/ONSCNewton.h \
    ../../ONSC/ONSC.h \
    ../../ONSC/OO.h \
    ../../MVG/MVG.h \
    ../../RAND/rand.h \
    #myGLWidget.h \
    #myWindow.h \
    camera.h \
    ../../EKF/v2/EKFO.h \
    ../../MVG/Frame.h \
    ../../ONSC/LS.h


