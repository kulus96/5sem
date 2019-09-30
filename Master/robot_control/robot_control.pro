TEMPLATE = app
CONFIG += console c++11
CONFIG -= app_bundle
CONFIG -= qt


INCLUDEPATH += $$PWD/../../../fuzzylite/fuzzylite
LIBS += -L$$OUT_PWD/../../../fuzzylite/fuzzylite/release/bin -lfuzzylite-static

SOURCES += main.cpp \
    lidar.cpp \
    camera.cpp

CONFIG += link_pkgconfig
PKGCONFIG += gazebo
PKGCONFIG += opencv

HEADERS += \
    lidar.h \
    camera.h
