#-------------------------------------------------
#
# Project created by QtCreator 2019-10-26T13:25:04
#
#-------------------------------------------------

QT       += core gui
QT += opengl
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = pcl_point_cloud_editor
TEMPLATE = app

# The following define makes your compiler emit warnings if you use
# any feature of Qt which has been marked as deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS
# You can also make your code fail to compile if you use deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000
# disables all the APIs deprecated before Qt 6.0.0


SOURCES += \
    src/cloud.cpp \
    src/cloudEditorWidget.cpp \
    src/cloudTransformTool.cpp \
    src/commandQueue.cpp \
    src/common.cpp \
    src/copyBuffer.cpp \
    src/cutCommand.cpp \
    src/deleteCommand.cpp \
    src/denoiseCommand.cpp \
    src/denoiseParameterForm.cpp \
    src/main.cpp \
    src/mainWindow.cpp \
    src/pasteCommand.cpp \
    src/select1DTool.cpp \
    src/select2DTool.cpp \
    src/selection.cpp \
    src/selectionTransformTool.cpp \
    src/statistics.cpp \
    src/statisticsDialog.cpp \
    src/trackball.cpp \
    src/transformCommand.cpp \
    src/displayDepthValue.cpp \
    src/ranging.cpp \
    src/screenpointconverter.cpp \
    src/highlightpoints.cpp \
    src/extracting.cpp \
    src/interactive_panel.cpp \
    src/recognition.cpp \
    src/pointcloudwidget.cpp \
    src/qrcodelocate.cpp \
    src/boundaryestimation.cpp \
    src/bestfitplane.cpp \
    src/euclideanseg.cpp \
    src/kdtreesearch.cpp \
    src/crackidentity.cpp \
    src/crackidentityform.cpp\
    src/groundfilter.cpp \
    src/groundfilterform.cpp\
   # src/test.cpp
    src/editpanel.cpp


HEADERS += \
    include/pcl/apps/point_cloud_editor/cloud.h \
    include/pcl/apps/point_cloud_editor/cloudEditorWidget.h \
    include/pcl/apps/point_cloud_editor/cloudTransformTool.h \
    include/pcl/apps/point_cloud_editor/command.h \
    include/pcl/apps/point_cloud_editor/commandQueue.h \
    include/pcl/apps/point_cloud_editor/common.h \
    include/pcl/apps/point_cloud_editor/copyBuffer.h \
    include/pcl/apps/point_cloud_editor/copyCommand.h \
    include/pcl/apps/point_cloud_editor/cutCommand.h \
    include/pcl/apps/point_cloud_editor/deleteCommand.h \
    include/pcl/apps/point_cloud_editor/denoiseCommand.h \
    include/pcl/apps/point_cloud_editor/denoiseParameterForm.h \
    include/pcl/apps/point_cloud_editor/localTypes.h \
    include/pcl/apps/point_cloud_editor/mainWindow.h \
    include/pcl/apps/point_cloud_editor/pasteCommand.h \
    include/pcl/apps/point_cloud_editor/select1DTool.h \
    include/pcl/apps/point_cloud_editor/select2DTool.h \
    include/pcl/apps/point_cloud_editor/selection.h \
    include/pcl/apps/point_cloud_editor/selectionTransformTool.h \
    include/pcl/apps/point_cloud_editor/statistics.h \
    include/pcl/apps/point_cloud_editor/statisticsDialog.h \
    include/pcl/apps/point_cloud_editor/toolInterface.h \
    include/pcl/apps/point_cloud_editor/trackball.h \
    include/pcl/apps/point_cloud_editor/transformCommand.h \
    include/pcl/apps/point_cloud_editor/displayDepthValue.h \
    include/pcl/apps/point_cloud_editor/ranging.h \
    include/pcl/apps/point_cloud_editor/screenpointconverter.h \
    include/pcl/apps/point_cloud_editor/highlightpoints.h \
    include/pcl/apps/point_cloud_editor/extracting.h \
    include/pcl/apps/point_cloud_editor/interactive_panel.h \
    include/pcl/apps/point_cloud_editor/recognition.h \
    include/pcl/apps/point_cloud_editor/pointcloudwidget.h \
    include/pcl/apps/point_cloud_editor/qrcodelocate.h \
    include/pcl/apps/point_cloud_editor/boundaryestimation.h \
    include/pcl/apps/point_cloud_editor/bestfitplane.h \
    include/pcl/apps/point_cloud_editor/euclideanseg.h \
    include/pcl/apps/point_cloud_editor/kdtreesearch.h \
    include/pcl/apps/point_cloud_editor/crackidentity.h \
    include/pcl/apps/point_cloud_editor/crackidentityform.h \
    include/pcl/apps/point_cloud_editor/groundfilter.h \
    include/pcl/apps/point_cloud_editor/groundfilterform.h \
    include/pcl/apps/point_cloud_editor/editpanel.h
  #   include/pcl/apps/point_cloud_editor/test.h

RESOURCES += \
    resources/pceditor_resources.qrc

DISTFILES += \
    resources/click.png \
    resources/copy.png \
    resources/cube.png \
    resources/cut.png \
    resources/delete.png \
    resources/info.png \
    resources/invert.png \
    resources/move.png \
    resources/open.png \
    resources/paste.png \
    resources/save.png \
    resources/select.png \
    resources/undo.png \
    resources/view.png \
    resources/pceditor.icns \
    resources/缩放大.png \
    resources/移动.png

INCLUDEPATH+=./include

#Vtk
INCLUDEPATH += /usr/local/include/vtk-8.2

LIBS += /usr/local/lib/libvtk*.so

#Boost
INCLUDEPATH += /usr/include/boost

LIBS += /usr/lib/x86_64-linux-gnu/libboost_*.so

#PCL Header
INCLUDEPATH += /usr/include/pcl-1.9

#PCL Lib
LIBS        += /usr/local/lib/libpcl_*.so

INCLUDEPATH+=/usr/include/qt5/QtOpenGL

#opengl

INCLUDEPATH+=/usr/include/GL

LIBS+=/usr/lib/x86_64-linux-gnu/libGL*.so

##opencv
INCLUDEPATH+=/usr/local/include/opencv4/

LIBS+=/usr/local/lib/libopencv*.so

#zbar
INCLUDEPATH+=/usr/include/zbar

LIBS+=/usr/lib/x86_64-linux-gnu/libzbar*

#Eigen
INCLUDEPATH += /usr/local/include/eigen3



