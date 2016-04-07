cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
PROJECT(ManualSFM)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Qt4 REQUIRED)
FIND_PACKAGE(VTK REQUIRED)
INCLUDE(${VTK_USE_FILE})
INCLUDE(${QT_USE_FILE})

SET(SOURCES 
src/main.cpp 
src/MainWindow.cpp
src/CloudWidget.cpp
src/ImageWidget.cpp
src/MatchWidget.cpp
src/MatchPanel.cpp
src/MatchPanelModel.cpp
src/core/PlyIO.cpp
src/CoreInterfaceWidget.cpp
src/core/Utils.cpp
src/core/PtCloud.cpp
src/core/PathReader.cpp
src/core/BAHandler.cpp
src/core/SFMPipeline.cpp
src/core/ba/colamd.c
src/core/ba/colamd_global.c
src/core/ba/v3d_metricbundle.cpp
src/core/ba/v3d_nonlinlsq.cpp
src/core/ba/v3d_optimization.cpp
src/core/ba/v3d_optimization_lm.cpp
src/core/ba/v3d_stereobundle.cpp
src/core/PlyIO.h
src/core/SFMPipeline.h
src/core/Utils.h
src/core/PtCloud.h
src/core/PathReader.h
src/core/BAHandler.h
src/core/ba/colamd.h
src/core/ba/SuiteSparse_config.h
src/core/ba/v3d_blockmatrix.h
src/core/ba/v3d_cameramatrix.h
src/core/ba/v3d_distortion.h
src/core/ba/v3d_exception.h
src/core/ba/v3d_ldl_private.h
src/core/ba/v3d_linear.h
src/core/ba/v3d_linearbase.h
src/core/ba/v3d_linear_ldlt.h
src/core/ba/v3d_linear_lu.h
src/core/ba/v3d_linear_tnt.h
src/core/ba/v3d_mathutilities.h
src/core/ba/v3d_metricbundle.h
src/core/ba/v3d_mviewutilities.h
src/core/ba/v3d_nonlinlsq.h
src/core/ba/v3d_optimization.h
src/core/ba/v3d_serialization.h
src/core/ba/v3d_stereobundle.h
src/core/ba/v3d_vrmlio.h
)
SET(HEADERS 
src/MainWindow.h
src/CloudWidget.h
src/CoreInterfaceWidget.h
src/ImageWidget.h
src/MatchWidget.h
src/MatchPanel.h
src/MatchPanelModel.h
)
QT4_WRAP_CPP(MOC ${HEADERS})

ADD_DEFINITIONS(${QT_DEFINITIONS})
ADD_EXECUTABLE(ManualSFM ${SOURCES} ${MOC})
TARGET_LINK_LIBRARIES(ManualSFM ${QT_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} QVTK)
