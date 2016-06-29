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
src/ImageTileWidget.cpp
src/MatchWidget.cpp
src/MatchPanel.cpp
src/MatchPanelModel.cpp
src/VisibleImagesPanel.cpp
src/KeyFramePanel.cpp
src/KeyFrameModel.cpp
src/core/PlyIO.cpp
src/core/ProjectIO.cpp
src/CoreInterfaceWidget.cpp
src/core/Utils.cpp
src/core/PtCloud.cpp
src/core/PolygonModel.cpp
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
src/core/ProjectIO.h
src/core/SFMPipeline.h
src/core/Utils.h
src/core/PtCloud.h
src/core/PolygonModel.h
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
src/core/ptam/KeyFrame.cc
src/core/ptam/KeyFrame.h
src/core/ptam/Params.cpp
src/core/ptam/Params.h
src/core/ptam/ShiTomasi.cc
src/core/ptam/ShiTomasi.h
src/core/ptam/SmallBlurryImage.cc
src/core/ptam/SmallBlurryImage.h
src/core/ptam/MapPoint.h
src/core/ptam/MapPoint.cc
src/core/ptam/ATANCamera.cc
src/core/ptam/ATANCamera.h
src/core/ptam/LevelHelpers.h
)
SET(HEADERS 
src/MainWindow.h
src/CloudWidget.h
src/CoreInterfaceWidget.h
src/ImageWidget.h
src/ImageTileWidget.h
src/MatchWidget.h
src/MatchPanel.h
src/VisibleImagesPanel.h
src/MatchPanelModel.h
src/KeyFramePanel.h
src/KeyFrameModel.h
)
QT4_WRAP_CPP(MOC ${HEADERS})


include_directories(
  include
  ${PROJECT_SOURCE_DIR}/../CamTrack/thirdparty/TooN/include 
  ${PROJECT_SOURCE_DIR}/../CamTrack/thirdparty/libcvd/include 
  ${PROJECT_SOURCE_DIR}/../CamTrack/thirdparty/gvars3/include 
  ${PROJECT_SOURCE_DIR}/../CamTrack/thirdparty/agast/include
  ${PROJECT_SOURCE_DIR}/../CamTrack/thirdparty/tinyxml2/include
)  

link_directories(
  ${PROJECT_SOURCE_DIR}/../CamTrack/thirdparty/libcvd/lib 
  ${PROJECT_SOURCE_DIR}/../CamTrack/thirdparty/gvars3/lib 
  ${PROJECT_SOURCE_DIR}/../CamTrack/thirdparty/agast/lib
  ${PROJECT_SOURCE_DIR}/../CamTrack/thirdparty/tinyxml2/lib
)

set(PTAM_LIBRARIES GL glut cvd GVars3_headless agast tinyxml2 blas lapack)

ADD_DEFINITIONS(${QT_DEFINITIONS})
ADD_EXECUTABLE(ManualSFM ${SOURCES} ${MOC})
TARGET_LINK_LIBRARIES(ManualSFM ${PTAM_LIBRARIES} ${QT_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBS} QVTK cvsba)
