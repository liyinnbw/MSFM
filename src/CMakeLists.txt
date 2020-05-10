set(BINARY ${CMAKE_PROJECT_NAME})
#add_subdirectory(opengv)

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Qt5 COMPONENTS Widgets REQUIRED)
FIND_PACKAGE(VTK REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Ceres REQUIRED)
#FIND_PACKAGE (opengv PATHS "${PROJECT_SOURCE_DIR}/thirdparty/cmake" NO_DEFAULT_PATH REQUIRED)

INCLUDE(${VTK_USE_FILE})
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

#if(CMAKE_VERSION VERSION_LESS "3.7.0")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
#endif()

SET(SOURCES 
ui/main.cpp 
ui/MainWindow.cpp
ui/CloudWidget.cpp
ui/ImageWidget.cpp
ui/ImageTileWidget.cpp
ui/MatchWidget.cpp
ui/MatchPanel.cpp
ui/MatchPanelModel.cpp
ui/VisibleImagesPanel.cpp
ui/EpipolarPanel.cpp
#ui/GraphWidget.cpp
#ui/VideoPanel.cpp
#ui/FramePanel.cpp
ui/CoreInterfaceWidget.cpp
core/ProjectIO.cpp
#core/InputFrameProcessor.cpp
core/Utils.cpp
core/BAHandler.cpp
core/SFMPipeline.cpp
#core/SFMVideoPipeline.cpp
core/Camera.cpp
core/DetectorExtractor.cpp
#core/ceres/EigenQuaternionParameterization.cpp
core/ProjectIO.h
core/SFMPipeline.h
#core/SFMVideoPipeline.h
#core/InputFrameProcessor.h
#core/ThreadsafeQueue.h
core/Utils.h
core/BAHandler.h
core/Camera.h
core/DetectorExtractor.h
core/datastructs/Frame.h
core/datastructs/LandMark.h
core/datastructs/Data.h
core/datastructs/Measurement.h
#core/ceres/EigenQuaternionParameterization.h
)


include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  #${PROJECT_SOURCE_DIR}/ui
  #${PROJECT_SOURCE_DIR}/core
)  


#ADD_EXECUTABLE(${BINARY} ${SOURCES} ${AGAST_SOURCES} ${BRISK_SOURCES} ${DBOW3_SOURCES} )
#TARGET_LINK_LIBRARIES(${BINARY} Qt5::Widgets ${VTK_LIBRARIES} ${OpenCV_LIBS} QVTK opengv ${CERES_LIBRARIES})
ADD_EXECUTABLE(${BINARY} ${SOURCES})
TARGET_LINK_LIBRARIES(${BINARY} Qt5::Widgets ${VTK_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES} )

#needed by test
add_library(${BINARY}_lib STATIC ${SOURCES})
TARGET_LINK_LIBRARIES(${BINARY}_lib Qt5::Widgets ${VTK_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
