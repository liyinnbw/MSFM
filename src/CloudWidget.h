/*
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#ifndef CLOUDWIDGET_H
#define CLOUDWIDGET_H

#include <QMainWindow>
#include <QWidget>
#include <QVTKWidget.h>
#include <QList>
#include <QVector3D>
#include <opencv2/core/core.hpp>

class vtkPolyDataMapper;
class vtkActor;
class vtkPolyData;
class vtkIdFilter;
class vtkDataSetSurfaceFilter;
class InteractorStyle;
class vtkRenderer;
class vtkRenderWindow;
class vtkAreaPicker;
class vtkRenderWindowInteractor;
class CloudWidget: public QVTKWidget
{
	Q_OBJECT
	//property, no ";" at the back
	//Q_PROPERTY(float minZoom READ minimumZoom WRITE setMinimumZoom)
	//Q_PROPERTY(float zoomFactor READ zoomFactor WRITE setZoomFactor)

signals:
	void deletePointIdx(const QList<int> idxs);
	void showCamerasSeeingPoints(const QList<int> idxs);

public slots:
	void highlightPointIdx(const QList<int> idxs, const int camIdx);
	void highlightPoints(const QList<QVector3D> &xyzs, const int camIdx);
	
public:
	CloudWidget(QWidget *parent = 0);
	void deletePoints(const QList<int> idxs);
	void showCameras(const QList<int> idxs);
	//void loadCloud(const std::vector<cv::Point3f> &xyzs);
	void loadCloudAndCamera(const std::vector<cv::Point3f> &xyzs, const std::vector<cv::Point3f> &norms, const std::vector<cv::Matx34d> &cams, bool resetView);
	void loadPolygonAndCamera(const std::vector<cv::Point3f> &verts, const std::vector<cv::Point3i> &faces, const std::vector<cv::Matx34d> &cams, bool resetView);

	const static int 			POINT_SIZE = 3;
	const static int 			LINE_WIDTH = 2;
	int 						numCloudPoints;
private:
	void disableInteraction();
	void enableInteraction();

	vtkPolyDataMapper 			*mapper;
	vtkActor 					*actor;
	vtkPolyData 				*pointsData;
	vtkIdFilter 				*idFilter;
	vtkDataSetSurfaceFilter		*surfaceFilter;
	InteractorStyle				*style;
	vtkRenderer					*renderer;
	vtkRenderWindow				*renderWindow;
	vtkAreaPicker				*areaPicker;
	vtkRenderWindowInteractor	*renderWindowInteractor;
	



};

#endif
