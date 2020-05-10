/*
 *  Interactive widget to display reconstructed scene
 */


#ifndef CLOUDWIDGET_H
#define CLOUDWIDGET_H

#include <map>

#include <QMainWindow>
#include <QWidget>
#include <QVTKWidget.h>
#include <QList>
#include <QVector3D>
#include <opencv2/core/core.hpp>

#include "core/datastructs/Data.h"

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
	void requestDeletePoints(const QList<int> idxs);
	void requestShowPointsMeasures(const QList<int> idxs);
	void requestShowAllProjections();

public:
	CloudWidget(QWidget *parent = 0);
	void deletePoints(const QList<int> idxs);
	void showPointsMeasurements(const QList<int> idxs);
	void showAllProjections();
	
	void visualize(bool resetView);
	void visualizeMeasurements(std::vector<Measurement::Ptr> &ms);

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
	
	std::map<vtkIdType, LandMark::Ptr> 								ptIdx2lm;
	std::map<vtkIdType, Frame::Ptr> 								camIdx2f;
	std::map<LandMark::Ptr, vtkIdType, Data::LandMarkPtrCompare> 	lm2ptIdx;
	std::map<Frame::Ptr, vtkIdType, Data::FramePtrCompare> 			f2camIdx;

};

#endif
