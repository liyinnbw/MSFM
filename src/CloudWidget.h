#ifndef CLOUDWIDGET_H
#define CLOUDWIDGET_H

#include <QMainWindow>
#include <QWidget>
#include <QVTKWidget.h>
#include <QList>

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

public slots:
	void loadCloud(QString fileName);

	
public:
	CloudWidget(QWidget *parent = 0);
	void deletePoints(const QList<int> idxs);
	void loadCloud(std::vector<cv::Point3f> &xyzs);
	

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
