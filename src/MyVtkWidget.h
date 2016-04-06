#ifndef MYVTKWIDGET_H
#define MYVTKWIDGET_H

#include <QMainWindow>
#include <QWidget>
#include <QVTKWidget.h>

class MyVtkWidget: public QVTKWidget
{
	Q_OBJECT
	//property, no ";" at the back
	//Q_PROPERTY(float minZoom READ minimumZoom WRITE setMinimumZoom)
	//Q_PROPERTY(float zoomFactor READ zoomFactor WRITE setZoomFactor)

//signals:
	//void houghTransformationDone(QList<myPoint> &cloud);
	//void hoverOnPoint(QPointF p);

public slots:
	void loadCloud();

	
public:
	MyVtkWidget(QWidget *parent = 0);
	
};

#endif
