/*
 *  Created on: Jun 16, 2016
 *      Author: yoyo
 */

#ifndef KEYFRAMEWIDGET_H
#define KEYFRAMEWIDGET_H

#include <QMainWindow>
#include <QWidget>
#include <QList>

#include "ImageWidget.h"


class QImage;
class QPointF;
class QPixmap;
class KeyFrameWidget: public ImageWidget
{
	Q_OBJECT
	//property, no ";" at the back
	Q_PROPERTY(float minZoom READ minimumZoom WRITE setMinimumZoom)
	Q_PROPERTY(float zoomFactor READ zoomFactor WRITE setZoomFactor)

public:
	KeyFrameWidget(QWidget *parent = 0);
	void setProjectionVerts(const QList<QPointF> &_verts);

protected:
	virtual void drawMarks();

private:
	QList<QPointF> 			verts;
};

#endif
