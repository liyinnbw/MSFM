/*
 * Interactive widget to display feature points on 2D image
 */

#ifndef MATCHWIDGET_H
#define MATCHWIDGET_H

#include <QMainWindow>
#include <QWidget>
#include <QList>

#include "ImageWidget.h"

class QImage;
class QPointF;
class QPixmap;
class MatchWidget: public ImageWidget
{
	Q_OBJECT
	//property, no ";" at the back
	//Q_PROPERTY(float zoomFactor READ zoomFactor WRITE setZoomFactor)

public slots:
	void setMarks(const QList<QPointF> &, const QList<QPointF> &, const QList<bool> &);
	void setImage1(const QImage &);
	void setImage2(const QImage &);
	

public:
	MatchWidget(QWidget *parent = 0);

protected:
	virtual void drawMarks();
	virtual void mouseMoveEvent(QMouseEvent *event);
	virtual void  distToNearestMark(const QPointF &pos, float &minDist, int &minIdx);
	void combineImages();

private:
	QImage 				image1;
	QImage				image2;
	QList<QPointF> 		marks1;
	QList<QPointF> 		marks2;
	const static int 	GAP_BETWEEN_IMAGES = 0;
};

#endif
