/*
 *  Created on: Jun 16, 2016
 *      Author: yoyo
 */

#ifndef IMAGETILEWIDGET_H
#define IMAGETILEWIDGET_H

#include <QMainWindow>
#include <QWidget>
#include <QList>

#include "ImageWidget.h"

class QImage;
class QPointF;
class QPixmap;
class ImageTileWidget: public ImageWidget
{
	Q_OBJECT
	//property, no ";" at the back
	Q_PROPERTY(float minZoom READ minimumZoom WRITE setMinimumZoom)
	Q_PROPERTY(float zoomFactor READ zoomFactor WRITE setZoomFactor)

public slots:
	void setMarks(const QList<QList<QPointF> >&);
	void setImages(const QString &root, const QList<QString> &list);
	

public:
	ImageTileWidget(QWidget *parent = 0);

protected:
	virtual void drawMarks();
	void combineImages();

private:
	QList<QImage> 			images;
	QList<QList<QPointF> > 	marks;
	int						rows;
	int 					cols;
	int						imgW;
	int						imgH;
	const static int 		GAP_BETWEEN_IMAGES = 0;
};

#endif
