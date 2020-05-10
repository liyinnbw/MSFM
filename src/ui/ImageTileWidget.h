/*
 * Widget to display multiple images
 */

#ifndef IMAGETILEWIDGET_H
#define IMAGETILEWIDGET_H

#include <QMainWindow>
#include <QWidget>
#include <QList>

#include "ImageWidget.h"


struct TileMark{
	int imgIdx;
	int pt2DIdx;
	QPointF pt;
	bool selected;
};

class QImage;
class QPointF;
class QPixmap;
class ImageTileWidget: public ImageWidget
{
	Q_OBJECT
	//property, no ";" at the back
//	Q_PROPERTY(float zoomFactor READ zoomFactor WRITE setZoomFactor)

public slots:
	void setMarks(const QList<QList<TileMark> >&);
	void setImages(const QString &root, const QList<QString> &list);
	void setImages(const QList<QImage> &);
	void setSelected(const QRect &);
	

public:
	ImageTileWidget(QWidget *parent = 0);
	void getSelectedMarks(QList<TileMark> &sMarks);

protected:
	virtual void drawMarks();
	void combineImages();

private:
	QList<QImage> 			images;
	QList<QList<TileMark> > marks;
	int						rows;
	int 					cols;
	int						imgW;
	int						imgH;
	const static int 		GAP_BETWEEN_IMAGES = 0;
};

#endif
