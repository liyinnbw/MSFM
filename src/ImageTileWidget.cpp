/*
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#include <QtGui>
#include <QtGlobal>
#include <QDebug>
#include <assert.h>

#include "ImageTileWidget.h"
#include "core/Utils.h"

using namespace std;
// Constructor
ImageTileWidget:: ImageTileWidget(QWidget *parent)
				 : ImageWidget(parent)
{
	setMinimumZoom(0.1);
	setFocusPolicy(Qt::StrongFocus);
	reset();
}

void ImageTileWidget::setMarks(const QList<QList<QPointF> >& _marks){
	marks = _marks;
	update();
}

void ImageTileWidget::drawMarks(){
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);
	for(int i=0; i<marks.size(); i++){
		int r = i/cols;
		int c = i%cols;
		int xOffset = c*imgW+GAP_BETWEEN_IMAGES*c;
		int yOffset = r*imgH+GAP_BETWEEN_IMAGES*r;
		for(int j=0; j<marks[i].size(); j++){
			QColor color(0,255,0);
			QPen pen(color, 3, Qt::SolidLine);
			painter.setPen(pen);
			QPointF p = marks[i][j]+QPointF(xOffset,yOffset);
			painter.drawEllipse(p*zoomFactor(),1,1);
		}
	}
	/*for(int i=0; i<marks1.size(); i++){
		if(isVisible(i)){
			QColor c(Utils::getRandomInt(0,255), Utils::getRandomInt(0,255), Utils::getRandomInt(0,255));
			QPen pen(c, 1, Qt::SolidLine);
			painter.setPen(pen);
			QPointF mark1 = marks1[i];
			QPointF mark2 = marks2[i]+QPointF(image1.width()+GAP_BETWEEN_IMAGES,0);
			//painter.drawPoint(mark1.x()*zoomFactor(), mark1.y()*zoomFactor());
			//painter.drawPoint(mark2.x()*zoomFactor(), mark2.y()*zoomFactor());
			painter.drawLine(mark1*zoomFactor(),mark2*zoomFactor());
			painter.drawEllipse(mark1*zoomFactor(),1,1);
			painter.drawEllipse(mark2*zoomFactor(),1,1);
			//painter.drawText(mark*zoom, QString::number(i));
		}
	}
	if(getMarkToHightlight()>=0){
		assert(isVisible(getMarkToHightlight()));
		QPen pen(Qt::yellow, 3, Qt::SolidLine);
		painter.setPen(pen);
		QPointF mark1 = marks1[getMarkToHightlight()];
		QPointF mark2 = marks2[getMarkToHightlight()]+QPointF(image1.width()+GAP_BETWEEN_IMAGES,0);
		painter.drawLine(mark1*zoomFactor(),mark2*zoomFactor());
		painter.drawEllipse(mark1*zoomFactor(),3,3);
		painter.drawEllipse(mark2*zoomFactor(),3,3);
	}*/
}

void ImageTileWidget::setImages(const QString &root, const QList<QString> &list){
	images.clear();
	for(int i=0; i<list.size(); i++){
		images.push_back(QImage(root+"/"+list[i]));
	}
	combineImages();
}

void ImageTileWidget::combineImages(){
	if (images.empty()){
		rows = 0;
		cols = 0;
		imgW = 0;
		imgH = 0;
	}else{
		cols = ceil(sqrt(images.size()));
		rows = ceil(images.size()*1.0/cols);
		imgW = images[0].width();
		imgH = images[0].height();
	}

	QImage result(imgW*cols + GAP_BETWEEN_IMAGES*(cols-1), imgH*rows + GAP_BETWEEN_IMAGES*(rows-1) ,QImage::Format_RGB32);
	result.fill(QColor(0,0,0));
	QPainter painter;
	painter.begin(&result);
	for(int i=0; i<images.size(); i++){
		int r = i/cols;
		int c = i%cols;
		int x = c*imgW+GAP_BETWEEN_IMAGES*c;
		int y = r*imgH+GAP_BETWEEN_IMAGES*r;
		painter.drawImage(x, y, images[i]);
	}
	painter.end();
	setImage(result);

}


