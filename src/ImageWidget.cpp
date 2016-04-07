/*
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#include <QtGui>
#include <QtGlobal>

#include "ImageWidget.h"
#include <QDebug>
#include <iostream>
#include <assert.h>
using namespace std;

// Constructor
ImageWidget:: ImageWidget(QWidget *parent)
				 : QWidget(parent)
{
	setMinimumZoom(0.1);
	setFocusPolicy(Qt::StrongFocus);
	reset();
}

void ImageWidget:: reset(){
	
	setZoomFactor(1.0);
  	resize(600,400);
	image = QImage(600, 400, QImage::Format_ARGB32);
	pixmap.convertFromImage(image, 0); 
	image.fill(qRgba(0, 0, 0, 0));
	imagePath = "";
	markToHighlight = -1;
	marks.clear();
	
}
QSize ImageWidget:: sizeHint() const{
	return this->size();
}
void ImageWidget:: setImage(const QString &fileName){
	
	imagePath = fileName.toUtf8().constData();
	setImage(QImage(fileName));
}

void ImageWidget:: setImage(const QImage &img){
	reset();
	image = img;
	pixmap.convertFromImage(image, 0); 

	QSize widgetSize = sizeHint();
	float WI = (float)image.width();
	float HI = (float)image.height();
	float WR = (float)widgetSize.width()/WI;
	if(WR>1) WR = 1.0;
	float HR = (float)widgetSize.height()/HI;
	if(HR>1) HR = 1.0;
	  
	if(WR>HR){
		setZoomFactor(HR);
		pixmap.scaledToHeight(widgetSize.height());
	}else{
		setZoomFactor(WR);
		pixmap.scaledToWidth(widgetSize.height());
	}

	//resize the widget to fit the image
	resize(image.width()*zoom,image.height()*zoom);
	update();

	emit imageLoaded(image);
}

//setter
void ImageWidget:: setMinimumZoom(float newZoom){
	minZoom = newZoom;
}
//setter
void ImageWidget:: setZoomFactor(float newZoom){
	if(newZoom<minZoom)
		zoom=minZoom;
	else
		zoom=newZoom;
	resize(image.width()*zoom,image.height()*zoom);
	update();
}
//setter
void ImageWidget::setMarks(const QList<QPointF> &_marks,  const QList<bool> &_mask){
	assert(_marks.size() == _mask.size());
	mask =_mask;
	marks=_marks;
	update();
}
//setter
void ImageWidget::setSelected(const int matchIdx){
	markToHighlight = matchIdx;
	update();
}
void ImageWidget::drawMarks(){
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);
	for(int i=0; i<marks.size(); i++){
		if(mask[i]){
			QColor c(GetRandomNumber(0,255), GetRandomNumber(0,255), GetRandomNumber(0,255));
			QPen pen(c, 2, Qt::SolidLine);
			painter.setPen(pen);
			QPointF mark = marks[i];
			//painter.drawPoint(mark.x()*zoom, mark.y()*zoom);
			painter.drawEllipse(mark*zoom, 2,2);
			//painter.drawText(mark*zoom, QString::number(i));
		}
	}
	if(markToHighlight>=0){
		assert(mask[markToHighlight]);
		QPen pen(Qt::yellow, 3, Qt::SolidLine);
		painter.setPen(pen);
		QPointF mark = marks[markToHighlight];
		//painter.drawPoint(mark.x()*zoom, mark.y()*zoom);
		painter.drawEllipse(mark*zoom, 3,3);
	}
}

void ImageWidget:: paintEvent(QPaintEvent *event){

	QPainter painter(this);
	painter.drawPixmap(0,0,image.width()*zoom,image.height()*zoom,pixmap);
	drawMarks();
}
void ImageWidget:: wheelEvent(QWheelEvent *event){

	//obtaint he mouse wheel's step size
	float step = event->delta()/1200.0;
	setZoomFactor(zoom+step);
	
}

//update event is handled in setSelected called by upper-level widget
void ImageWidget::mouseMoveEvent(QMouseEvent *event){
	QPointF p = event->posF();
	float 	minDist;
	int 	minIdx;
	distToNearestMark(p, minDist, minIdx);
	if(minDist<5){
		emit markSelected(minIdx);
	}else{
		emit markSelected(-1);
	}
}

void ImageWidget::keyPressEvent (QKeyEvent *event){

	switch (event -> key()){
		case Qt::Key_D:
			emit deletePressed();
			break;
		default:
			break;
	}
}
void  ImageWidget::distToNearestMark(const QPointF &pos, float &minDist, int &minIdx){
	minDist = 1000.0f;
	minIdx 	= -1;
	for(int i=0; i<marks.size(); i++){
		if(!mask[i]){
			continue;
		}
		QPointF diff = pos - marks[i]*zoom;
		float dist 	= diff.manhattanLength();	//manhattanlength is more efficient approx of true dist
		if (dist<minDist){
			minIdx 	= i;
			minDist = dist;
		}
	}
}
void ImageWidget::deleteSelected(){
	if(getMarkToHightlight()>=0){
		assert(getMarkToHightlight()<mask.size());
		mask[getMarkToHightlight()] = false;
		setSelected(-1);
		update();
	}
}
bool ImageWidget::isVisible(int idx){
	assert(idx>=0 && idx<mask.size());
	return mask[idx];
}
