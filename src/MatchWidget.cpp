/*
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#include <QtGui>
#include <QtGlobal>
#include <QDebug>
#include <assert.h>

#include "MatchWidget.h"

using namespace std;
// Constructor
MatchWidget:: MatchWidget(QWidget *parent)
				 : ImageWidget(parent)
{
	setMinimumZoom(0.1);
	setFocusPolicy(Qt::StrongFocus);
	reset();
}

void MatchWidget::setMarks(const QList<QPointF> &_marks1,  const QList<QPointF> &_marks2, const QList<bool> &_mask){
	assert(_marks1.size() == _mask.size() && _marks2.size() == _mask.size());
	setMask(_mask);
	marks1	=_marks1;
	marks2	=_marks2;

	update();
}

void MatchWidget::drawMarks(){

	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);
	for(int i=0; i<marks1.size(); i++){
		if(isVisible(i)){
			QColor c(GetRandomNumber(0,255), GetRandomNumber(0,255), GetRandomNumber(0,255));
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
	}
}

void MatchWidget::setImage1(const QImage & img){
	image1 = img;
	combineImages();
}
void MatchWidget::setImage2(const QImage & img){
	image2 = img;
	combineImages();
}
void MatchWidget::combineImages(){
	QImage result(image1.width() + image2.width() + GAP_BETWEEN_IMAGES, image1.height() ,QImage::Format_RGB32);
	QPainter painter;
	painter.begin(&result);
	painter.drawImage(0, 0, image1);
	painter.drawImage(image1.width()+GAP_BETWEEN_IMAGES, 0, image2);
	painter.end();
	setImage(result);

}

void MatchWidget::mouseMoveEvent(QMouseEvent *event){
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

void  MatchWidget::distToNearestMark(const QPointF &pos, float &minDist, int &minIdx){
	minDist = 1000.0f;
	minIdx 	= -1;
	for(int i=0; i<marks1.size(); i++){
		if(!isVisible(i)){
			continue;
		}
		QPointF diff = pos - marks1[i]*zoomFactor();
		float dist 	= diff.manhattanLength();	//manhattanlength is more efficient approx of true dist
		if (dist<minDist){
			minIdx 	= i;
			minDist = dist;
		}
	}
	for(int i=0; i<marks2.size(); i++){
		if(!isVisible(i)){
			continue;
		}
		QPointF diff = pos - (marks2[i]+QPointF(image1.width()+GAP_BETWEEN_IMAGES,0))*zoomFactor();
		float dist 	= diff.manhattanLength();	//manhattanlength is more efficient approx of true dist
		if (dist<minDist){
			minIdx 	= i;
			minDist = dist;
		}
	}

}
