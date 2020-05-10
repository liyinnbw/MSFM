/*
 * Base widget for displaying zoomable images
 */

#include <QtGui>
#include <QtGlobal>

#include "ImageWidget.h"
#include "core/Utils.h"
#include <QDebug>
#include <iostream>
#include <assert.h>
using namespace std;

// Constructor
ImageWidget:: ImageWidget(QWidget *parent)
: QWidget(parent)
,minZoom(0.1)
,zoom(1.0)
,markToHighlight(-1)
,showSelectWin(false)
,selectWinStart(QPointF(0,0))
,selectWinEnd(QPointF(0,0))
,defaultSize(400,400)
{
	setFocusPolicy(Qt::StrongFocus);
	resize(defaultSize);	//initial widget size
}

void ImageWidget:: reset(){
	//imagePath = "";
	marks.clear();
	lines.clear();
	mask.clear();
	markToHighlight = -1;
	showSelectWin 	= false;
	selectWinStart 	= QPointF(0,0);
	selectWinEnd	= QPointF(0,0);


	setZoomFactor(1.0);
  	resize(parentWidget()->size());
  	image 	= QImage();
  	pixmap 	= QPixmap();
  	update();
}
QSize ImageWidget:: sizeHint() const{
	return this->size();
}

//read from file
void ImageWidget:: setImage(const QString &fileName){
	reset();
	//imagePath = fileName.toUtf8().constData();
	image = QImage(fileName);
	pixmap.convertFromImage(image, Qt::AutoColor);
	fitPixmapToWidgetSize();
	update();
	emit imageLoaded(image);
}
//copy from input QImage
void ImageWidget:: setImage(const QImage &img){
	reset();
	image = img;
	pixmap.convertFromImage(image, Qt::AutoColor);
	fitPixmapToWidgetSize();
	update();
	emit imageLoaded(image);
}

//fit pixelmap to current widget size
void ImageWidget::fitPixmapToWidgetSize(){
	if(pixmap.isNull() || image.isNull()) return;
	QSize widgetSize = sizeHint();
	float WI = (float)image.width();
	float HI = (float)image.height();
	float WR = (float)widgetSize.width()/WI;
	//if(WR>1) WR = 1.0;
	float HR = (float)widgetSize.height()/HI;
	//if(HR>1) HR = 1.0;

	if(WR>HR){
		setZoomFactor(HR);
		pixmap.scaledToHeight(widgetSize.height());
	}else{
		setZoomFactor(WR);
		pixmap.scaledToWidth(widgetSize.height());
	}
}

//setter
void ImageWidget:: setZoomFactor(float newZoom){
	if(newZoom<minZoom)
		zoom=minZoom;
	else
		zoom=newZoom;
}
//setter
void ImageWidget::setMarks(const QList<QPointF> &_marks,  const QList<bool> &_mask){
	if(!_mask.empty()){
		assert(_marks.size() == _mask.size());
	}
	mask=_mask;
	marks = _marks;
	update();

	emit pointsMarked(marks);

}

void ImageWidget::setMask(QList<bool> m){
	showSelectWin = false;
	mask = m;
	update();
}

void ImageWidget::setLines(const QList<QPointF> &linePts){
	lines = linePts;
	update();
}

void ImageWidget::setRect(const QRect &rect){
	selectWinStart 	= rect.topLeft();
	selectWinEnd	= rect.bottomRight();
	showSelectWin 	= true;
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
		if(mask.empty() || mask[i]){
			QColor c(Utils::getRandomInt(0,255), Utils::getRandomInt(0,255), Utils::getRandomInt(0,255));
			QPen pen(c, 2, Qt::SolidLine);
			painter.setPen(pen);
			QPointF mark = marks[i];
			//painter.drawPoint(mark.x()*zoom, mark.y()*zoom);
			painter.drawEllipse(mark*zoom, 2,2);
			//painter.drawText(mark*zoom, QString::number(i));
		}
	}
	if(markToHighlight>=0){
		if(markToHighlight>=marks.size()){
			markToHighlight = -1;
		}else{
			assert(mask.empty() || mask[markToHighlight]);
			QPen pen(Qt::yellow, 3, Qt::SolidLine);
			painter.setPen(pen);
			QPointF mark = marks[markToHighlight];
			//painter.drawPoint(mark.x()*zoom, mark.y()*zoom);
			painter.drawEllipse(mark*zoom, 3,3);
		}
	}
}
void ImageWidget:: drawLines(){
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);
	for(int i=0; i<lines.size(); i+=2){
		QColor c(Utils::getRandomInt(0,255), Utils::getRandomInt(0,255), Utils::getRandomInt(0,255));
		QPen pen(c, 2, Qt::SolidLine);
		painter.setPen(pen);
		painter.drawLine(lines[i]*zoomFactor(),lines[i+1]*zoomFactor());
	}

}
void ImageWidget:: drawSelectionWindow(){
	QPainter painter(this);
	QColor color(255,0,0);
	QPen pen(color, 3, Qt::SolidLine);
	painter.setPen(pen);
	QRect selectionRect = pointsToRect(selectWinStart*zoom,selectWinEnd*zoom);
	painter.drawRect(selectionRect);
	emit selectionWindowChanged(selectionRect);
}

void ImageWidget:: paintEvent(QPaintEvent *event){
	QPainter painter(this);
	painter.drawPixmap(0,0,image.width()*zoom,image.height()*zoom,pixmap);
	drawMarks();
	drawLines();
	if(showSelectWin){
		drawSelectionWindow();
	}
}
void ImageWidget:: wheelEvent(QWheelEvent *event){

	//obtain the mouse wheel's step size
	float step = event->delta()/1200.0;
	setZoomFactor(zoom+step);
	resize(image.width()*zoom,image.height()*zoom);	//fit widget size to
	update();
}

//update event is handled in setSelected called by upper-level widget
void ImageWidget::mouseMoveEvent(QMouseEvent *event){
	QPointF p = event->localPos()/zoom;
	selectWinEnd  = p;
	float 	minDist;
	int 	minIdx;
	distToNearestMark(p, minDist, minIdx);
	if(minDist*zoom<5){
		emit markSelected(minIdx);
	}else{
		emit markSelected(-1);
	}
	update();
}
void ImageWidget::mousePressEvent(QMouseEvent *event){
	selectWinStart 	= event->localPos()/zoom;
	selectWinEnd 	= selectWinStart;
	showSelectWin = true;
	update();
}
void ImageWidget::mouseReleaseEvent(QMouseEvent *event){
	selectWinEnd  = event->localPos()/zoom;
	showSelectWin = false;
	update();
}
void ImageWidget::mouseDoubleClickEvent(QMouseEvent *event){
	QPointF p = event->localPos()/zoom;
	QList<QPointF> _marks;
	//QList<bool> _mask;
	_marks.push_back(p);
	//_mask.push_back(true);
	setMarks(_marks);//, _mask);
	emit doubleClicked(p);
	update();
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
		if(!mask.empty() && !mask[i]){
			continue;
		}
		QPointF diff = pos - marks[i];
		float dist 	= diff.manhattanLength();	//manhattanlength is more efficient approx of true dist
		if (dist<minDist){
			minIdx 	= i;
			minDist = dist;
		}
	}
}
QRect  ImageWidget::pointsToRect(const QPointF &p1, const QPointF &p2){
	QPointF pdiff = p2-p1;
	if(pdiff.x()>=0 && pdiff.y()>=0){
		return QRect(p1.x(),p1.y(),pdiff.x(),pdiff.y());
	}else if(pdiff.x()<0 && pdiff.y()<0){
		return QRect(p2.x(),p2.y(),-pdiff.x(),-pdiff.y());
	}else if(pdiff.x()<0){
		return QRect(p2.x(),p1.y(),-pdiff.x(),pdiff.y());
	}else{
		return QRect(p1.x(),p2.y(),pdiff.x(),-pdiff.y());
	}

}
void ImageWidget::deleteSelected(){
	if(getMarkToHightlight()>=0){
		if(!mask.empty()){
			assert(getMarkToHightlight()<mask.size());
			mask[getMarkToHightlight()] = false;
		}
		setSelected(-1);
		update();
	}
}
bool ImageWidget::isVisible(int idx){
	assert(idx>=0);
	if(!mask.empty()){
		assert(idx<mask.size());
		return mask[idx];
	}else{
		return true;
	}

}
