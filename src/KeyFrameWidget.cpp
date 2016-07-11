/*
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#include <QtGui>
#include <QtGlobal>
#include <QDebug>
#include <assert.h>

#include "KeyFrameWidget.h"
#include "core/Utils.h"

using namespace std;
// Constructor
KeyFrameWidget:: KeyFrameWidget(QWidget *parent)
				 : ImageWidget(parent)
{
	setMinimumZoom(0.1);
	setFocusPolicy(Qt::StrongFocus);
	reset();
}

void KeyFrameWidget::setProjectionVerts(const QList<QPointF> &_verts){
	verts = _verts;
	update();
}

void KeyFrameWidget::drawMarks(){
	ImageWidget::drawMarks();

	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing);
	QColor c(0,255,0);	//green
	QPen pen(c, 1, Qt::SolidLine);
	painter.setPen(pen);
	assert(verts.size()%3==0);
	for(int i=0; i<verts.size(); i+=3){
		QPointF v0 = verts[i];
		QPointF v1 = verts[i+1];
		QPointF v2 = verts[i+2];
		painter.drawLine(v0*zoomFactor(),v1*zoomFactor());
		painter.drawLine(v1*zoomFactor(),v2*zoomFactor());
		painter.drawLine(v2*zoomFactor(),v0*zoomFactor());
	}


}
