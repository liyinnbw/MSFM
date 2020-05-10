/*
 * Widget to display multiple images
 */

#include <QtGui>
#include <QtGlobal>
#include <QDebug>
#include <assert.h>

#include "ImageTileWidget.h"
#include "core/Utils.h"
#include "core/Camera.h"

using namespace std;
// Constructor
ImageTileWidget:: ImageTileWidget(QWidget *parent)
:ImageWidget(parent)
,rows(0)
,cols(0)
,imgW(0)
,imgH(0)
{
	setFocusPolicy(Qt::StrongFocus);
	connect(this, SIGNAL(selectionWindowChanged(const QRect &)), this, SLOT(setSelected(const QRect &)));
}

void ImageTileWidget::setMarks(const QList<QList<TileMark> >& _marks){
	marks = _marks;
	update();
}
void ImageTileWidget::setSelected(const QRect &selWin){
	for(int i=0; i<marks.size(); i++){
		int r = i/cols;
		int c = i%cols;
		int xOffset = c*imgW+GAP_BETWEEN_IMAGES*c;
		int yOffset = r*imgH+GAP_BETWEEN_IMAGES*r;
		for(int j=0; j<marks[i].size(); j++){
			QPointF p = (marks[i][j].pt+QPointF(xOffset,yOffset))*zoomFactor();
			if(selWin.contains((int)p.x(), (int)p.y())){
				marks[i][j].selected = true;
			}else{
				marks[i][j].selected = false;
			}
		}
	}
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
			QColor color;
			if(marks[i][j].selected){
				color = QColor(255,0,0);
			}else{
				color = QColor(0,255,0);
			}
			QPen pen(color, 3, Qt::SolidLine);
			painter.setPen(pen);
			QPointF p = marks[i][j].pt+QPointF(xOffset,yOffset);
			painter.drawEllipse(p*zoomFactor(),1,1);
		}
	}
}
void ImageTileWidget::setImages(const QString &root, const QList<QString> &list){
	images.clear();
	Camera &camera = Camera::GetInstance();
	for(int i=0; i<list.size(); i++){
		QImage frameImg(root+"/"+list[i]);
		if(frameImg.isNull()){
			frameImg = QImage(camera.w, camera.h, QImage::Format_RGB888);
			frameImg.fill(0);
		}
		images.push_back(frameImg);
	}
	combineImages();
}

void ImageTileWidget::setImages(const QList<QImage> &frameImgs){
	images.clear();
	for(int i=0; i<frameImgs.size(); i++){
		images.push_back(frameImgs[i]);
	}
	//images = frameImgs;
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

	QImage result(imgW*cols + GAP_BETWEEN_IMAGES*(cols-1), imgH*rows + GAP_BETWEEN_IMAGES*(rows-1) ,QImage::Format_RGB888);
	result.fill(0);
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
void ImageTileWidget::getSelectedMarks(QList<TileMark> &sMarks){
	sMarks.clear();
	for(int i=0; i<marks.size(); i++){
		for(int j=0; j<marks[i].size(); j++){
			if(marks[i][j].selected){
				sMarks.push_back(marks[i][j]);
			}
		}
	}
}

