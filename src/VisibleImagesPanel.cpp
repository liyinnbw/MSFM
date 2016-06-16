/*
 *  Created on: Jun 16, 2016
 *      Author: yoyo
 */

#include <QtGui>

#include <iostream>

#include "VisibleImagesPanel.h"
#include "ImageTileWidget.h"

using namespace std;

VisibleImagesPanel::VisibleImagesPanel(QWidget *parent)
		   : QWidget(parent) {
	createWidgets();
	connectWidgets();
	setImagePaths(QString(),QList<QString>());
}

VisibleImagesPanel::~VisibleImagesPanel() {
	// TODO Auto-generated destructor stub
}

void VisibleImagesPanel::createWidgets(){
	imageTileView 			= new ImageTileWidget;
	QScrollArea *scrollArea = new QScrollArea;
	scrollArea->setWidget(imageTileView);
	QVBoxLayout *vLayout = new QVBoxLayout;
	vLayout->addWidget(scrollArea);
	this->setLayout(vLayout);
}
void VisibleImagesPanel::connectWidgets(){

}

void VisibleImagesPanel::setImagePaths(const QString &root, const QList<QString> &list){
	imgRoot = root;
	imgs	= list;
}

void VisibleImagesPanel::setVisibleImagesAndMeasures( QMap<int, QList<QPointF> > img2pt2Ds){
	QList<QString> visibleImgs;
	QList<QList<QPointF> >marks;
	for(QMap<int, QList<QPointF> >::iterator it = img2pt2Ds.begin(); it!=img2pt2Ds.end(); ++it){
		visibleImgs.push_back(imgs[it.key()]);
		marks.push_back(it.value());
	}
	imageTileView->setImages(imgRoot, visibleImgs);
	imageTileView->setMarks(marks);
}
