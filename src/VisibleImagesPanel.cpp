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
	connect(imageTileView,SIGNAL(deletePressed()), this, SLOT(deleteMeasures()));
}

void VisibleImagesPanel::setImagePaths(const QString &root, const QList<QString> &list){
	imgRoot = root;
	imgs	= list;
}

void VisibleImagesPanel::setVisibleImagesAndMeasures( 	const vector<vector<pair<int,int> > > 	&pt3D2Measures,
														const vector<vector<QPointF> > 			&pt3D2pt2Ds){

	QMap<int, QList<TileMark> > img2tileMarks;
	for(int i=0; i<pt3D2Measures.size(); i++){
		for(int j=0; j<pt3D2Measures[i].size(); j++){
			TileMark tm;
			tm.imgIdx	= pt3D2Measures[i][j].first;
			tm.pt2DIdx 	= pt3D2Measures[i][j].second;
			tm.pt		= pt3D2pt2Ds[i][j];
			tm.selected	= false;
			if(img2tileMarks.find(tm.imgIdx) == img2tileMarks.end()){
				img2tileMarks[tm.imgIdx] = QList<TileMark>();
			}
			img2tileMarks[tm.imgIdx].push_back(tm);
		}
	}

	QList<QString> 			visibleImgs;
	QList<QList<TileMark> >	marks;
	for(QMap<int, QList<TileMark> >::const_iterator it = img2tileMarks.begin(); it!=img2tileMarks.end(); ++it){
		visibleImgs.push_back(imgs[it.key()]);
		marks.push_back(it.value());
	}
	imageTileView->setImages(imgRoot, visibleImgs);
	imageTileView->setMarks(marks);
}
void VisibleImagesPanel::deleteMeasures(){
	QList<TileMark> selectedMarks;
	imageTileView->getSelectedMarks(selectedMarks);
	QList<QPair<int,int> > img2pt2DIdxs;
	for(int i=0; i<selectedMarks.size(); i++){
		img2pt2DIdxs.push_back(qMakePair(selectedMarks[i].imgIdx,selectedMarks[i].pt2DIdx));
	}
	emit deleteSelectedMeasures(img2pt2DIdxs);
}
