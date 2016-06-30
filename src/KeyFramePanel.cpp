/*
 * KeyFramePanel.cpp
 *
 *  Created on: May 3, 2016
 *      Author: yoyo
 */
#include <QtGui>
#include <iostream>

#include "KeyFramePanel.h"
#include "ImageWidget.h"

using namespace std;

KeyFramePanel::KeyFramePanel() {
	createWidgets();
	connectWidgets();
	setImagePaths(QString(),QList<QString>());

}

KeyFramePanel::~KeyFramePanel() {
	// TODO Auto-generated destructor stub
}

void KeyFramePanel::createWidgets(){
	imageView 				= new ImageWidget;
	imgList					= new QComboBox;
	lvlList					= new QComboBox;
	lvlList->addItem(tr("All Levels"));
	lvlList->addItem(tr("Level 0: 1/1"));
	lvlList->addItem(tr("Level 1: 1/2"));
	lvlList->addItem(tr("Level 2: 1/4"));
	lvlList->addItem(tr("Level 3: 1/8"));
	QScrollArea *scrollArea	= new QScrollArea;


	imgList->setMaxVisibleItems(10);	//limit list dropdown size to 10
	scrollArea->setWidget(imageView);

	QVBoxLayout *vLayout = new QVBoxLayout;
	vLayout->addWidget(imgList);
	vLayout->addWidget(scrollArea);
	vLayout->addWidget(lvlList);

	this->setLayout(vLayout);
}

void KeyFramePanel::connectWidgets(){
	connect(imgList, SIGNAL(activated(int)), this, SLOT(handleImageSelected(int)));
	connect(lvlList, SIGNAL(activated(int)), this, SLOT(handleImageLevelSelected(int)));
	connect(imageView, SIGNAL(pointsMarked(const QList<QPointF> &)), this, SLOT(handlePointsSelected(const QList<QPointF> &)));
	connect(imgList, SIGNAL(currentIndexChanged(int)), this, SIGNAL(imageChanged(int)));
}

void KeyFramePanel::setImagePaths(const QString &root, const QList<QString> &list){
	imgRoot = root;
	imgList->clear();
	imgList->addItem(tr("Select image"));
	QList<QString> indexedList;
	for(int i=0; i<list.size(); i++){
		QString index = QString("[%1]").arg(i);
		indexedList.push_back(index+list[i]);
	}
	imgList->addItems(indexedList);
}

void KeyFramePanel::handleImageSelected(int idx){
	if(imgList->currentIndex()>0){

		QRegExp rx("(\\[|\\])"); //RegEx for '[' or ']'
		QStringList tokens = imgList->itemText(idx).split(rx);
		imageView->setImage(imgRoot+"/"+tokens[tokens.size()-1]);
		cout<<"KeyFramePanel image:["<<idx-1<<"]"<<(imgList->itemText(idx)).toStdString()<<endl;
		emit doComputeKeyFrame(idx-1);
	}
}
void KeyFramePanel::handleImageLevelSelected(int option){
	int lvl = option-1;
	cout<<"KeyFramePanel image level:"<<lvl<<endl;
	QList<bool> mask;
	QList<QPointF> corners;

	if(lvl == -1){
		int cornerCnt = levelCorners[0].size()+levelCorners[1].size()+levelCorners[2].size()+levelCorners[3].size();
		mask.reserve(cornerCnt);
		corners.reserve(cornerCnt);
		for(int l = 0; l<4; l++){
			for(int i=0; i<levelCorners[l].size(); i++){
				mask.push_back(true);
				corners.push_back(levelCorners[l][i]);
			}
		}
	}else{
		mask.reserve(levelCorners[lvl].size());
		for(int i=0; i<levelCorners[lvl].size(); i++){
			mask.push_back(true);
		}
		corners = levelCorners[lvl];
	}
	imageView->setMarks(corners,mask);
}
void KeyFramePanel::computeKeyFrame(){
	/*
	if(imgList->currentIndex()>0){
		emit doComputeKeyFrame(imgList->currentIndex()-1);
	}else{
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image cannot be empty!");
		return;
	}*/
}
void KeyFramePanel::updateCorners(const QList<QList<QPointF> > &data){
	cout<<"updateCorners called"<<endl;
	levelCorners = data;
	handleImageLevelSelected(lvlList->currentIndex());
}

void KeyFramePanel::handlePointsSelected(const QList<QPointF> & pts){
	int imgIdx = imgList->currentIndex()-1;
	if(imgIdx>0){
		emit imagePointsSelected(imgIdx, pts);
	}
}
