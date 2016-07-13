/*
 * KeyFramePanel.cpp
 *
 *  Created on: May 3, 2016
 *      Author: yoyo
 */
#include <QtGui>
#include <iostream>

#include "KeyFramePanel.h"
//#include "ImageWidget.h"
#include "KeyFrameWidget.h"

using namespace std;

KeyFramePanel::KeyFramePanel() {
	createWidgets();
	connectWidgets();
	setImagePaths(QString(),QList<QString>());
	currentImgIdx = -1;
	levelCorners.clear();

}

KeyFramePanel::~KeyFramePanel() {
	// TODO Auto-generated destructor stub
}

void KeyFramePanel::createWidgets(){
	imageView 				= new KeyFrameWidget;
	imgList					= new QComboBox;
	lvlList					= new QComboBox;
	computeKeyFrameButton 	= new QPushButton(tr("Compute KeyFrame"));
	lvlList->addItem(tr("All Levels"));
	lvlList->addItem(tr("Level 0: 1/1"));
	lvlList->addItem(tr("Level 1: 1/2"));
	lvlList->addItem(tr("Level 2: 1/4"));
	lvlList->addItem(tr("Level 3: 1/8"));
	QScrollArea *scrollArea	= new QScrollArea;


	imgList->setMaxVisibleItems(10);	//limit list dropdown size to 10
	scrollArea->setWidget(imageView);

	QHBoxLayout *hLayout = new QHBoxLayout;
	hLayout->addWidget(lvlList);
	hLayout->addWidget(computeKeyFrameButton);

	QVBoxLayout *vLayout = new QVBoxLayout;
	vLayout->addWidget(imgList);
	vLayout->addWidget(scrollArea);
	vLayout->addLayout(hLayout);

	this->setLayout(vLayout);
}

void KeyFramePanel::connectWidgets(){
	//connect(imgList, SIGNAL(activated(int)), this, SLOT(handleImageSelected(int)));
	connect(lvlList, SIGNAL(activated(int)), this, SLOT(handleImageLevelSelected(int)));
	connect(imageView, SIGNAL(pointsMarked(const QList<QPointF> &)), this, SLOT(handlePointsSelected(const QList<QPointF> &)));
	connect(imgList, SIGNAL(currentIndexChanged(int)), this, SLOT(handleImageSelected(int)));
	connect(computeKeyFrameButton, SIGNAL(clicked()), this, SLOT(handleComputeKeyFrameClicked()));
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
	currentImgIdx = idx-1;
	if(imgList->currentIndex()>0){

		QRegExp rx("(\\[|\\])"); //RegEx for '[' or ']'
		QStringList tokens = imgList->itemText(idx).split(rx);
		imageView->setImage(imgRoot+"/"+tokens[tokens.size()-1]);
		cout<<"KeyFramePanel image:["<<idx-1<<"]"<<(imgList->itemText(idx)).toStdString()<<endl;
		levelCorners.clear();
		//emit doComputeKeyFrame(idx-1);
		emit imageChanged(idx);
	}
}
void KeyFramePanel::handleComputeKeyFrameClicked(){
	if(currentImgIdx>=0){
		emit doComputeKeyFrame(currentImgIdx);
	}
}
void KeyFramePanel::handleImageLevelSelected(int option){

	int lvl = option-1;
	cout<<"KeyFramePanel image level:"<<lvl<<endl;
	if(levelCorners.empty()) return;

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
	if(imgIdx>=0){
		emit imagePointsSelected(imgIdx, pts);
	}
}
void KeyFramePanel::handleImagesUsed(std::vector<int> &usedImgIdxs){
	for(int i=0; i<imgList->count(); i++){
		imgList->setItemData(i, Qt::black, Qt::TextColorRole);
	}
	for(int i=0; i<usedImgIdxs.size(); i++){
		imgList->setItemData(usedImgIdxs[i]+1, Qt::yellow, Qt::TextColorRole);
	}

}
void KeyFramePanel::handleImagesUsed2(std::vector<int> &usedImgIdxs){
	for(int i=0; i<usedImgIdxs.size(); i++){
		imgList->setItemData(usedImgIdxs[i]+1, Qt::red, Qt::TextColorRole);
	}
}
void KeyFramePanel::drawProjection(const QList<QPointF> &verts){
	QList<bool> mask;
	for(int i = 0; i<verts.size(); i++){
		mask.push_back(true);
	}
	imageView->setProjectionVerts(verts);
	//imageView->setMarks(verts,mask);
}
