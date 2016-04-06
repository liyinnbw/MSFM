/*
 * MatchPanel.cpp
 *
 *  Created on: Apr 3, 2016
 *      Author: yoyo
 */

#include <QtGui>

#include <iostream>

#include "MatchPanel.h"
#include "ImageWidget.h"
#include "MatchWidget.h"

using namespace std;

MatchPanel::MatchPanel(QWidget *parent)
		   : QWidget(parent) {
	createWidgets();
	connectWidgets();
	setImagePaths(QString(),QList<QString>());
}

MatchPanel::~MatchPanel() {
	// TODO Auto-generated destructor stub
}

void MatchPanel::createWidgets(){
	imageView1 				= new ImageWidget;
	imageView2 				= new ImageWidget;
	matchView 				= new MatchWidget;
	imgList1 				= new QComboBox;
	imgList2 				= new QComboBox;
	QScrollArea *scrollArea1= new QScrollArea;
	QScrollArea *scrollArea2= new QScrollArea;
	QScrollArea *scrollArea3= new QScrollArea;

	scrollArea1->setWidget(imageView1);
	scrollArea2->setWidget(imageView2);
	scrollArea3->setWidget(matchView);

	QVBoxLayout *vLayout1 = new QVBoxLayout;
	vLayout1->addWidget(imgList1);
	vLayout1->addWidget(scrollArea1);

	QVBoxLayout *vLayout2 = new QVBoxLayout;
	vLayout2->addWidget(imgList2);
	vLayout2->addWidget(scrollArea2);

	QHBoxLayout *leftTop = new QHBoxLayout;
	leftTop->addLayout(vLayout1);
	leftTop->addLayout(vLayout2);

	QVBoxLayout *vLayout = new QVBoxLayout;
	vLayout->addLayout(leftTop);
	vLayout->addWidget(scrollArea3);

	this->setLayout(vLayout);
}
void MatchPanel::connectWidgets(){
	connect(imgList1, SIGNAL(currentIndexChanged(int)), imgList1, SIGNAL(activated(int)));
	connect(imgList2, SIGNAL(currentIndexChanged(int)), imgList2, SIGNAL(activated(int)));
	connect(imgList1, SIGNAL(activated(int)), this, SLOT(handleFirstImageSelected(int)));
	connect(imgList2, SIGNAL(activated(int)), this, SLOT(handleSecondImageSelected(int)));
	connect(imageView1, SIGNAL(markSelected(const int)), this, SLOT(updateSelected(const int)));
	connect(imageView2, SIGNAL(markSelected(const int)), this, SLOT(updateSelected(const int)));
	connect(matchView, SIGNAL(markSelected(const int)), this, SLOT(updateSelected(const int)));
	connect(imageView1, SIGNAL(deletePressed()), this, SLOT(handleDelete()));
	connect(imageView2, SIGNAL(deletePressed()), this, SLOT(handleDelete()));
	connect(matchView, SIGNAL(deletePressed()), this, SLOT(handleDelete()));
	connect(imageView1, SIGNAL(imageLoaded(const QImage &)), matchView, SLOT(setImage1(const QImage &)));
	connect(imageView2, SIGNAL(imageLoaded(const QImage &)), matchView, SLOT(setImage2(const QImage &)));

}

void MatchPanel::setImagePaths(const QString &root, const QList<QString> &list){
	imgRoot = root;
	imgList1->clear();
	imgList2->clear();
	imgList1->addItem(tr("Select image"));
	imgList2->addItem(tr("Select image"));
	imgList1->addItems(list);
	imgList2->addItems(list);
}

void MatchPanel::handleFirstImageSelected(int idx){
	if(imgList1->currentIndex()>0){
		cout<<"1st image:["<<idx-1<<"]"<<imgList1->itemText(idx).toStdString()<<endl;
		imageView1->setImage(imgRoot+"/"+imgList1->itemText(idx));
	}

}
void MatchPanel::handleSecondImageSelected(int idx){
	if(imgList2->currentIndex()>0){
		cout<<"2nd image:["<<idx-1<<"]"<<imgList2->itemText(idx).toStdString()<<endl;
		imageView2->setImage(imgRoot+"/"+imgList2->itemText(idx));
	}
}
void MatchPanel::updateViews(const QList<QPointF> &pts1, const QList<QPointF> &pts2, const QList<bool> &mask){
	imageView1->setMarks(pts1,mask);
	imageView2->setMarks(pts2,mask);
	matchView ->setMarks(pts1, pts2, mask);
}
void MatchPanel::updateSelected(const int matchIdx){
	imageView1->setSelected(matchIdx);
	imageView2->setSelected(matchIdx);
	matchView->setSelected(matchIdx);
}

void MatchPanel::handleDelete(){
	imageView1->deleteSelected();
	imageView2->deleteSelected();
	matchView ->deleteSelected();
}

void MatchPanel::getSelectedImages(int &idx1, int &idx2){
	idx1 = imgList1->currentIndex();
	idx2 = imgList2->currentIndex();
}

void MatchPanel::getMask(QList<bool> &mask){
	mask = imageView1->getMask();
}
void MatchPanel::setImagePair(const int img1, const int img2){
	imgList1->setCurrentIndex(img1+1);
	imgList2->setCurrentIndex(img2+1);
}
