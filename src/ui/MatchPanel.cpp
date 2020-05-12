/*
 * Interactive widget display feature matches between two 2D images
 */

#include <QtWidgets>
#include <iostream>
#include <QColor>

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

	imgList1->setMaxVisibleItems(10);	//limit list dropdown size to 10
	imgList2->setMaxVisibleItems(10);	//limit list dropdown size to 10

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
	connect(imgList1, SIGNAL(currentIndexChanged(int)), this, SLOT(handleFirstImageSelected(int)));
	connect(imgList2, SIGNAL(currentIndexChanged(int)), this, SLOT(handleSecondImageSelected(int)));
	//connect(imgList1, SIGNAL(currentIndexChanged(int)), imgList1, SIGNAL(activated(int))); //side effect, forces handleFirstImageSelected to be called again
	//connect(imgList2, SIGNAL(currentIndexChanged(int)), imgList2, SIGNAL(activated(int))); //side effect, forces handleSecondImageSelected to be called again
	//connect(imgList1, SIGNAL(activated(int)), this, SLOT(handleFirstImageSelected(int)));
	//connect(imgList2, SIGNAL(activated(int)), this, SLOT(handleSecondImageSelected(int)));
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
	QList<QString> indexedList;
	for(int i=0; i<list.size(); i++){
		QString index = QString("[%1]").arg(i);
		indexedList.push_back(index+list[i]);
	}
	imgList1->addItems(indexedList);
	imgList2->addItems(indexedList);
}

void MatchPanel::handleFirstImageSelected(int idx){
	if(imgList1->currentIndex()>0){
		cout<<"1st image:["<<idx-1<<"]"<<imgList1->itemText(idx).toStdString()<<endl;
		QRegExp rx("(\\[|\\])"); //RegEx for '[' or ']'
		QStringList tokens = imgList1->itemText(idx).split(rx);
		imageView1->setImage(imgRoot+"/"+tokens[tokens.size()-1]);
		//emit firstImageSelected(idx-1);
		emit imageChanged(idx-1,imgList2->currentIndex()-1);
	}

}
void MatchPanel::handleSecondImageSelected(int idx){
	if(imgList2->currentIndex()>0){
		cout<<"2nd image:["<<idx-1<<"]"<<imgList2->itemText(idx).toStdString()<<endl;
		QRegExp rx("(\\[|\\])"); //RegEx for '[' or ']'
		QStringList tokens = imgList2->itemText(idx).split(rx);
		imageView2->setImage(imgRoot+"/"+tokens[tokens.size()-1]);
		emit imageChanged(imgList1->currentIndex()-1,idx-1);
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

void MatchPanel::handleImagesUsed(std::vector<int> &usedImgIdxs){
	for(int i=0; i<imgList1->count(); i++){
		imgList1->setItemData(i, QColor(0, 0, 0), Qt::TextColorRole);
	}
	for(int i=0; i<imgList2->count(); i++){
		imgList2->setItemData(i, QColor(0, 0, 0), Qt::TextColorRole);
	}
	for(int i=0; i<usedImgIdxs.size(); i++){
		imgList1->setItemData(usedImgIdxs[i]+1, QColor(0, 0, 255), Qt::TextColorRole);
		imgList2->setItemData(usedImgIdxs[i]+1, QColor(0, 0, 255), Qt::TextColorRole);
	}

}

void MatchPanel::getSelectedImages(int &idx1, int &idx2){
	idx1 = imgList1->currentIndex()-1;
	idx2 = imgList2->currentIndex()-1;
}

void MatchPanel::getMask(QList<bool> &mask){
	mask = imageView1->getMask();
}
void MatchPanel::setImagePair(const int img1, const int img2){
	imgList1->setCurrentIndex(img1+1);
	imgList2->setCurrentIndex(img2+1);
}
