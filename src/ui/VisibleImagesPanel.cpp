/*
 * UI Widget to display back projected 3D points on 2D images 
 */


#include <QtWidgets>

#include <iostream>

#include "VisibleImagesPanel.h"
#include "ImageTileWidget.h"
#include "core/Camera.h"
#include "core/datastructs/Data.h"
#include <Eigen/Eigen>

using namespace std;
using namespace cv;

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


void VisibleImagesPanel::showMeasurements(vector<Measurement::Ptr> &ms){

	QMap<int, QList<TileMark> > img2tileMarks;

	for(vector<Measurement::Ptr>::iterator it = ms.begin(); it!=ms.end(); ++it){
		TileMark tm;
		tm.imgIdx	= (*it)->frame->imgIdx;
		tm.pt2DIdx 	= (*it)->featureIdx;


		//Point2f &pt	= (*it)->frame->kpts[tm.pt2DIdx].pt;
		Camera::ProjectionStatus s;
		Eigen::Vector2d ppt= Camera::GetInstance().project((*it)->frame, (*it)->landmark->pt, s);
		assert(s == Camera::ProjectionStatus::Success);

		//tm.pt		= QPointF(pt.x, pt.y);
		tm.pt		= QPointF(ppt[0], ppt[1]);
		tm.selected	= false;
		if(img2tileMarks.find(tm.imgIdx) == img2tileMarks.end()){
			img2tileMarks[tm.imgIdx] = QList<TileMark>();
		}
		img2tileMarks[tm.imgIdx].push_back(tm);
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

//show projections of all map points to all frames, including non-keyframes
void VisibleImagesPanel::showProjections(){
	Data &data = Data::GetInstance();
	Camera &camera = Camera::GetInstance();
	const vector<LandMark::Ptr> &lms = data.getLandMarks();
	const vector<Frame::Ptr> &frames = data.getFrames();
	const vector<Frame::Ptr> &nonKeyFrames = data.nonKeyFrames;

	int kfCnt = frames.size();
	int nonkfCnt = nonKeyFrames.size();
	int totalCnt = kfCnt+nonkfCnt;
	QList<QList<TileMark> >	marks;
	QList<QImage> images;
	for(int i =0; i<totalCnt; i++){
		const Frame::Ptr &frame = (i<kfCnt) ? frames[i] : nonKeyFrames [i-kfCnt];
		QList<TileMark> tms;
		for(vector<LandMark::Ptr>::const_iterator jt = lms.begin(); jt != lms.end(); ++jt){
			Camera::ProjectionStatus s;
			Eigen::Vector2d ppt= Camera::GetInstance().project(frame, (*jt)->pt, s);
			if(s == Camera::ProjectionStatus::Success){
				TileMark tm;
				tm.imgIdx	= frame->imgIdx;
				tm.pt2DIdx 	= 0;
				tm.pt		= QPointF(ppt[0], ppt[1]);
				tm.selected	= false;
				tms.push_back(tm);
			}
		}
		if(!tms.empty()){
			marks.push_back(tms);
			QImage frameImg;
			if(frame->img.empty()){
				//try load from path
				std::string imgPath = frame->imgRoot+"/"+frame->imgName;
				frameImg = QImage(QString::fromStdString(imgPath));
				if(frameImg.isNull()){
					//failed to load, create black image
					cout<<"empty: "<<frame->imgIdx<<endl;
					frameImg = QImage(camera.w, camera.h, QImage::Format_RGB888);
					frameImg.fill(0);
				}
			}else{
				
				if(frame->img.channels()==3){
					cout<<"non empty rgb image: "<<frame->imgIdx<<endl;
					frameImg = QImage((uchar*) frame->img.data, frame->img.cols, frame->img.rows, frame->img.step, QImage::Format_RGB888);
				}else if(frame->img.channels()==1){
					cout<<"non empty gray scale image: "<<frame->imgIdx<<endl;
					cv::Mat tmpImg;
					cv::cvtColor(frame->img,tmpImg,cv::COLOR_GRAY2RGB);
					frameImg = QImage((uchar*) tmpImg.data, tmpImg.cols, tmpImg.rows, tmpImg.step, QImage::Format_RGB888).copy();
					
				}
				
			}
			images.push_back(frameImg);
		}
	}

	imageTileView->setImages(images);
	imageTileView->setMarks(marks);

}

//slot functions

void VisibleImagesPanel::deleteMeasures(){
	Data &data = Data::GetInstance();
	QList<TileMark> selectedMarks;
	imageTileView->getSelectedMarks(selectedMarks);
	QList<QPair<int,int> > img2pt2DIdxs;
	for(int i=0; i<selectedMarks.size(); i++){
		Frame::Ptr frame = data.getFrame(selectedMarks[i].imgIdx);
		assert(frame);
		Measurement::Ptr m = data.getMeasurement(frame, selectedMarks[i].pt2DIdx);
		assert(m);
		data.deleteMeasurement(m);
	}
	emit measuresDeleted();
}
