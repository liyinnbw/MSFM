/*
 * Interactive Widget to draw epipolar line between two images
 */

#include <QtWidgets>
#include <QColor>
#include <iostream>

#include "EpipolarPanel.h"

#include "MatchPanel.h"
#include "ImageWidget.h"
#include "core/datastructs/Data.h"
#include "opencv2/opencv.hpp"
#include "core/Camera.h"
#include "Eigen/Eigen"

using namespace std;
using namespace Eigen;
using namespace cv;

EpipolarPanel::EpipolarPanel(QWidget *parent)
:QWidget(parent)
{
	createWidgets();
	connectWidgets();
	setImagePaths(QString(),QList<QString>());
}

EpipolarPanel::~EpipolarPanel() {
	// TODO Auto-generated destructor stub
}

void EpipolarPanel::createWidgets(){
	imageView1 				= new ImageWidget;
	imageView2 				= new ImageWidget;
	imgList1 				= new QComboBox;
	imgList2 				= new QComboBox;
	QScrollArea *scrollArea1= new QScrollArea;
	QScrollArea *scrollArea2= new QScrollArea;

	imgList1->setMaxVisibleItems(10);	//limit list dropdown size to 10
	imgList2->setMaxVisibleItems(10);	//limit list dropdown size to 10

	scrollArea1->setWidget(imageView1);
	scrollArea2->setWidget(imageView2);

	QVBoxLayout *vLayout1 = new QVBoxLayout;
	vLayout1->addWidget(imgList1);
	vLayout1->addWidget(scrollArea1);

	QVBoxLayout *vLayout2 = new QVBoxLayout;
	vLayout2->addWidget(imgList2);
	vLayout2->addWidget(scrollArea2);


	QVBoxLayout *vLayout = new QVBoxLayout;
	vLayout->addLayout(vLayout1);
	vLayout->addLayout(vLayout2);

	this->setLayout(vLayout);
}
void EpipolarPanel::connectWidgets(){
	connect(imgList1, SIGNAL(currentIndexChanged(int)), this, SLOT(handleFirstImageSelected(int)));
	connect(imgList2, SIGNAL(currentIndexChanged(int)), this, SLOT(handleSecondImageSelected(int)));
	connect(imageView1, SIGNAL(pointsMarked(const QList<QPointF> &)), this, SLOT(handleImageView1PointMarked(const QList<QPointF> &)));
	connect(imageView1, SIGNAL(markSelected(const int)), this, SLOT(handleImageView1PointSelected(const int)));
}

void EpipolarPanel::setImagePaths(const QString &root, const QList<QString> &list){
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

void EpipolarPanel::handleFirstImageSelected(int idx){
	if(imgList1->currentIndex()>0){
		cout<<"1st image:["<<idx-1<<"]"<<imgList1->itemText(idx).toStdString()<<endl;
		QRegExp rx("(\\[|\\])"); //RegEx for '[' or ']'
		QStringList tokens = imgList1->itemText(idx).split(rx);
		imageView1->setImage(imgRoot+"/"+tokens[tokens.size()-1]);

		Data &data = Data::GetInstance();
		const Frame::Ptr &frame = data.getFrame(idx-1);
		if(frame){
			QList<QPointF> marks;
			for(vector<KeyPoint>::const_iterator it = frame->kpts.begin(); it != frame->kpts.end(); ++it){
				marks.push_back(QPointF(it->pt.x, it->pt.y));
			}
			imageView1->setMarks(marks);
		}
		//emit firstImageSelected(idx-1);
		emit imageChanged(idx-1,imgList2->currentIndex()-1);
	}

}
void EpipolarPanel::handleSecondImageSelected(int idx){
	if(imgList2->currentIndex()>0){
		cout<<"2nd image:["<<idx-1<<"]"<<imgList2->itemText(idx).toStdString()<<endl;
		QRegExp rx("(\\[|\\])"); //RegEx for '[' or ']'
		QStringList tokens = imgList2->itemText(idx).split(rx);
		imageView2->setImage(imgRoot+"/"+tokens[tokens.size()-1]);

		Data &data = Data::GetInstance();
		const Frame::Ptr &frame = data.getFrame(idx-1);
		if(frame){
			QList<QPointF> marks;
			for(vector<KeyPoint>::const_iterator it = frame->kpts.begin(); it != frame->kpts.end(); ++it){
				marks.push_back(QPointF(it->pt.x, it->pt.y));
			}
			imageView2->setMarks(marks);
		}

		emit imageChanged(imgList1->currentIndex()-1,idx-1);
	}
}
void EpipolarPanel::handleImageView1PointMarked(const QList<QPointF> &pts){
	int imgIdx1 = imgList1->currentIndex()-1;
	int imgIdx2 = imgList2->currentIndex()-1;
	Data &data = Data::GetInstance();
	Frame::Ptr frame1 = data.getFrame(imgIdx1);
	Frame::Ptr frame2 = data.getFrame(imgIdx2);
	if(!frame1 || !frame2) return;

	//use cloud depth range viewed by frame1 to determine epipolar search length
	//extend the search range 1.5 times and in front of frame 1 camera
	const vector<LandMark::Ptr> &lms = data.getLandMarks();
	double minZ, maxZ;
	bool zInitialized = false;
	for(vector<LandMark::Ptr>::const_iterator it = lms.begin(); it!=lms.end(); ++it){
		Vector3d pt = frame1->rotation*((*it)->pt - frame1->position);
		if(!zInitialized){
			minZ = maxZ = pt[2];
			zInitialized = true;
		}else{
			if(pt[2]<minZ){
				minZ = pt[2];
			}else if(pt[2]>maxZ){
				maxZ = pt[2];
			}
		}
	}
	double midZ = (maxZ+minZ)/2;
	minZ		-= midZ*1.5;
	maxZ		+= midZ*1.5;
	double f	= Camera::GetInstance().getCamFocal();
	int	imgW	= Camera::GetInstance().w;
	int	imgH	= Camera::GetInstance().h;

	if (minZ<1) minZ=1;
	if (maxZ<minZ) maxZ	= minZ*1.5;
	int EPIPOLAR_SEARCH_PIXEL_THRESH = 8;

	KeyPoint kpt(pts[0].x(), pts[0].y(), 0);
	Vector3d vec1 	= Camera::GetInstance().getBearingVector(kpt);
	//find 3D line in frame1's coordinate
	Vector3d lStart1= vec1*minZ;
	Vector3d lEnd1	= vec1*maxZ;
	//find 3D line in world coordinate
	Vector3d lStart	= frame1->rotation.conjugate()*lStart1+frame1->position;
	Vector3d lEnd	= frame1->rotation.conjugate()*lEnd1+frame1->position;
	//find 3D line in frame2's coordinate.
	Vector3d lStart2= frame2->rotation*(lStart-frame2->position);
	Vector3d lEnd2	= frame2->rotation*(lEnd-frame2->position);
	//some precalculations to prevent duplicated calculation when calculating distance to epipolar line
	//for a 2D line defined by two points p1, p2, the distance of another point p0 on the plane to that line is:
	//abs((p2.y-p1.y)*p0.x - (p2.x-p1.x)*p0.y + p2.x*p1.y - p2.y*p1.x)/magnitude(p2-p1)
	Vector2d p1 	= Camera::GetInstance().project(lStart2);
	Vector2d p2 	= Camera::GetInstance().project(lEnd2);
	Vector2d diff 	= p2-p1;
	double grad		= diff[1]/diff[0];
	double grad_inv	= diff[0]/diff[1];
	if(p1[0]<0){
		p1[1]		+= -p1[0]*grad;
		p1[0]		= 0;
	}
	if(p1[0]>=imgW){
		p1[1]		+= (imgW-1-p1[0])*grad;
		p1[0]		= imgW-1;
	}
	if(p1[1]<0){
		p1[0]		+= -p1[1]*grad_inv;
		p1[1]		= 0;
	}
	if(p1[1]>=imgH){
		p1[0]		+= (imgH-1-p1[1])*grad_inv;
		p1[1]		= imgH-1;
	}
	if(p2[0]<0){
		p2[1]		+= -p2[0]*grad;
		p2[0]		= 0;
	}
	if(p2[0]>=imgW){
		p2[1]		+= (imgW-1-p2[0])*grad;
		p2[0]		= imgW-1;
	}
	if(p2[1]<0){
		p2[0]		+= -p2[1]*grad_inv;
		p2[1]		= 0;
	}
	if(p2[1]>=imgH){
		p2[0]		+= (imgH-1-p2[1])*grad_inv;
		p2[1]		= imgH-1;
	}
	diff			= p2-p1;
	double diffNorm = diff.norm();
	double term3	= p2[0]*p1[1]-p2[1]*p1[0];
	//find the search feature x range
	//extend the search range by distance threshold
	int xMin2, xMax2;
	if(p1[0]<=p2[0]){
		xMin2		= std::max(0,int(std::floor(p1[0]-EPIPOLAR_SEARCH_PIXEL_THRESH)));
		xMax2		= std::min(imgW-1, int(std::ceil(p2[0]+EPIPOLAR_SEARCH_PIXEL_THRESH)));
	}else{
		xMin2		= std::max(0,int(std::floor(p2[0]-EPIPOLAR_SEARCH_PIXEL_THRESH)));
		xMax2		= std::min(imgW-1, int(std::ceil(p1[0]+EPIPOLAR_SEARCH_PIXEL_THRESH)));
	}
	//determine the image row range to search
	//extend the search range by distance threshold
	int yMin2, yMax2;
	if(p1[1]<=p2[1]){
		yMin2		= std::max(0,int(std::floor(p1[1]-EPIPOLAR_SEARCH_PIXEL_THRESH)));
		yMax2		= std::min(imgH-1, int(std::ceil(p2[1]+EPIPOLAR_SEARCH_PIXEL_THRESH)));
	}else{
		yMin2		= std::max(0,int(std::floor(p2[1]-EPIPOLAR_SEARCH_PIXEL_THRESH)));
		yMax2		= std::min(imgH-1, int(std::ceil(p1[1]+EPIPOLAR_SEARCH_PIXEL_THRESH)));
	}
	//get feature search start and end idxs
	int feIdxStart	= frame2->kptLUT[yMin2];
	if(feIdxStart >= frame2->kpts.size()) return;
	int feIdxEnd	= frame2->kptLUT[yMax2];
	if(feIdxEnd >= frame2->kpts.size()) feIdxEnd = frame2->kpts.size()-1;
	if(feIdxEnd<feIdxStart) return;

	QList<QPointF> candidateFeatures;

	//create mask to be used for matching
	for(unsigned int j =feIdxStart; j<feIdxEnd; j++){
		Point2f &pt	= frame2->kpts[j].pt;
		//skip if not between the horizontal range
		if(pt.x<xMin2 || pt.x>xMax2) continue;
		double dist = std::fabs(diff[1]*pt.x - diff[0]*pt.y+term3)/diffNorm;
		//skip if not along epipolar line
		if(dist<=EPIPOLAR_SEARCH_PIXEL_THRESH){
			candidateFeatures.push_back(QPointF(pt.x, pt.y));
		}
	}
	imageView2->setMarks(candidateFeatures);

	QList<QPointF> lines;
	lines.push_back(QPointF(p1[0],p1[1]));
	lines.push_back(QPointF(p2[0],p2[1]));
	lines.push_back(QPointF(0,yMin2));
	lines.push_back(QPointF(1980,yMin2));
	lines.push_back(QPointF(0,yMax2));
	lines.push_back(QPointF(1980,yMax2));
	imageView2->setLines(lines);
}

void EpipolarPanel::handleImageView1PointSelected(const int idx){
	imageView1->setSelected(idx);

	QList<bool> img2Mask;

	Data &data = Data::GetInstance();
	Camera &camera = Camera::GetInstance();
	int imgIdx1 = imgList1->currentIndex()-1;
	int imgIdx2 = imgList2->currentIndex()-1;
	const Frame::Ptr &frame1 = data.getFrame(imgIdx1);
	const Frame::Ptr &frame2 = data.getFrame(imgIdx2);

	if(idx<0 || !frame1 || !frame2){
		if(frame2){
			imageView2->setMask(img2Mask);
		}
		return;
	}

	int searchRadius = 100;
	if(frame1->measured[idx]){
		//the point has a measure, project the 3d point to frame2
		const Measurement::Ptr &m = data.getMeasurement(frame1,idx);
		Camera::ProjectionStatus s;
		Vector2d pos = camera.project(frame2, m->landmark->pt,s);
		//skip if landmark is behind camera plane z=1
		if(s == Camera::ProjectionStatus::Behind){
			imageView2->setMask(img2Mask);
			return;
		}
		//skip if out of image given search radius
		if(camera.outofbound(pos, searchRadius)){
			imageView2->setMask(img2Mask);
			return;
		}
		//get feature search range around pos
		int xMin				= pos[0]-searchRadius;
		int xMax				= pos[0]+searchRadius;
		int yMin				= pos[1]-searchRadius;
		int yMax				= pos[1]+searchRadius;
		if(xMin<0) xMin = 0;
		if(yMin<0) yMin = 0;
		if(xMax>=camera.w) xMax = camera.w-1;
		if(yMax>=camera.h) yMax = camera.h-1;

		int feIdxStart			= frame2->kptLUT[yMin];
		if(feIdxStart >= frame2->decs.rows){
			imageView2->setMask(img2Mask);
			return;
		}
		int feIdxEnd			= frame2->kptLUT[yMax];
		if(feIdxEnd >= frame2->decs.rows) feIdxEnd = frame2->decs.rows-1;
		if(feIdxEnd<feIdxStart){
			imageView2->setMask(img2Mask);
			return;
		}

		for(int i=0; i<frame2->kpts.size(); i++){
			if(i>=feIdxStart && i<feIdxEnd){
				Point2f &pt	= frame2->kpts[i].pt;
				if(pt.x<xMin || pt.x>xMax){
					img2Mask.push_back(false);
				}else{
					img2Mask.push_back(true);
				}
			}else{
				img2Mask.push_back(false);
			}
		}
		imageView2->setMask(img2Mask);
		imageView2->setRect(QRect(xMin, yMin, xMax-xMin, yMax-yMin));

	}else{

	}
}

void EpipolarPanel::handleImagesUsed(std::vector<int> &usedImgIdxs){
	for(int i=0; i<imgList1->count(); i++){
		imgList1->setItemData(i, QColor(0,0,0), Qt::TextColorRole);
	}
	for(int i=0; i<imgList2->count(); i++){
		imgList2->setItemData(i, QColor(0,0,0), Qt::TextColorRole);
	}
	for(int i=0; i<usedImgIdxs.size(); i++){
		imgList1->setItemData(usedImgIdxs[i]+1, QColor(0,0,255), Qt::TextColorRole);
		imgList2->setItemData(usedImgIdxs[i]+1, QColor(0,0,255), Qt::TextColorRole);
	}

}

void EpipolarPanel::getSelectedImages(int &idx1, int &idx2){
	idx1 = imgList1->currentIndex()-1;
	idx2 = imgList2->currentIndex()-1;
}

void EpipolarPanel::getMask(QList<bool> &mask){
	mask = imageView1->getMask();
}
void EpipolarPanel::setImagePair(const int img1, const int img2){
	imgList1->setCurrentIndex(img1+1);
	imgList2->setCurrentIndex(img2+1);
}
