/*
 * KeyFrameModel.cpp
 *
 *  Created on: May 3, 2016
 *      Author: yoyo
 */

#include "KeyFrameModel.h"
#include "CoreInterfaceWidget.h"
#include "core/ptam/KeyFrame.h"
#include "core/ptam/LevelHelpers.h"
#include <iostream>
#include <QDebug>
#include <cvd/vision.h>
#include <cvd/fast_corner.h>


using namespace std;
KeyFrameModel::KeyFrameModel(CoreInterfaceWidget *_coreInterface) {
	coreInterface = _coreInterface;

}

KeyFrameModel::~KeyFrameModel() {
	// TODO Auto-generated destructor stub
}

void KeyFrameModel::computeKeyFrame(const int idx){
	cout<<"keyframe model is computing keyframe for image ["<<idx<<"]"<<endl;
	if(!(coreInterface->coreIsSet())){
		cout<<"keyframe model Error: core is not set!";
		return;
	}
	//setImageIdx(idx);
	KeyFrame kf;
	int levels = 4;
	coreInterface->computeKeyFrame(idx, kf);
	cout<<"level 0 image size: "<<kf.aLevels[0].im.size()<<"corners: "<<kf.aLevels[0].vMaxCorners.size()<<"/"<<kf.aLevels[0].vCorners.size()<<endl;
	cout<<"level 1 image size: "<<kf.aLevels[1].im.size()<<"corners: "<<kf.aLevels[1].vMaxCorners.size()<<"/"<<kf.aLevels[1].vCorners.size()<<endl;
	cout<<"level 2 image size: "<<kf.aLevels[2].im.size()<<"corners: "<<kf.aLevels[2].vMaxCorners.size()<<"/"<<kf.aLevels[2].vCorners.size()<<endl;
	cout<<"level 3 image size: "<<kf.aLevels[3].im.size()<<"corners: "<<kf.aLevels[3].vMaxCorners.size()<<"/"<<kf.aLevels[3].vCorners.size()<<endl;

	QList<QList<QPointF> > levelCorners;
	for(int i=0; i<levels; i++){
		int cornerSize = kf.aLevels[i].vCorners.size();
		QList<QPointF> corners;
		for(int j=0; j<cornerSize; j++){
			int x = kf.aLevels[i].vCorners[j].x;
			int y = kf.aLevels[i].vCorners[j].y;
			double x_level0 = LevelZeroPos(x,i);
			double y_level0 = LevelZeroPos(y,i);
			corners.push_back(QPointF(x_level0,y_level0));
		}
		levelCorners.push_back(corners);
	}

	emit keyFrameCornersReady(levelCorners);
}

void KeyFrameModel::setImageIdx(const int idx){
	imgIdx = idx-1;
	cout<<"keyframe model set image index to "<<imgIdx<<endl;
}
