/*
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#include <QtGui>
#include <QDebug>

#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <vector>
#include <string>
#include "CoreInterfaceWidget.h"
#include "core/SFMPipeline.h"
#include "core/PathReader.h"
#include "core/ptam/KeyFrame.h"

using namespace std;
using namespace cv;
TaskThread::TaskThread(SFMPipeline *core){
	mCore = core;
	currentTask = TASK_NONE;
}

void TaskThread::setTask(const int &taskID){
	currentTask = taskID;
}
void TaskThread::setDeleteIdxs(const QList<int> &idxs){
	deleteIdxs = idxs;
}
void TaskThread::setImagePair(const int &imgIdx1, const int &imgIdx2){
	mImgIdx1 = imgIdx1;
	mImgIdx2 = imgIdx2;
}

void TaskThread::setReconstructionParameters(	const int 				&imgIdx1,
												const int 				&imgIdx2,
												const QList<bool>		&mask)
{
	mImgIdx1 	= imgIdx1;
	mImgIdx2 	= imgIdx2;
	mMask		= mask;
}

void TaskThread::getImagePair(int &imgIdx1, int &imgIdx2){
	imgIdx1 	= mImgIdx1;
	imgIdx2 	= mImgIdx2;
}
void TaskThread::run(){
	switch(currentTask){
		case TASK_RECONSTRUCT:
		{
			vector<KeyPoint> 	kpts1,kpts2;
			Mat					decs1,decs2;
			vector<DMatch>		matches,maskedMatches;
			mCore->getKptsAndDecs(mImgIdx1,kpts1,decs1);
			mCore->getKptsAndDecs(mImgIdx2,kpts2,decs2);
			mCore->matchFeatures(decs1,decs2,matches);

			assert(matches.size()==mMask.size());
			for(int i=0; i<matches.size(); i++){
				if(mMask[i]){
					maskedMatches.push_back(matches[i]);
				}
			}
			if(mCore->ptCloud.pt3Ds.empty()){
				//mCore->processBasePair();

				if(mCore->checkMatchesBasePair(kpts1,kpts2,maskedMatches)){
					mCore->reconstructBasePair(mImgIdx1, mImgIdx2, kpts1, kpts2, decs1, decs2, maskedMatches);
				}

			}else{
				//mCore->processAddNewCamera();

				if(mCore->checkMatchesNextPair(mImgIdx1, mImgIdx2, kpts1,kpts2,maskedMatches)){
					mCore->reconstructNextPair(mImgIdx1, mImgIdx2, kpts1, kpts2, decs1, decs2, maskedMatches);
				}

			}
			//mCore->bundleAdjustment();
			//mCore->pruneHighReprojectionErrorPoints();
			mCore->printDebug();
		}
			break; 
		case TASK_DELETEPOINTS:
		{
			vector<bool> removeMask(mCore->ptCloud.pt3Ds.size(),false);
			for(int i=0; i<deleteIdxs.size(); i++){
				removeMask[deleteIdxs[i]]=true;
			}
			mCore->ptCloud.remove3Ds(removeMask);
		}
			break;
		case TASK_BUNDLEADJUST:
		{
			if(!(mCore->ptCloud.pt3Ds.empty())){
				mCore->bundleAdjustment();
			}
		}
			break;
		case TASK_NEXTPAIR:
		{
			mCore->getNextPair(mImgIdx1, mImgIdx2);
		}
			break;
		default:
			break;
	}
	
	emit finished();
}

CoreInterfaceWidget::CoreInterfaceWidget(){
	core = NULL;
	tt 	 = NULL;
}
void CoreInterfaceWidget::setImagePaths(const QString &root, const QList<QString> &list){
	vector<string> paths;
	paths.reserve(list.size());
	for(int i=0; i<list.size(); i++){
		string path = list[i].toStdString();
		paths.push_back(path);
	}
	if(core!=NULL){
		delete core;
		core = NULL;
	}
	if(tt!=NULL){
		delete tt;
		tt = NULL;
	}
	core = new SFMPipeline(root.toStdString(), paths);
	tt = new TaskThread(core);
}

void CoreInterfaceWidget::getImagePaths(QString &root, QList<QString> &list){
	if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image folder is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	list.clear();
	root = QString::fromStdString(core->ptCloud.imgRoot);
	list.reserve(core->ptCloud.imgs.size());
	for(int i=0; i<(core->ptCloud.imgs.size()); i++){
		list.push_back(QString::fromStdString(core->ptCloud.imgs[i]));
	}
}

void CoreInterfaceWidget::nextPair(){
	if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image folder is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	cout<<"core interface select next pair"<<endl;
	int task = TaskThread::TASK_NEXTPAIR;
	tt->setTask(task);
	connect(tt, SIGNAL(finished()), this, SLOT(handleNextPairFinished()));
	tt->start();
}

void CoreInterfaceWidget::handleNextPairFinished(){
	disconnect(tt, SIGNAL(finished()), this, SLOT(handleNextPairFinished()));
	tt->getImagePair(imgIdx1, imgIdx2);
	emit nextPairReady(imgIdx1, imgIdx2);
}

void CoreInterfaceWidget::reconstruct(const QList<bool> &mask){
	if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image folder is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	cout<<"core interface reconstruct"<<endl;

	int task = TaskThread::TASK_RECONSTRUCT;
	tt->setTask(task);
	tt->setReconstructionParameters(imgIdx1,imgIdx2,mask);
	connect(tt, SIGNAL(finished()), this, SLOT(handleReconstructFinished()));
	tt -> start();

}
void CoreInterfaceWidget::handleReconstructFinished(){
	disconnect(tt, SIGNAL(finished()), this, SLOT(handleReconstructFinished()));
	emit pointCloudReady(true);
}
void CoreInterfaceWidget::bundleAdjust(){
	if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image folder is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	cout<<"core interface bundle adjust"<<endl;
	int task = TaskThread::TASK_BUNDLEADJUST;
	tt->setTask(task);
	connect(tt, SIGNAL(finished()), this, SLOT(handleBundleAdjustFinished()));
	tt -> start();
}
void CoreInterfaceWidget::handleBundleAdjustFinished(){
	disconnect(tt, SIGNAL(finished()), this, SLOT(handleBundleAdjustFinished()));
	emit pointCloudReady(true);
}
void CoreInterfaceWidget::deletePointIdx(const QList<int> idxs){
	if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image folder is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	cout<<"core interface delete points"<<endl;
	int task = TaskThread::TASK_DELETEPOINTS;
	tt->setTask(task);
	tt->setDeleteIdxs(idxs);
	connect(tt, SIGNAL(finished()), this, SLOT(handleDeletePointIdxFinished()));
	tt -> start();
}
void CoreInterfaceWidget::handleDeletePointIdxFinished(){
	disconnect(tt, SIGNAL(finished()), this, SLOT(handleDeletePointIdxFinished()));
	emit pointCloudReady(true);
}
void CoreInterfaceWidget::matchImages(	const int &_imgIdx1,
										const int &_imgIdx2){
	if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image folder is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	imgIdx1 = _imgIdx1;
	imgIdx2 = _imgIdx2;
	vector<KeyPoint> 	kpts1,kpts2;
	Mat					decs1,decs2;
	vector<DMatch>		matches;
	cout<<"core interface match images ["<<imgIdx1<<"]->["<<imgIdx2<<"]"<<endl;
	core->getKptsAndDecs(imgIdx1, kpts1, decs1);
	core->getKptsAndDecs(imgIdx2, kpts2, decs2);
	core->matchFeatures(decs1,decs2,matches);

	//GOOD TO KNOW
	//convert cv mat to QImage
	//matchDrawing = QImage((uchar*) display.data, display.cols, display.rows, display.step, QImage::Format_RGB888);
	QList<QPointF> pts1, pts2;
	for(int i=0; i<matches.size(); i++){
		int pt1Idx = matches[i].queryIdx;
		int pt2Idx = matches[i].trainIdx;
		Point2f pt1= kpts1[pt1Idx].pt;
		Point2f pt2= kpts2[pt2Idx].pt;
		pts1.push_back(QPointF(pt1.x,pt1.y));
		pts2.push_back(QPointF(pt2.x,pt2.y));
	}
	cout<<"core interface matches found = "<<pts1.size()<<endl;
	emit matchResultReady(pts1, pts2);
}
void CoreInterfaceWidget::checkMatch(const QList<bool> &mask){
	if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image folder is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	vector<KeyPoint> 	kpts1,kpts2;
	Mat					decs1,decs2;
	vector<DMatch>		matches,maskedMatches;
	core->getKptsAndDecs(imgIdx1,kpts1,decs1);
	core->getKptsAndDecs(imgIdx2,kpts2,decs2);
	core->matchFeatures(decs1,decs2,matches);
	assert(matches.size()==mask.size());
	for(int i=0; i<matches.size(); i++){
		if(mask[i]){
			maskedMatches.push_back(matches[i]);
		}
	}
	if(core->ptCloud.pt3Ds.empty()){
		if(!(core->checkMatchesBasePair(kpts1,kpts2,maskedMatches))){
			QMessageBox messageBox;
			messageBox.critical(0,"Error","check failed!");
		}

	}else{
		if(!(core->checkMatchesNextPair(imgIdx1, imgIdx2, kpts1,kpts2,maskedMatches))){
			QMessageBox messageBox;
			messageBox.critical(0,"Error","check failed!");
		}
	}
}

void CoreInterfaceWidget::removeBad(){

	if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image folder is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	core->pruneHighReprojectionErrorPoints();
	emit pointCloudReady(true);
}
void CoreInterfaceWidget::denseReconstruct(){
	if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image folder is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
}

void CoreInterfaceWidget::computeKeyFrame(const int imgIdx, KeyFrame &kf){
	if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image folder is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	core->computeKeyFrame(imgIdx,kf);
}

void CoreInterfaceWidget::getPointCloud(vector<Point3f> &xyzs){
	if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image folder is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	core-> ptCloud.getXYZs(xyzs);
}
void CoreInterfaceWidget::getCameras(vector<Matx34d> &cams){
	if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image folder is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	cams = core->ptCloud.camMats;
}
void CoreInterfaceWidget::getUsedImageIdxs(std::vector<int> &usedImgIdxs){
	if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image folder is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	core->ptCloud.getUsedImageIdxs(usedImgIdxs);
}

void CoreInterfaceWidget::getAll3DfromImage2D(	const int 					imgIdx,
												std::vector<cv::Point3f>	&pts3D,
												std::vector<int>			&pts3DIdxs)
{
	if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image folder is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	if(core->ptCloud.imageIsUsed(imgIdx)){
		core->ptCloud.getAll3DfromImage2D(imgIdx,pts3D,pts3DIdxs);
	}
}
void CoreInterfaceWidget::getCameraIdx(			const int 					imgIdx,
												int 						&camIdx)
{
	if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image folder is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	if(core->ptCloud.imageIsUsed(imgIdx)){
		camIdx = core->ptCloud.img2camMat[imgIdx];
	}else{
		camIdx = -1;
	}
}
void CoreInterfaceWidget::getOverlappingImgs(	const int 					baseImgIdx,
												map<int,vector<int> > 		&img2pt3Didxs)
{
	core->ptCloud.getOverlappingImgs(baseImgIdx, img2pt3Didxs);
}
void CoreInterfaceWidget::getBestOverlappingImgs(const int 					baseImgIdx,
												map<int,vector<int> > 		&img2pt3Didxs)
{
	core->ptCloud.getBestOverlappingImgs(baseImgIdx, img2pt3Didxs);
}
void CoreInterfaceWidget::saveProject(const QString &fname){
	if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image folder is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	cout<<"core interface save project"<<endl;
	//core-> writePLY("outputs","");
	core-> saveProject(fname.toStdString());
}

void CoreInterfaceWidget::loadProject(const QString &pname){
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	cout<<"core interface load project"<<endl;
	setImagePaths(QString(), QList<QString>());	// just to recreate the core
	core ->loadProject(pname.toStdString());
	emit projectLoaded();
	if(core->ptCloud.pt3Ds.size()>0){
		emit pointCloudReady(true);
	}
}

void CoreInterfaceWidget::loadGPS(const QString &fname){
	if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","project is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	cout<<"core interface load gps"<<endl;
	core ->loadGPS(fname.toStdString());

}

void CoreInterfaceWidget::ApplyGlobalTransformation(const std::vector<double> &transformation){
	cout<<"core interface transform map using given transformation matrix"<<endl;

	cv::Matx34d transfMat(	transformation[0], transformation[1], transformation[2], transformation[3],
							transformation[4], transformation[5], transformation[6], transformation[7],
							transformation[8], transformation[9], transformation[10], transformation[11]);
	cout<<transfMat<<endl;

	core->ptCloud.ApplyGlobalTransformation(Mat(transfMat));
	emit pointCloudReady(true);	//need to reset camera view
}
