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
			mCore->ptCloud.removeRedundancy();
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

void CoreInterfaceWidget::deleteCameraByImageIdxs(const std::vector<int> &imgIdxs){
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
	set<int> camIdxs;
	for(int i=0; i<imgIdxs.size(); i++){
		if(core->ptCloud.imageIsUsed(imgIdxs[i])){
			int camIdx = core->ptCloud.img2camMat[imgIdxs[i]];
			camIdxs.insert(camIdx);
		}
	}
	core->ptCloud.removeCameras(camIdxs);
	core->ptCloud.removeRedundancy();

	emit pointCloudReady(true);
}

void CoreInterfaceWidget::deleteMeasures(const QList<QPair<int,int> > &measures){
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
	vector<pair<int,int> > ms;
	ms.reserve(measures.size());
	for(int i=0; i<measures.size(); i++){
		ms.push_back(make_pair(measures[i].first, measures[i].second));
	}
	core->ptCloud.removeMeasures(ms);
	core->ptCloud.removeRedundancy();

	emit pointCloudReady(false);
}

void CoreInterfaceWidget::keepMinSpanCameras(){
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
	core->keepMinSpanCameras();
	emit pointCloudReady(true);
}
void CoreInterfaceWidget::denseReconstruct(){
	QMessageBox messageBox;
	messageBox.information(0,"Info","function not supported yet");
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
void CoreInterfaceWidget::getPointCloud2(vector<Point3f> &xyzs){
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
	core-> ptCloud2.getXYZs(xyzs);
}
void CoreInterfaceWidget::getPolygons(vector<Point3f> &verts, vector<Point3i> &faces){
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
	core-> poly.getPolygons(verts, faces);
}
void CoreInterfaceWidget::getVisiblePolygons(const int imgIdx, vector<Point3f> &verts, vector<Point3i> &faces){
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
	core-> getVisiblePolygons(imgIdx, verts, faces);
}
void CoreInterfaceWidget::getPointNormals(	vector<cv::Point3f> &norms){
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
	core-> ptCloud.getPointNormals(norms);
}
void CoreInterfaceWidget::getPointNormals2(	vector<cv::Point3f> &norms){
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
	core-> ptCloud2.getPointNormals(norms);
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
void CoreInterfaceWidget::getCameras2(vector<Matx34d> &cams){
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
	cams = core->ptCloud2.camMats;
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
void CoreInterfaceWidget::getMeasuresToPoints(	const vector<int> 						&pt3DIdxs,
												vector<vector<pair<int,int> > >			&pt3D2Measures,
												vector<vector<QPointF> > 				&pt3D2pt2Ds){
	pt3D2pt2Ds.clear();
	vector<vector<Point2f> > pt3D2pt2Ds_cv;
	core->ptCloud.getMeasuresToPoints(pt3DIdxs, pt3D2Measures, pt3D2pt2Ds_cv);
	for(int i=0; i<pt3D2pt2Ds_cv.size(); i++){
		vector<QPointF> pt2Ds;
		for(int j=0; j<pt3D2pt2Ds_cv[i].size(); j++){
			QPointF p = QPointF(pt3D2pt2Ds_cv[i][j].x,pt3D2pt2Ds_cv[i][j].y);
			pt2Ds.push_back(p);
		}
		pt3D2pt2Ds.push_back(pt2Ds);
	}
}
void CoreInterfaceWidget::saveProject(const QString &fname, const int cloudIdx){
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
	core-> saveProject(fname.toStdString(), cloudIdx);
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

void CoreInterfaceWidget::loadPolygon(const QString &fname){
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
	cout<<"core interface load polygons"<<endl;
	core ->loadPolygon(fname.toStdString());
	emit polygonReady(true);	//need to reset camera view

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

void CoreInterfaceWidget::projectImagePointsTo3DSurface(	const int 						imgIdx,
															const QList<QPointF> 			&xys,
															QList<QVector3D>				&xyzs,
															QList<QVector3D>				&norms,
															vector<bool>					&status)
{
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
	cout<<"core interface project image points to 3d surface"<<endl;
	vector<Point2f> pt2Ds;
	vector<Point3f> pt3Ds;
	vector<Point3f> normals;
	pt2Ds.reserve(xys.size());
	for(int i=0; i<xys.size(); i++){
		pt2Ds.push_back(Point2f(xys[i].x(),xys[i].y()));
	}
	core->projectImagePointsTo3DSurface(imgIdx, pt2Ds, pt3Ds, normals, status);

	for(int i=0; i<pt3Ds.size(); i++){
		xyzs.push_back(QVector3D(pt3Ds[i].x,pt3Ds[i].y,pt3Ds[i].z));
	}
	for(int i=0; i<normals.size(); i++){
		norms.push_back(QVector3D(normals[i].x,normals[i].y,normals[i].z));
	}

}

void CoreInterfaceWidget::projectPolygonToImage(	const int 						imgIdx,
													QList<QPointF>					&xys)
{
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
	cout<<"core interface project polygon to image "<<imgIdx<<endl;
	vector<Point2f> verts;
	vector<Point3i> faces;
	core->projectPolygonToImage(imgIdx,verts,faces);
	xys.clear();
	xys.reserve(faces.size()*3);
	for(int i=0; i<faces.size(); i++){
		int vId0 = faces[i].x;
		int vId1 = faces[i].y;
		int vId2 = faces[i].z;
		xys.push_back(QPointF(verts[vId0].x,verts[vId0].y));
		xys.push_back(QPointF(verts[vId1].x,verts[vId1].y));
		xys.push_back(QPointF(verts[vId2].x,verts[vId2].y));
	}
}

void CoreInterfaceWidget::addKeyFrame(		const int 						imgIdx,
											const QList<QPointF> 			&xys,
											const QList<QVector3D>			&xyzs,
											const QList<QVector3D>			&norms,
											const std::vector<bool>			&status)
{
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
	cout<<"core interface add keyframe"<<endl;
	if(core->ptCloud2.imageIsUsed(imgIdx)){
		cout<<"keyframe already added"<<endl;
		return;
	}
	//set has normal
	core->ptCloud2.hasPointNormal = true;

	//add images
	if(core->ptCloud2.imgs.empty()){
		core->ptCloud2.imgs = core->ptCloud.imgs;
	}

	//add camera
	int camIdx = core->ptCloud.img2camMat[imgIdx];
	core->ptCloud2.camMats.push_back(core->ptCloud.camMats[camIdx]);
	int camIdx2= core->ptCloud2.camMats.size()-1;
	core->ptCloud2.camMat2img[camIdx2] = imgIdx;
	core->ptCloud2.img2camMat[imgIdx] = camIdx2;

	//add gps
	core->ptCloud2.img2GPS[imgIdx] = core->ptCloud.img2GPS[imgIdx];
	cout<<"gps = "<<core->ptCloud.img2GPS[imgIdx].first<<","<<core->ptCloud.img2GPS[imgIdx].second<<endl;

	assert(xys.size() == xyzs.size());
	assert(xyzs.size() == norms.size());
	assert(status.size() == norms.size());
	for(int i=0; i<status.size(); i++){
		if(!status[i]) continue;
		//add 3D
		Pt3D pt3D;
		pt3D.pt = Point3f(xyzs[i].x(),xyzs[i].y(),xyzs[i].z());
		pt3D.norm = Point3f(norms[i].x(),norms[i].y(),norms[i].z());
		pt3D.img2ptIdx[imgIdx] = i;
		core->ptCloud2.pt3Ds.push_back(pt3D);

		//add 2D
		Pt2D pt2D;
		pt2D.img_idx 	= imgIdx;
		pt2D.pt 		= Point2f(xys[i].x(), xys[i].y());
		pt2D.pt3D_idx 	= core->ptCloud2.pt3Ds.size()-1;
		pt2D.dec		= Mat(1,32, CV_8UC1, Scalar(0)); // dummy orb descriptor
		if(core->ptCloud2.img2pt2Ds.find(imgIdx) == core->ptCloud2.img2pt2Ds.end()){
			core->ptCloud2.img2pt2Ds[imgIdx] = vector<Pt2D>();
		}
		core->ptCloud2.img2pt2Ds[imgIdx].push_back(pt2D);
	}
	emit pointCloud2Ready(true);	//need to reset camera view
}
