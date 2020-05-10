/*
 *  Widget to interface with core API
 */


#include <QtWidgets>
#include <QDebug>

#include <opencv2/highgui/highgui.hpp>

#include <iostream>
#include <vector>
#include <string>
#include "CoreInterfaceWidget.h"
#include "core/SFMPipeline.h"
#include "core/datastructs/Data.h"
// #include "core/SFMVideoPipeline.h"
#include "core/ProjectIO.h"

#include <Eigen/Eigen>

using namespace std;
using namespace cv;
using namespace Eigen;

TaskThread::TaskThread(SFMPipeline *core)
:mCore(core)
,mImgIdx1(0)
,mImgIdx2(0)
,currentTask(TASK_NONE)
{

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
	Data &data = Data::GetInstance();

	switch(currentTask){
		case TASK_RECONSTRUCT:
		{

			if(data.countFrames()==0){
				mCore->reconstructBasePair(mImgIdx1, mImgIdx2);
			}else{
				mCore->positionAndAddFrame(mImgIdx1);
			}
			//mCore->bundleAdjustment();
			//mCore->pruneHighReprojectionErrorPoints();
			mCore->printDebug();
		}
			break; 
		case TASK_DELETEPOINTS:
		{
			for(int i=0; i<deleteIdxs.size(); i++){
				data.deleteLandMark(data.getLandMarks()[deleteIdxs[i]]);
			}
			data.deleteTrashes();
			cout<<"finished deletion"<<endl;
		}
			break;
		case TASK_BUNDLEADJUST:
		{
			mCore->bundleAdjustment();

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

CoreInterfaceWidget::CoreInterfaceWidget()
:core(NULL)
,tt(NULL)
{
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
	root = QString::fromStdString(core->imgRoot);
	list.reserve(core->imgNames.size());
	for(int i=0; i<(core->imgNames.size()); i++){
		list.push_back(QString::fromStdString(core->imgNames[i]));
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
	int imgIdx1, imgIdx2;
	tt->getImagePair(imgIdx1, imgIdx2);
	emit nextPairReady(imgIdx1, imgIdx2);
}

void CoreInterfaceWidget::reconstruct(	const int& 			imgIdx1,
										const int& 			imgIdx2,
										const QList<bool> &	mask){
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

// void CoreInterfaceWidget::poseOptimization(	int 			imgIdx)
// {
// 	if(!coreIsSet()){
// 		QMessageBox messageBox;
// 		messageBox.critical(0,"Error","image folder is not loaded!");
// 		return;
// 	}
// 	if(tt!=NULL && tt->isRunning()){
// 		QMessageBox messageBox;
// 		messageBox.critical(0,"Error","previous task is still running!");
// 		return;
// 	}

// 	Data &data 								= Data::GetInstance();
// 	Frame::Ptr frame 						= data.getFrame(imgIdx);
// 	const vector<Measurement::Ptr> &ms 		= data.getMeasurements(frame);
// 	core->optimizeCamPose_gv(frame, ms, 3.0, 4);

// 	emit pointCloudReady(true);
// }

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
void CoreInterfaceWidget::deletePointIdx(const QList<int> idxs, const int cloudID){
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

void CoreInterfaceWidget::matchImages(	const int &imgIdx1,
										const int &imgIdx2){
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

	Data &data = Data::GetInstance();
	Frame::Ptr frame1 = data.getFrame(imgIdx1);
	Frame::Ptr frame2 = data.getFrame(imgIdx2);

	if(!frame1){
		frame1.reset(new Frame(imgIdx1,core->imgRoot, core->imgNames[imgIdx1]));
		frame1->init();
	}
	if(!frame2){
		frame2.reset(new Frame(imgIdx2,core->imgRoot, core->imgNames[imgIdx2]));
		frame2->init();
	}
	vector<DMatch> matches;
	core->matchFrames(frame1, frame2, matches);

	//XXX:GOOD TO KNOW
	//convert cv mat to QImage
	//matchDrawing = QImage((uchar*) display.data, display.cols, display.rows, display.step, QImage::Format_RGB888);
	QList<QPointF> pts1, pts2;
	vector<KeyPoint> &kpts1 = frame1->kpts;
	vector<KeyPoint> &kpts2 = frame2->kpts;

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

void CoreInterfaceWidget::matchImagesEpipolar(	const int &imgIdx1,
												const int &imgIdx2)
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

	Data &data = Data::GetInstance();
	Frame::Ptr frame1 = data.getFrame(imgIdx1);
	Frame::Ptr frame2 = data.getFrame(imgIdx2);

	assert(frame1 && frame2);

	/*
	Mat mask12;
	core->epipolarConstrainedMatchMask(imgIdx1, imgIdx2, mask12);
	BFMatcher matcher(NORM_HAMMING,false);	//for masked match, cannot use cross check
	//do cross checking in the hard way
	vector<DMatch> matches;
	matcher.match(frame1->decs,frame2->decs,matches,mask12);*/

	Mat mask12, mask21;
	core->epipolarConstrainedMatchMask(imgIdx1, imgIdx2, mask12);
	core->epipolarConstrainedMatchMask(imgIdx2, imgIdx1, mask21);
	vector<DMatch> matches;
	if(!(mask12.empty() || mask21.empty())){
		core->matchFeatures(frame1->decs, frame2->decs, matches, mask12, mask21);
	}


	QList<QPointF> pts1, pts2;
	vector<KeyPoint> &kpts1 = frame1->kpts;
	vector<KeyPoint> &kpts2 = frame2->kpts;

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
void CoreInterfaceWidget::checkMatch(	const int& 			imgIdx1,
										const int& 			imgIdx2,
										const QList<bool> &	mask){
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

	Data &data = Data::GetInstance();
	Frame::Ptr frame1 = data.getFrame(imgIdx1);
	Frame::Ptr frame2 = data.getFrame(imgIdx2);
	if(!frame1){
		frame1.reset(new Frame(imgIdx1, core->imgRoot, core->imgNames[imgIdx1]));
		frame1->init();
	}
	if(!frame2){
		frame2.reset(new Frame(imgIdx2, core->imgRoot, core->imgNames[imgIdx2]));
		frame2->init();
	}
	vector<KeyPoint> &kpts1 = frame1->kpts;
	vector<KeyPoint> &kpts2 = frame2->kpts;

	vector<DMatch> matches, maskedMatches;
	core->matchFrames(frame1, frame2, matches);
	assert(matches.size()==mask.size());
	for(int i=0; i<matches.size(); i++){
		if(mask[i]){
			maskedMatches.push_back(matches[i]);
		}
	}
	if(data.countFrames() == 0){
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
void CoreInterfaceWidget::addMoreMeasures(	const int& 		imgIdx){
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
	Data &data = Data::GetInstance();
	Frame::Ptr frame = data.getFrame(imgIdx);

	const vector<Measurement::Ptr> &ms		= data.getMeasurements(frame);
	const vector<Measurement::Ptr> &ms1new 	= core->getMoreMeasuresByProjection(frame,nullptr,ms,10);
	for(vector<Measurement::Ptr>::const_iterator it = ms1new.begin(); it!=ms1new.end(); ++it){
		data.addMeasurement(*it);
	}

	//core->addMoreMeasures(frame);
	emit pointCloudReady(true);
}
void CoreInterfaceWidget::addMorePoints(	const int& 		imgIdx1,
											const int& 		imgIdx2)
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
	core->addMoreLandMarksAndMeasures(imgIdx1, imgIdx2);
	// core->addMoreLandMarksAndMeasures_gv(imgIdx1, imgIdx2);
	emit pointCloudReady(true);
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
	core->pruneHighErrorMeasurements();
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

	Data &data = Data::GetInstance();

	for(int i=0; i<imgIdxs.size(); i++){
		Frame::Ptr f = data.getFrame(imgIdxs[i]);
		if(f){
			data.deleteFrame(f);
		}
	}
	data.deleteTrashes();

	emit pointCloudReady(true);
}

void CoreInterfaceWidget::keepCameraByImageIdxs(const std::vector<int> &imgIdxs){
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

	Data &data = Data::GetInstance();
	vector<int> deleteIdxs;
	const vector<Frame::Ptr> &frames = data.getFrames();
	for(vector<Frame::Ptr>::const_iterator it = frames.begin(); it!=frames.end(); ++it){
		bool shouldDelete = true;
		for(int i=0; i<imgIdxs.size(); i++){
			if(imgIdxs[i] == (*it)->imgIdx){
				shouldDelete = false;
				break;
			}
		}
		if(shouldDelete){
			deleteIdxs.push_back((*it)->imgIdx);
		}
	}

	deleteCameraByImageIdxs(deleteIdxs);
	
}


void CoreInterfaceWidget::getUsedImageIdxs(std::vector<int> &usedImgIdxs){
	/*if(!coreIsSet()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","image folder is not loaded!");
		return;
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}*/

	usedImgIdxs.clear();
	const vector<Frame::Ptr> &fs = Data::GetInstance().getFrames();
	for(vector<Frame::Ptr>::const_iterator it = fs.begin(); it!=fs.end(); ++it){
		usedImgIdxs.push_back((*it)->imgIdx);
	}
}

void CoreInterfaceWidget::getMeasurementsByFrames(	const vector<int> &			imgIdxs,
													vector<Measurement::Ptr>& 	measures)
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

	measures.clear();
	Data &data = Data::GetInstance();
	for(vector<int>::const_iterator it = imgIdxs.begin(); it!= imgIdxs.end(); ++it){
		Frame::Ptr f = data.getFrame(*it);
		if(f){
			vector<Measurement::Ptr> ms = data.getMeasurements(f);
			for(vector<Measurement::Ptr>::iterator jt = ms.begin(); jt!= ms.end(); ++jt){
				measures.push_back(*jt);
			}
		}
	}
}

void CoreInterfaceWidget::getMeasurementsByLandMarks(	const vector<int> &			landmarkIdxs,
														vector<Measurement::Ptr>&	measures)
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

	measures.clear();

	Data &data = Data::GetInstance();
	const vector<LandMark::Ptr>& lms = data.getLandMarks();
	for(vector<int>::const_iterator it = landmarkIdxs.begin(); it!=landmarkIdxs.end(); ++it){
		vector<Measurement::Ptr> ms = data.getMeasurements(lms[*it]);
		for(vector<Measurement::Ptr>::iterator jt = ms.begin(); jt!=ms.end(); ++jt){
			measures.push_back(*jt);
		}
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
	core-> saveProject(fname.toStdString());
}

void CoreInterfaceWidget::loadProject(const QString &pname, const int cloudIdx){
	if (cloudIdx != 0){
		if(!coreIsSet()){
			QMessageBox messageBox;
			messageBox.critical(0,"Error","CloudViewer 1 must be loaded first!");
			return;
		}
	}
	if(tt!=NULL && tt->isRunning()){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","previous task is still running!");
		return;
	}
	cout<<"core interface load project"<<endl;

	setImagePaths(QString(), QList<QString>());	// just to recreate the core

	core ->loadProject(pname.toStdString());

	emit projectLoaded();
	emit pointCloudReady(true);


}

void CoreInterfaceWidget::ApplyGlobalTransformation(const std::vector<double> &transformation){
	cout<<"currently not supported : core interface transform map using given transformation matrix"<<endl;

	cv::Matx34d transfMat(	transformation[0], transformation[1], transformation[2], transformation[3],
							transformation[4], transformation[5], transformation[6], transformation[7],
							transformation[8], transformation[9], transformation[10], transformation[11]);
	cout<<transfMat<<endl;

	Data::GetInstance().applyGlobalTransformation(Mat(transfMat));
	emit pointCloudReady(true);	//need to reset camera view*/
}

void CoreInterfaceWidget::saveFeatures(				const string& 			saveFolder){
	Data &data = Data::GetInstance();


	if(data.getFrames().empty()){
		data.reset();
		//all images
		for(unsigned int i=0; i<core->imgNames.size(); i++){
			Frame::Ptr frame(new Frame(i,core->imgRoot, core->imgNames[i]));
			frame->init();
			ProjectIO::writeFeatures(saveFolder,frame->imgName,frame->kpts, frame->decs);
			cout<<"saved features for "<<frame->imgName<<endl;
			data.addFrame(frame);

			/*Mat tmp;
			if(frame->img.empty()){
				tmp 	= cv::imread(frame->imgRoot+"/"+frame->imgName,cv::IMREAD_GRAYSCALE);
			}else{
				frame->img.copyTo(tmp);
			}
			drawKeypoints(tmp, frame->kpts, tmp, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			imwrite(saveFolder+"/"+frame->imgName,tmp);*/


		}
		// data.makeVocabDB();
		// data.saveVocabDB(saveFolder+"/"+"vocabDB.yaml.gz");

		//all pair match
		const vector<Frame::Ptr> &frames = data.getFrames();
		for(vector<Frame::Ptr>::const_iterator it = frames.begin(); it!=frames.end(); ++it){
			for(vector<Frame::Ptr>::const_iterator jt = frames.begin(); jt!=frames.end(); ++jt){
				if(it == jt) continue;
				vector<DMatch> matches;
				core->matchFrames(*it, *jt, matches);
				ProjectIO::writeMatches(saveFolder+"/matches.txt",(*it)->imgName, (*jt)->imgName, matches);
				cout<<"saved matches for "<<(*it)->imgName<<" & "<<(*jt)->imgName<<endl;
			}
		}
	}else{
		const vector<Frame::Ptr> &frames = data.getFrames();
		for(vector<Frame::Ptr>::const_iterator it = frames.begin(); it!=frames.end(); ++it){

			ProjectIO::writeFeatures(saveFolder,(*it)->imgName,(*it)->kpts, (*it)->decs);
			cout<<"saved features for "<<(*it)->imgName<<endl;
			/*Mat tmp;
			if((*it)->img.empty()){
				tmp 	= cv::imread((*it)->imgRoot+"/"+(*it)->imgName,cv::IMREAD_GRAYSCALE);
			}else{
				(*it)->img.copyTo(tmp);
			}
			drawKeypoints(tmp, (*it)->kpts, tmp, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
			imwrite(saveFolder+"/"+(*it)->imgName,tmp);*/

		}
	}
}
