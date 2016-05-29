/*
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#include <QtGui>

#include <opencv2/core/core.hpp>

#include <iostream>

#include "MainWindow.h"
#include "CloudWidget.h"
#include "CoreInterfaceWidget.h"
#include "ImageWidget.h"
#include "MatchWidget.h"
#include "MatchPanel.h"
#include "MatchPanelModel.h"
//#include "KeyFramePanel.h"
//#include "KeyFrameModel.h"
#include "core/PathReader.h"

using namespace std;
using namespace cv;

// Constructor
MainWindow::MainWindow()
{
	createWidgets();
	createActions();
	createMenus();
	createStatusBar();
	createToolBars();
	connectWidgets();

}


// Create widgets, actions, menus.

void MainWindow::createWidgets()
{	
	//prevent redrawing???
	//setAttribute(Qt::WA_StaticContents);
	//setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);	


	cloudViewer 	= new CloudWidget;
	coreInterface 	= new CoreInterfaceWidget;
	matchPanel 		= new MatchPanel;
	matchPanelModel = new MatchPanelModel;
	commandBox		= new QLineEdit;
	//keyframePanel 	= new KeyFramePanel;
	//keyframeModel 	= new KeyFrameModel(coreInterface);

	QSplitter *splitter =  new QSplitter;
	splitter->setOrientation(Qt::Horizontal);

	splitter->addWidget(matchPanel);
	//splitter->addWidget(keyframePanel);
	splitter->addWidget(cloudViewer);

	QVBoxLayout  *vl= new QVBoxLayout;
	vl->addWidget(splitter);
	vl->addWidget(commandBox);

	QWidget *centralWidget = new QWidget;
	centralWidget->setLayout(vl);

	setCentralWidget(centralWidget);
	setGeometry(0, 0, 1000, 800);
}

void MainWindow::createActions()
{
    openAction = new QAction(tr("&Open"), this);
    //openAction->setIcon(QIcon(":/images/open.png"));
    openAction->setShortcut(tr("Ctrl+1"));
    openAction->setStatusTip(tr("Open image folder"));
    connect(openAction, SIGNAL(triggered()), this, SLOT(openDirectory()));

    openSavedAction = new QAction(tr("&Open saved"), this);
    openSavedAction->setShortcut(tr("Ctrl+2"));
    openSavedAction->setStatusTip(tr("Open saved project"));
    connect(openSavedAction, SIGNAL(triggered()), this, SLOT(openFile()));


    featureMatchAction = new QAction(tr("&Match"), this);
    featureMatchAction->setStatusTip(tr("Match selected image pair"));
    connect(featureMatchAction, SIGNAL(triggered()), this, SLOT(handleFeatureMatch()));
    //connect(featureMatchAction, SIGNAL(triggered()), matchPanelModel, SLOT(getMatches()));

	reconstructAction = new QAction(tr("&Reconstruct"), this);
	reconstructAction->setStatusTip(tr("Run a reconstruction step"));
	connect(reconstructAction, SIGNAL(triggered()), this, SLOT(handleReconstruct()));
	
	bundleAdjustmentAction = new QAction(tr("&BA"), this);
	bundleAdjustmentAction->setStatusTip(tr("Run bundle adjustment on current cloud"));
	connect(bundleAdjustmentAction, SIGNAL(triggered()), this, SLOT(handleBundleAdjustment()));

	saveAction = new QAction(tr("&Save"),this);
	saveAction->setStatusTip(tr("Save project file and write <pointCloud>.ply"));
	connect(saveAction, SIGNAL(triggered()), this, SLOT(handleSave()));

	nextPairAction = new QAction(tr("&NextPair"),this);
	nextPairAction->setStatusTip(tr("Let the program choose the next pair to match"));
	connect(nextPairAction, SIGNAL(triggered()), this, SLOT(handleNextPair()));

	checkMatchAction = new QAction(tr("&CheckMatch"),this);
	checkMatchAction->setStatusTip(tr("Check quality of match"));
	connect(checkMatchAction, SIGNAL(triggered()), this, SLOT(handleCheckMatch()));

	removeBadAction = new QAction(tr("&RemoveBad"),this);
	removeBadAction->setStatusTip(tr("Remove 3D points with high reprojection error"));
	connect(removeBadAction, SIGNAL(triggered()), this, SLOT(handleRemoveBad()));

	denseAction = new QAction(tr("&DenseReconstruct"),this);
	denseAction->setStatusTip(tr("Dense reconstruction using reovered camera poses"));
	connect(denseAction, SIGNAL(triggered()), this, SLOT(handleDense()));

	//keyframeAction = new QAction(tr("&KeyFrame"),this);
	//keyframeAction->setStatusTip(tr("Compute KeyFrame"));
	//connect(keyframeAction, SIGNAL(triggered()), keyframePanel, SLOT(computeKeyFrame()));

}

void MainWindow::createMenus()
{
    QMenu *fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(openAction);
    fileMenu->addAction(openSavedAction);
}

void MainWindow::createStatusBar()
{
    statusBar()->showMessage(tr(""));
}

void MainWindow::createToolBars()
{
    fileToolBar = addToolBar(tr("File"));
    fileToolBar->addAction(openAction);
    fileToolBar->addAction(openSavedAction);
    
    sfmToolBar = addToolBar(tr("SFM"));

    sfmToolBar->addAction(nextPairAction);
    sfmToolBar->addAction(featureMatchAction);
    sfmToolBar->addAction(checkMatchAction);
    sfmToolBar->addAction(reconstructAction);
	sfmToolBar->addAction(bundleAdjustmentAction);
	sfmToolBar->addAction(removeBadAction);
	sfmToolBar->addAction(saveAction);
	sfmToolBar->addAction(denseAction);

	//ptamToolBar = addToolBar(tr("PTAM"));
	//ptamToolBar->addAction(keyframeAction);

    //helpToolBar->addAction(helpAction);
    //helpToolBar->addAction(aboutAction);
}

void MainWindow::connectWidgets(){
	connect(commandBox, SIGNAL(returnPressed()), this, SLOT(handleLineCommand()));
	connect(cloudViewer, SIGNAL(deletePointIdx(const QList<int>)), this, SLOT(handleDeletePointIdx(const QList<int>)));
	connect(coreInterface, SIGNAL(pointCloudReady(bool)), this, SLOT(displayPointCloud(bool)));
	connect(matchPanelModel, SIGNAL(matchChanged(const QList<QPointF> &, const QList<QPointF> &, const QList<bool> &)), matchPanel, SLOT(updateViews(const QList<QPointF> &, const QList<QPointF> &, const QList<bool> &)));
	connect(coreInterface, SIGNAL(matchResultReady(const QList<QPointF> &, const QList<QPointF> &)), matchPanelModel, SLOT(setMatches(const QList<QPointF> &, const QList<QPointF> &)));
	connect(coreInterface, SIGNAL(nextPairReady(const int, const int)), matchPanel, SLOT(setImagePair(const int, const int)));
	connect(coreInterface, SIGNAL(projectLoaded()), this, SLOT(handleProjectLoaded()));
	connect(matchPanel, SIGNAL(imageChanged(const int, const int)), this, SLOT(highlightPoints(const int, const int)));
	connect(matchPanel, SIGNAL(imageChanged(const int, const int)), matchPanelModel, SLOT(setImages(const int, const int)));
	//connect(keyframePanel, SIGNAL(imageChanged(const int)), keyframeModel, SLOT(setImageIdx(const int)));
	//connect(keyframePanel, SIGNAL(doComputeKeyFrame(const int)), keyframeModel, SLOT(computeKeyFrame(const int)));
	//connect(keyframeModel, SIGNAL(keyFrameCornersReady(const QList<QList<QPointF> > &)), keyframePanel, SLOT(updateCorners(const QList<QList<QPointF> > &)));
}

// Slot functions

void MainWindow::handleProjectLoaded(){
	QString imgRoot;
	QList<QString> imgList;
	coreInterface->getImagePaths(imgRoot, imgList);
	matchPanel->setImagePaths(imgRoot, imgList);
	//keyframePanel->setImagePaths(imgRoot, imgList);
}

void MainWindow::handleFeatureMatch(){

	int idx1, idx2;
	matchPanel->getSelectedImages(idx1, idx2);
	if(idx1==0 || idx2==0){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","1st image or 2nd image cannot be empty!");
		return;
	}
	if(idx1== idx2){
		QMessageBox messageBox;
		messageBox.critical(0,"Error","1st image and 2nd image cannot be the same!");
		return;
	}

	statusBar()->showMessage(tr("matching features..."));
	coreInterface->matchImages(idx1-1, idx2-1);

}
void MainWindow::handleReconstruct(){
	statusBar()->showMessage(tr("reconstructing..."));
	QList<bool> mask;
	matchPanel->getMask(mask);
	coreInterface->reconstruct(mask);
}
void MainWindow::handleBundleAdjustment(){
	statusBar()->showMessage(tr("bundle adjusting..."));
	coreInterface->bundleAdjust();
}
void MainWindow::handleDeletePointIdx(const QList<int> idxs){
	statusBar()->showMessage(tr("deleting points..."));
	coreInterface->deletePointIdx(idxs);
}
void MainWindow::handleSave(){
	statusBar()->showMessage(tr("saving project and writing point cloud to ply..."));
	coreInterface->saveCloud();
}
void MainWindow::handleNextPair(){
	statusBar()->showMessage(tr("auto-selecting the next pair to match..."));
	coreInterface->nextPair();
}
void MainWindow::handleCheckMatch(){
	statusBar()->showMessage(tr("checking match..."));
	QList<bool> mask;
	matchPanel->getMask(mask);
	coreInterface->checkMatch(mask);
}
void MainWindow::handleRemoveBad(){
	statusBar()->showMessage(tr("removing bad points..."));
	coreInterface->removeBad();
}
void MainWindow::handleDense(){
	statusBar()->showMessage(tr("dense reconstructing points..."));
	coreInterface->denseReconstruct();
}
void MainWindow::handleLineCommand(){
	QString s = commandBox->text();
	commandBox->clear();
	QStringList tokens = s.split(' ', QString::SkipEmptyParts);
	if(tokens[0].toLower().compare("overlap")==0){
		cout<<"command show overlapping images"<<endl;
		int baseImgIdx = tokens[1].toInt();
		int camIdx;
		coreInterface->getCameraIdx(baseImgIdx,camIdx);
		if(camIdx>=0){
			displayPointCloud(false);
			map<int,vector<int> > img2pt3Didxs;
			coreInterface->getOverlappingImgs(baseImgIdx,img2pt3Didxs);
			for(map<int,vector<int> >::iterator it=img2pt3Didxs.begin(); it!=img2pt3Didxs.end(); ++it){
				int overlapCamIdx;
				coreInterface->getCameraIdx(it->first,overlapCamIdx);
				cout<<"img["<<it->first<<"] cam["<<overlapCamIdx<<"] sees "<<it->second.size()<<" points from img["<<baseImgIdx<<"] cam["<<camIdx<<"]"<<endl;
				QList<int> idxs;
				for(int i=0; i<it->second.size(); i++){
					idxs.push_back(it->second[i]);
				}
				cloudViewer->highlightPointIdx(idxs, overlapCamIdx);
			}

		}else{
			cout<<"ERROR: image is not used. must select an image that is used."<<endl;
		}
	}else if(tokens[0].toLower().compare("bestoverlap")==0){
		cout<<"command show best overlapping"<<endl;
		int baseImgIdx = tokens[1].toInt();
		int camIdx;
		coreInterface->getCameraIdx(baseImgIdx,camIdx);
		if(camIdx>=0){
			displayPointCloud(false);
			map<int,vector<int> > img2pt3Didxs;
			coreInterface->getBestOverlappingImgs(baseImgIdx,img2pt3Didxs);
			for(map<int,vector<int> >::iterator it=img2pt3Didxs.begin(); it!=img2pt3Didxs.end(); ++it){
				int overlapCamIdx;
				coreInterface->getCameraIdx(it->first,overlapCamIdx);
				cout<<"img["<<it->first<<"] cam["<<overlapCamIdx<<"] sees "<<it->second.size()<<" points from img["<<baseImgIdx<<"] cam["<<camIdx<<"]"<<endl;
				QList<int> idxs;
				for(int i=0; i<it->second.size(); i++){
					idxs.push_back(it->second[i]);
				}
				cloudViewer->highlightPointIdx(idxs, overlapCamIdx);
			}
		}else{
			cout<<"ERROR: image is not used. must select an image that is used."<<endl;
		}
	}else if(tokens[0].toLower().compare("highlight")==0){
		int cnt = tokens.size() - 1;
		cout<<"command highlighing "<<cnt<<" cameras"<<endl;
		displayPointCloud(false);
		for(int i=1; i<tokens.size(); i++){
			int imgIdx = tokens[i].toInt();
			int camIdx;
			coreInterface->getCameraIdx(imgIdx,camIdx);
			if(camIdx>=0){
				vector<Point3f> xyzs;
				vector<int>		highlightIdxs;
				coreInterface->getAll3DfromImage2D(imgIdx,xyzs,highlightIdxs);
				QList<int> idxs;
				for(int i=0; i<highlightIdxs.size(); i++){
					idxs.push_back(highlightIdxs[i]);
				}
				cloudViewer->highlightPointIdx(idxs, camIdx);
			}
		}
	}
	/*
	std::vector<double> vals;
	for(int i=0; i<tokens.size(); i++){
		vals.push_back(tokens.at(i).toDouble());
	}
	for(vector<double>::iterator it=vals.begin(); it!=vals.end(); ++it){
		cout<<(*it)<<" ";
	}
	cout<<endl;
	coreInterface->ApplyGlobalTransformationToMap(vals);
	*/

}
void MainWindow::displayPointCloud(bool resetView){
	statusBar()->showMessage(tr("loading cloud..."));
	vector<Point3f> xyzs;
	coreInterface->getPointCloud(xyzs);
	vector<Matx34d> cams;
	coreInterface->getCameras(cams);
	//cloudViewer->loadCloud(xyzs);
	cloudViewer->loadCloudAndCamera(xyzs,cams, resetView);
	statusBar()->showMessage(tr("cloud loaded"));
	
	//also update imgList2 to include only used images
	vector<int>	useImgIdxs;
	coreInterface->getUsedImageIdxs(useImgIdxs);
	matchPanel->handleImageUsed(useImgIdxs);

}

void MainWindow::highlightPoints(const int imgIdx1, const int imgIdx2){
	displayPointCloud(false);	//to reset previous highlights just reload the point cloud

	int camIdx1, camIdx2;
	coreInterface->getCameraIdx(imgIdx1,camIdx1);
	coreInterface->getCameraIdx(imgIdx2,camIdx2);
	if(camIdx1>=0){
		vector<Point3f> xyzs;
		vector<int>		highlightIdxs;
		coreInterface->getAll3DfromImage2D(imgIdx1,xyzs,highlightIdxs);
		QList<int> idxs;
		for(int i=0; i<highlightIdxs.size(); i++){
			idxs.push_back(highlightIdxs[i]);
		}
		cloudViewer->highlightPointIdx(idxs, camIdx1);
	}
	if(camIdx2>=0){
		vector<Point3f> xyzs;
		vector<int>		highlightIdxs;
		coreInterface->getAll3DfromImage2D(imgIdx2,xyzs,highlightIdxs);
		QList<int> idxs;
		for(int i=0; i<highlightIdxs.size(); i++){
			idxs.push_back(highlightIdxs[i]);
		}
		cloudViewer->highlightPointIdx(idxs, camIdx2);
	}

}
void MainWindow::openFile()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Project"), ".",
		tr("YAML files (*.yaml)\nNVM files (*.nvm)"));

    if (!fileName.isEmpty() && loadFile(fileName)==0){
    	coreInterface -> loadProject(fileName);
    }
}
void MainWindow::openDirectory(){

	 QString dir = QFileDialog::getExistingDirectory(this,
			 tr("Open Directory"), ".",
			 QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

	 if(dir==""){
		 return;
	 }
	 imageRoot = dir;
	 cout<<dir.toStdString()<<endl;
	 vector<string> sortedImageList;
	 //PathReader::readPaths(dir.toStdString(),".jpg", sortedImageList);
	 PathReader::readPaths(dir.toStdString(),".JPG", sortedImageList);
	 //PathReader::readPaths(dir.toStdString(),".png", sortedImageList);
	 QList<QString> imageList;
	 imageList.reserve(sortedImageList.size());
	 for(int i=0; i<sortedImageList.size(); i++){
		 imageList.push_back(QString::fromStdString(sortedImageList[i]));
	 }

	 coreInterface 	->setImagePaths(imageRoot, imageList);
	 matchPanel		->setImagePaths(imageRoot, imageList);
	 //keyframePanel 	->setImagePaths(imageRoot, imageList);

}

int MainWindow::loadFile(const QString &fileName)
{
    QFile file(fileName);
    if (!file.open(QFile::ReadOnly | QFile::Text))
    {
        QMessageBox::warning(this, tr("CloudWidget"),
            tr("Cannot read file %1:\n%2.")
            .arg(fileName).arg(file.errorString()));
        return 1;
    }
    return 0;
}


