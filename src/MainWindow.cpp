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
#include "VisibleImagesPanel.h"
#include "MatchPanelModel.h"
#include "KeyFramePanel.h"
#include "KeyFrameModel.h"
#include "core/PathReader.h"
#include "core/Utils.h"

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


	cloudViewer 		= new CloudWidget;
	cloudViewer2		= new CloudWidget;
	coreInterface 		= new CoreInterfaceWidget;
	matchPanel 			= new MatchPanel;
	visibleImagesPanel 	= new VisibleImagesPanel;
	matchPanelModel 	= new MatchPanelModel;
	commandBox			= new QLineEdit;
	keyframePanel 		= new KeyFramePanel;
	keyframeModel 		= new KeyFrameModel(coreInterface);

	tabs				= new QTabWidget;
	tabs->addTab(matchPanel, tr("MatchPanel"));
	tabs->addTab(visibleImagesPanel, tr("VisibleImages"));
	tabs->addTab(keyframePanel, tr("SurfaceProjection"));

	tabs2				= new QTabWidget;
	tabs2->addTab(cloudViewer, tr("CloudViewer1"));
	tabs2->addTab(cloudViewer2, tr("CloudViewer2"));

	QSplitter *splitter =  new QSplitter;
	splitter->setOrientation(Qt::Horizontal);

	splitter->addWidget(tabs);
	//splitter->addWidget(keyframePanel);
	splitter->addWidget(tabs2);

	QVBoxLayout  *vl= new QVBoxLayout;
	vl->addWidget(splitter);
	vl->addWidget(commandBox);

	QWidget *centralWidget = new QWidget;
	centralWidget->setLayout(vl);

	setCentralWidget(centralWidget);
	setGeometry(0, 0, 1000, 800);

	keyframeSelectedImgIdx = -1;
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


    openGPSAction = new QAction(tr("&Open GPS"), this);
    openGPSAction->setStatusTip(tr("Open GPS file"));
	connect(openGPSAction, SIGNAL(triggered()), this, SLOT(openGPSFile()));

	openPolygonAction = new QAction(tr("&Open Polygon"), this);
	openPolygonAction->setStatusTip(tr("Open Polygon file"));
	connect(openPolygonAction, SIGNAL(triggered()), this, SLOT(openPolygonFile()));

	saveAction = new QAction(tr("&Save"),this);
	saveAction->setShortcut(tr("Ctrl+S"));
	saveAction->setStatusTip(tr("Save project file"));
	//connect(saveAction, SIGNAL(triggered()), this, SLOT(handleSave()));
	connect(saveAction, SIGNAL(triggered()), this, SLOT(saveFileAs()));

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

	nextPairAction = new QAction(tr("&NextPair"),this);
	nextPairAction->setStatusTip(tr("Let the program choose the next pair to match"));
	connect(nextPairAction, SIGNAL(triggered()), this, SLOT(handleNextPair()));

	checkMatchAction = new QAction(tr("&CheckMatch"),this);
	checkMatchAction->setStatusTip(tr("Check quality of match"));
	connect(checkMatchAction, SIGNAL(triggered()), this, SLOT(handleCheckMatch()));

	removeBadAction = new QAction(tr("&RemoveBad"),this);
	removeBadAction->setStatusTip(tr("Remove 3D points with high reprojection error"));
	connect(removeBadAction, SIGNAL(triggered()), this, SLOT(handleRemoveBad()));

	removeCameraAction = new QAction(tr("&RemoveCamera"),this);
	removeCameraAction->setStatusTip(tr("Remove image selected in matchpanel if the image is used"));
	connect(removeCameraAction, SIGNAL(triggered()), this, SLOT(handleDeleteCamera()));

	minSpanCamerasAction = new QAction(tr("&MinSpan"),this);
	minSpanCamerasAction->setStatusTip(tr("Keep minimum cameras covering all points"));
	connect(minSpanCamerasAction, SIGNAL(triggered()), this, SLOT(handleMinSpanCamera()));

	denseAction = new QAction(tr("&DenseReconstruct"),this);
	denseAction->setStatusTip(tr("Dense reconstruction using reovered camera poses"));
	connect(denseAction, SIGNAL(triggered()), this, SLOT(handleDense()));

	addKeyFrameAction = new QAction(tr("&Add Keyframe"),this);
	addKeyFrameAction->setStatusTip(tr("Add Keyframe to cloud viewer 2"));
	connect(addKeyFrameAction, SIGNAL(triggered()), this, SLOT(handleAddKeyFrame()));

	//keyframeAction = new QAction(tr("&KeyFrame"),this);
	//keyframeAction->setStatusTip(tr("Compute KeyFrame"));
	//connect(keyframeAction, SIGNAL(triggered()), keyframePanel, SLOT(computeKeyFrame()));

	renderNormalToggleAction = new QAction(tr("&Show/Hide Normal"),this);
	showNormal = false;
	renderNormalToggleAction->setStatusTip(tr("Show/Hide Normal"));
	connect(renderNormalToggleAction, SIGNAL(triggered()), this, SLOT(handleNormalRenderToggle()));

	renderPolygonToggleAction= new QAction(tr("&Show/Hide Polygon"),this);
	showPolygon = false;
	renderPolygonToggleAction->setStatusTip(tr("Show/Hide Polygon"));
	connect(renderPolygonToggleAction, SIGNAL(triggered()), this, SLOT(handlePolygonRenderToggle()));

}

void MainWindow::createMenus()
{
    QMenu *fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(openAction);
    fileMenu->addAction(openSavedAction);
    fileMenu->addAction(openGPSAction);
    fileMenu->addAction(openPolygonAction);
    fileMenu->addAction(saveAction);
    QMenu *optionMenu = menuBar()->addMenu(tr("&Option"));
    optionMenu->addAction(renderNormalToggleAction);
    optionMenu->addAction(renderPolygonToggleAction);
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
    fileToolBar->addAction(saveAction);
    
    sfmToolBar = addToolBar(tr("SFM"));

    sfmToolBar->addAction(nextPairAction);
    sfmToolBar->addAction(featureMatchAction);
    sfmToolBar->addAction(checkMatchAction);
    sfmToolBar->addAction(reconstructAction);
	sfmToolBar->addAction(bundleAdjustmentAction);
	sfmToolBar->addAction(removeBadAction);
	sfmToolBar->addAction(removeCameraAction);
	sfmToolBar->addAction(minSpanCamerasAction);
	sfmToolBar->addAction(addKeyFrameAction);
	sfmToolBar->addAction(denseAction);


	//ptamToolBar = addToolBar(tr("PTAM"));
	//ptamToolBar->addAction(keyframeAction);

    //helpToolBar->addAction(helpAction);
    //helpToolBar->addAction(aboutAction);
}

void MainWindow::connectWidgets(){
	connect(commandBox, SIGNAL(returnPressed()), this, SLOT(handleLineCommand()));
	connect(cloudViewer, SIGNAL(deletePointIdx(const QList<int>)), this, SLOT(handleDeletePointIdx(const QList<int>)));
	connect(cloudViewer2, SIGNAL(deletePointIdx(const QList<int>)), this, SLOT(handleDeletePointIdx2(const QList<int>)));
	connect(cloudViewer, SIGNAL(showCamerasSeeingPoints(const QList<int>)), this, SLOT(handleShowCamerasSeeingPoints(const QList<int>)));
	connect(coreInterface, SIGNAL(pointCloudReady(bool)), this, SLOT(displayPointCloud(bool)));
	connect(coreInterface, SIGNAL(pointCloud2Ready(bool)), this, SLOT(displayPointCloud2(bool)));
	connect(coreInterface, SIGNAL(polygonReady(bool)), this, SLOT(displayPolygon(bool)));
	connect(matchPanelModel, SIGNAL(matchChanged(const QList<QPointF> &, const QList<QPointF> &, const QList<bool> &)), matchPanel, SLOT(updateViews(const QList<QPointF> &, const QList<QPointF> &, const QList<bool> &)));
	connect(coreInterface, SIGNAL(matchResultReady(const QList<QPointF> &, const QList<QPointF> &)), matchPanelModel, SLOT(setMatches(const QList<QPointF> &, const QList<QPointF> &)));
	connect(coreInterface, SIGNAL(nextPairReady(const int, const int)), matchPanel, SLOT(setImagePair(const int, const int)));
	connect(coreInterface, SIGNAL(projectLoaded()), this, SLOT(handleProjectLoaded()));
	connect(matchPanel, SIGNAL(imageChanged(const int, const int)), this, SLOT(highlightPoints(const int, const int)));
	connect(matchPanel, SIGNAL(imageChanged(const int, const int)), matchPanelModel, SLOT(setImages(const int, const int)));
	connect(visibleImagesPanel, SIGNAL(deleteSelectedMeasures(const QList<QPair<int,int> > &)), this, SLOT(handleDeleteMeasures(const QList<QPair<int,int> > &)));
	connect(keyframePanel, SIGNAL(imageChanged(const int)), keyframeModel, SLOT(setImageIdx(const int)));
	connect(keyframePanel, SIGNAL(doComputeKeyFrame(const int)), keyframeModel, SLOT(computeKeyFrame(const int)));
	connect(keyframeModel, SIGNAL(keyFrameCornersReady(const QList<QList<QPointF> > &)), keyframePanel, SLOT(updateCorners(const QList<QList<QPointF> > &)));
	connect(keyframePanel, SIGNAL(imagePointsSelected(const int, const QList<QPointF> &)), this, SLOT(handleKeyframeImagePointsSelected(const int, const QList<QPointF> &)));
	connect(keyframePanel, SIGNAL(imageChanged(const int)), this, SLOT(handleKeyFramePanelImageChange(const int)));

}

// Slot functions

void MainWindow::handleProjectLoaded(){
	QString imgRoot;
	QList<QString> imgList;
	coreInterface->getImagePaths(imgRoot, imgList);
	matchPanel->setImagePaths(imgRoot, imgList);
	visibleImagesPanel->setImagePaths(imgRoot, imgList);
	keyframePanel->setImagePaths(imgRoot, imgList);
}

void MainWindow::handleFeatureMatch(){

	int idx1, idx2;
	matchPanel->getSelectedImages(idx1, idx2);
	if(idx1<0 || idx2<0){
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
	coreInterface->matchImages(idx1, idx2);

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
	coreInterface->deletePointIdx(idxs, 0);
}
void MainWindow::handleDeletePointIdx2(const QList<int> idxs){
	statusBar()->showMessage(tr("deleting points..."));
	coreInterface->deletePointIdx(idxs, 1);
}
void MainWindow::handleShowCamerasSeeingPoints(const QList<int> idxs){
	statusBar()->showMessage(tr("showing cameras..."));
	displayPointCloud(false);
	vector<int> pt3DIdxs;
	pt3DIdxs.reserve(idxs.size());
	for(int i=0; i<idxs.size(); i++){
		pt3DIdxs.push_back(idxs[i]);
	}
	vector<vector<pair<int,int> > > pt3D2Measures;
	vector<vector<QPointF> > pt3D2pt2Ds;
	coreInterface->getMeasuresToPoints(pt3DIdxs,pt3D2Measures,pt3D2pt2Ds);

	//hihglight 3D
	for(int i=0; i<pt3D2Measures.size(); i++){
		for(int j=0; j<pt3D2Measures[i].size(); j++){
			int imgIdx = pt3D2Measures[i][j].first;
			int camIdx;
			coreInterface->getCameraIdx(imgIdx,camIdx);
			QList<int> ptIdxs;
			ptIdxs.push_back(pt3DIdxs[i]);
			cloudViewer->highlightPointIdx(ptIdxs, camIdx);
		}
	}

	visibleImagesPanel->setVisibleImagesAndMeasures(pt3D2Measures, pt3D2pt2Ds);
/*
	QMap<int, QList<QPointF> > 	img2pt2Ds;
	QMap<int, QList<int> >		img2pt2DIdxs;
	for(int i=0; i<pt3D2Measures.size(); i++){
		for(int j=0; j<pt3D2Measures[i].size(); j++){
			int imgIdx 	= pt3D2Measures[i][j].first;
			int pt2DIdx = pt3D2Measures[i][j].second;
			if(img2pt2Ds.find(imgIdx) == img2pt2Ds.end()){
				img2pt2Ds[imgIdx] = QList<QPointF>();
				img2pt2DIdxs[imgIdx] = QList<int>();
			}
			img2pt2Ds[imgIdx].push_back(pt3D2pt2Ds[i][j]);
			img2pt2DIdxs[imgIdx].push_back(pt2DIdx);
		}
	}
	visibleImagesPanel->setVisibleImagesAndMeasures(img2pt2Ds,img2pt2DIdxs);*/
}
//void MainWindow::handleSave(){
//	statusBar()->showMessage(tr("saving project and writing point cloud to ply..."));
//	coreInterface->saveCloud();
//}
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

void MainWindow::handleDeleteCamera(){
	statusBar()->showMessage(tr("deleting selected camera(s)..."));
	int imgIdx1, imgIdx2;
	matchPanel->getSelectedImages(imgIdx1, imgIdx2);
	vector<int> imgIdxs;
	imgIdxs.push_back(imgIdx1);
	imgIdxs.push_back(imgIdx2);
	coreInterface->deleteCameraByImageIdxs(imgIdxs);
}
void MainWindow::handleDeleteMeasures(const QList<QPair<int,int> > &measures){
	statusBar()->showMessage(tr("deleting measures..."));
	coreInterface->deleteMeasures(measures);
	QList<int> camIdxs;
	handleShowCamerasSeeingPoints(camIdxs);
}
void MainWindow::handleMinSpanCamera(){
	statusBar()->showMessage(tr("finding minimum span camera(s)..."));
	coreInterface->keepMinSpanCameras();
}
void MainWindow::handleDense(){
	statusBar()->showMessage(tr("dense reconstructing points..."));
	coreInterface->denseReconstruct();
}

void MainWindow::handleKeyframeImagePointsSelected(const int imgIdx, const QList<QPointF> &pts){
	assert(keyframeSelectedImgIdx == imgIdx);
	statusBar()->showMessage(tr("project points to 3d surface..."));
	QList<QVector3D> xyzs;
	QList<QVector3D> norms;
	vector<bool> status;
	coreInterface->projectImagePointsTo3DSurface(imgIdx,pts,xyzs,norms,status);

	QList<QVector3D> intersects;
	for(int i=0; i<xyzs.size(); i++){
		if(status[i]){
			intersects.push_back(xyzs[i]);
		}
	}

	//to reset previous highlights
	if(showPolygon){
		displayPolygon(false);
	}else{
		displayPointCloud(false);
	}

	int camIdx;
	coreInterface->getCameraIdx(imgIdx,camIdx);
	cloudViewer->highlightPoints(intersects, camIdx);
	cout<<intersects.size()<<" intersections found"<<endl;

	mxys 	= pts;
	mxyzs	= xyzs;
	mnorms	= norms;
	mstatus = status;

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
	}else if(tokens[0].toLower().compare("transform")==0){
		std::vector<double> vals;
		for(int i=1; i<tokens.size(); i++){
			vals.push_back(tokens.at(i).toDouble());
		}
		for(vector<double>::iterator it=vals.begin(); it!=vals.end(); ++it){
			cout<<(*it)<<" ";
		}
		cout<<endl;
		coreInterface->ApplyGlobalTransformation(vals);
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
	statusBar()->showMessage(tr("refreshing cloud..."));
	vector<Point3f> xyzs, norms;
	coreInterface->getPointCloud(xyzs);
	if(showNormal){
		coreInterface->getPointNormals(norms);
	}
	vector<Matx34d> cams;
	coreInterface->getCameras(cams);
	//cloudViewer->loadCloud(xyzs);
	cloudViewer->loadCloudAndCamera(xyzs, norms, cams, resetView);
	statusBar()->showMessage(tr("cloud refreshed"));
	
	//also update match panel image lists to highlight images
	vector<int>	useImgIdxs;
	coreInterface->getUsedImageIdxs(useImgIdxs);
	matchPanel->handleImagesUsed(useImgIdxs);
	keyframePanel->handleImagesUsed(useImgIdxs);
	statusBar()->showMessage(tr("image list updated"));

}
void MainWindow::displayPointCloud2(bool resetView){
	statusBar()->showMessage(tr("refreshing cloud2..."));
	cout<<"display pointcloud 2"<<endl;
	vector<Point3f> xyzs, norms;
	coreInterface->getPointCloud2(xyzs);
	if(showNormal){
		coreInterface->getPointNormals2(norms);
	}
	vector<Matx34d> cams;
	coreInterface->getCameras2(cams);
	cloudViewer2->loadCloudAndCamera(xyzs, norms, cams, resetView);
	statusBar()->showMessage(tr("cloud2 refreshed"));

	//also update keyframe image lists to highlight images
	vector<int>	useImgIdxs;
	coreInterface->getUsedImageIdxs(useImgIdxs);
	keyframePanel->handleImagesUsed(useImgIdxs);
	coreInterface->getUsedImageIdxs2(useImgIdxs);
	keyframePanel->handleImagesUsed2(useImgIdxs);
	statusBar()->showMessage(tr("image list updated"));

}
void MainWindow::displayPolygon(bool resetView){
	showPolygon = true;
	statusBar()->showMessage(tr("refreshing polygon..."));
	vector<Point3f> verts;
	vector<Point3i> faces;
	//coreInterface->getPolygons(verts, faces);

	coreInterface->getVisiblePolygons(keyframeSelectedImgIdx, verts, faces); //
	vector<Matx34d> cams;
	coreInterface->getCameras(cams);
	cloudViewer->loadPolygonAndCamera(verts, faces, cams, resetView);
	statusBar()->showMessage(tr("polygon refreshed"));
}
void MainWindow::handleKeyFramePanelImageChange(const int imgIdx){
	keyframeSelectedImgIdx = imgIdx-1;
	projectPolygonToImage(keyframeSelectedImgIdx);
	displayPolygon(false);

}
void MainWindow::handleNormalRenderToggle(){
	showNormal = !showNormal;
	int cloudViewerIdx = tabs2->currentIndex();
	if(cloudViewerIdx == 0){
		displayPointCloud(false);
	}else if(cloudViewerIdx == 1){
		displayPointCloud2(false);
	}
}
void MainWindow::handlePolygonRenderToggle(){
	showPolygon = !showPolygon;
	if(showPolygon){
		displayPolygon(false);
	}else{
		displayPointCloud(false);
	}
}
void MainWindow::handleAddKeyFrame(){
	statusBar()->showMessage(tr("adding keyframe..."));
	cout<<"adding img["<<keyframeSelectedImgIdx<<"] as keyframe"<<endl;
	coreInterface->addKeyFrame(keyframeSelectedImgIdx, mxys, mxyzs, mnorms, mstatus);
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
void MainWindow::projectPolygonToImage(int imgIdx)
{
	statusBar()->showMessage(tr("project polygon to image..."));
	QList<QPointF>	verts;
	coreInterface ->projectPolygonToImage(imgIdx, verts);
	cout<<"projected faces = "<<verts.size()/3<<endl;
	keyframePanel ->drawProjection(verts);

}
void MainWindow::openFile()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Project"), "/home/yoyo/Desktop/data/save",
		tr("YAML files (*.yaml)\nNVM files (*.nvm)\nPATCH files (*.patch)\nTINY files (*.tiny)"));

    if (!fileName.isEmpty() && loadFile(fileName)==0){
    	coreInterface -> loadProject(fileName, tabs2->currentIndex());
    }
}
void MainWindow::openGPSFile(){
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open GPS File"), "/home/yoyo/Desktop/data/gps",
		tr("CSV files (*.csv)"));

	if (!fileName.isEmpty() && loadFile(fileName)==0){
		coreInterface -> loadGPS(fileName);
	}
}
void MainWindow::openPolygonFile(){
	QString fileName = QFileDialog::getOpenFileName(this,
		tr("Open Polygon File"), "/home/yoyo/Desktop/data/polygons",
		tr("PLY files (*.ply)"));

	if (!fileName.isEmpty() && loadFile(fileName)==0){
		coreInterface -> loadPolygon(fileName);
	}
}
void MainWindow::openDirectory(){

	 QString dir = QFileDialog::getExistingDirectory(this,
			 tr("Open Directory"), "/home/yoyo/Desktop/data/pics",
			 QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

	 if(dir==""){
		 return;
	 }
	 imageRoot = dir;
	 cout<<dir.toStdString()<<endl;
	 vector<string> sortedImageList;
	 PathReader::readPaths(dir.toStdString(),sortedImageList);
	 QList<QString> imageList;
	 imageList.reserve(sortedImageList.size());
	 for(int i=0; i<sortedImageList.size(); i++){
		 imageList.push_back(QString::fromStdString(sortedImageList[i]));
	 }

	 coreInterface 	->setImagePaths(imageRoot, imageList);
	 matchPanel		->setImagePaths(imageRoot, imageList);
	 visibleImagesPanel		->setImagePaths(imageRoot, imageList);
	 keyframePanel 	->setImagePaths(imageRoot, imageList);

}

void MainWindow::saveFileAs(){

	string tstr;
	Utils::getTimeStampAsString(tstr);
	QFileDialog fd(this);
	QString ext;
	QString fileName = fd.getSaveFileName(this,
	        tr("Save Project"), "/home/yoyo/Desktop/data/save",
	        tr("TINY files (*.tiny)\nPLY files (*.ply)\nYAML files (*.yaml)\nNVM files (*.nvm)\nSKTXT files (*.sktxt)"), &ext);

	if (!fileName.isEmpty()){
		if(ext=="TINY files (*.tiny)"){
			fileName+=".tiny";
		}else if(ext=="PLY files (*.ply)"){
			fileName+=".ply";
		}else if(ext=="YAML files (*.yaml)"){
			fileName+=".yaml";
		}else if(ext=="NVM files (*.nvm)"){
			fileName+=".nvm";
		}else if(ext=="SKTXT files (*.sktxt)"){
			fileName+=".sktxt";
		}
		cout<<"saving file: "<<fileName.toStdString()<<endl;
		coreInterface -> saveProject(fileName, tabs2->currentIndex());
	}

}

int MainWindow::loadFile(const QString &fileName)
{
    QFile file(fileName);
    if (!file.open(QFile::ReadOnly | QFile::Text))
    {
        QMessageBox::warning(this, tr("ManualSFM"),
            tr("Cannot read file %1:\n%2.")
            .arg(fileName).arg(file.errorString()));
        return 1;
    }
    return 0;
}


