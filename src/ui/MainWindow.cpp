/*
 * Main UI
 */

#include <QtWidgets>
// #include <QFileDialog>
// #include <QMessageBox>
// #include <QStatusBar>
// #include <QLineEdit>
// #include <QSplitter>
// #include <QToolBar>
// #include <QMenu>
// #include <QMenuBar>
// #include <QLayout>
#include <opencv2/core/core.hpp>

#include <iostream>

#include "MainWindow.h"
#include "CloudWidget.h"
#include "CoreInterfaceWidget.h"
#include "ImageWidget.h"
#include "MatchWidget.h"
#include "MatchPanel.h"
#include "VisibleImagesPanel.h"
#include "EpipolarPanel.h"
// #include "VideoPanel.h"
// #include "FramePanel.h"
// #include "GraphWidget.h"
#include "MatchPanelModel.h"
#include "core/Utils.h"

#include "core/datastructs/Data.h"
// #include "core/SFMVideoPipeline.h"

#include <Eigen/Eigen>

using namespace std;
using namespace cv;
using namespace Eigen;
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
	coreInterface 		= new CoreInterfaceWidget;
	matchPanel 			= new MatchPanel;
	visibleImagesPanel 	= new VisibleImagesPanel;
	matchPanelModel 	= new MatchPanelModel;
	epipolarPanel		= new EpipolarPanel;
	commandBox			= new QLineEdit;

	tabs				= new QTabWidget;
	tabs->addTab(matchPanel, tr("Feature Match"));
	tabs->addTab(visibleImagesPanel, tr("Back Projections"));
	tabs->addTab(epipolarPanel, tr("EpipolarPanel"));

	tabs2				= new QTabWidget;
	tabs2->addTab(cloudViewer, tr("3D"));

	QSplitter *splitter =  new QSplitter;
	splitter->setOrientation(Qt::Horizontal);

	splitter->addWidget(tabs);
	splitter->addWidget(tabs2);

	splitter->setSizes(QList<int>() << 500 << 500);

	QVBoxLayout  *vl= new QVBoxLayout;
	vl->addWidget(splitter);
	// vl->addWidget(commandBox);

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

	saveAction = new QAction(tr("&Save"),this);
	saveAction->setShortcut(tr("Ctrl+S"));
	saveAction->setStatusTip(tr("Save project file"));
	//connect(saveAction, SIGNAL(triggered()), this, SLOT(handleSave()));
	connect(saveAction, SIGNAL(triggered()), this, SLOT(saveFileAs()));

	// saveFeaturesAction = new QAction(tr("&SaveFeas"),this);
	// saveFeaturesAction->setStatusTip(tr("Save features to .sift file"));
	// connect(saveFeaturesAction, SIGNAL(triggered()), this, SLOT(handleSaveFeatures()));

    featureMatchAction = new QAction(tr("&Match"), this);
    featureMatchAction->setStatusTip(tr("Match selected image pair"));
    connect(featureMatchAction, SIGNAL(triggered()), this, SLOT(handleFeatureMatch()));
    //connect(featureMatchAction, SIGNAL(triggered()), matchPanelModel, SLOT(getMatches()));

    epipolarMatchAction = new QAction(tr("&EMatch"), this);
    epipolarMatchAction->setStatusTip(tr("Match selected image pair using epipolar search"));
    connect(epipolarMatchAction, SIGNAL(triggered()), this, SLOT(handleEpipolarFeatureMatch()));

	reconstructAction = new QAction(tr("&Reconstruct"), this);
	reconstructAction->setStatusTip(tr("Run a reconstruction step"));
	connect(reconstructAction, SIGNAL(triggered()), this, SLOT(handleReconstruct()));
	
	addMoreMeasuresAction = new  QAction(tr("&MoreMeasures"), this);
	addMoreMeasuresAction->setStatusTip(tr("Add more measurements to visible landmarks"));
	connect(addMoreMeasuresAction, SIGNAL(triggered()), this, SLOT(handleAddMoreMeasures()));

	addMorePointsAction = new QAction(tr("&MorePoints"), this);
	addMorePointsAction->setStatusTip(tr("Add more points & measures using epipolar search"));
	connect(addMorePointsAction, SIGNAL(triggered()), this, SLOT(handleAddMorePoints()));

	poseOptimizationAction = new QAction(tr("&Optimize"), this);
	poseOptimizationAction->setStatusTip(tr("Optimize current camera pose using all measurements"));
	connect(poseOptimizationAction, SIGNAL(triggered()), this, SLOT(handlePoseOptimization()));

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

}

void MainWindow::createMenus()
{
    QMenu *fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(openAction);
    fileMenu->addAction(openSavedAction);
    fileMenu->addAction(saveAction);
    // fileMenu->addAction(saveFeaturesAction);
}

void MainWindow::createStatusBar()
{
    statusBar()->showMessage(tr(""));
}

void MainWindow::createToolBars()
{
    // fileToolBar = addToolBar(tr("File"));
    // fileToolBar->addAction(openAction);
    // fileToolBar->addAction(openSavedAction);
    // fileToolBar->addAction(saveAction);
    // // fileToolBar->addAction(saveFeaturesAction);
    
    // addToolBarBreak();
    sfmToolBar = addToolBar(tr("SFM"));

    sfmToolBar->addAction(nextPairAction);
    sfmToolBar->addAction(featureMatchAction);
    // sfmToolBar->addAction(epipolarMatchAction);
    // sfmToolBar->addAction(checkMatchAction);
    sfmToolBar->addAction(reconstructAction);
    // sfmToolBar->addAction(addMoreMeasuresAction);
    sfmToolBar->addAction(addMorePointsAction);
    // sfmToolBar->addAction(poseOptimizationAction);
	sfmToolBar->addAction(bundleAdjustmentAction);
	sfmToolBar->addAction(removeBadAction);
	// sfmToolBar->addAction(removeCameraAction);

    //helpToolBar->addAction(helpAction);
    //helpToolBar->addAction(aboutAction);
}

void MainWindow::connectWidgets(){
	//file operations
	connect(coreInterface, SIGNAL(projectLoaded()), this, SLOT(handleProjectLoaded()));

	connect(matchPanelModel, SIGNAL(matchChanged(const QList<QPointF> &, const QList<QPointF> &, const QList<bool> &)), matchPanel, SLOT(updateViews(const QList<QPointF> &, const QList<QPointF> &, const QList<bool> &)));
	connect(coreInterface, SIGNAL(matchResultReady(const QList<QPointF> &, const QList<QPointF> &)), matchPanelModel, SLOT(setMatches(const QList<QPointF> &, const QList<QPointF> &)));
	connect(coreInterface, SIGNAL(nextPairReady(const int, const int)), matchPanel, SLOT(setImagePair(const int, const int)));

	connect(matchPanel, SIGNAL(imageChanged(const int, const int)), this, SLOT(highlightPoints(const int, const int)));
	connect(matchPanel, SIGNAL(imageChanged(const int, const int)), matchPanelModel, SLOT(setImages(const int, const int)));

	//3D visualization
	connect(coreInterface, SIGNAL(pointCloudReady(bool)), 						this, SLOT(displayPointCloud(bool)));
	connect(cloudViewer, SIGNAL(requestDeletePoints(const QList<int>)), 		this, SLOT(handleDeletePoints(const QList<int>)));
	connect(cloudViewer, SIGNAL(requestShowPointsMeasures(const QList<int>)), 	this, SLOT(handleShowPointsMeasures(const QList<int>)));
	connect(cloudViewer, SIGNAL(requestShowAllProjections()), 	visibleImagesPanel, SLOT(showProjections()));
	connect(commandBox, SIGNAL(returnPressed()), 								this, SLOT(handleLineCommand()));

}

// Slot functions
void MainWindow::handleNextPair(){
	statusBar()->showMessage(tr("auto-selecting the next pair to match..."));
	coreInterface->nextPair();
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

void MainWindow::handleEpipolarFeatureMatch(){
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
	statusBar()->showMessage(tr("epipolar matching features..."));
	coreInterface->matchImagesEpipolar(idx1, idx2);
}

void MainWindow::handleCheckMatch(){
	statusBar()->showMessage(tr("checking match..."));
	QList<bool> mask;
	matchPanel->getMask(mask);
	int imgIdx1, imgIdx2;
	matchPanel->getSelectedImages(imgIdx1, imgIdx2);
	coreInterface->checkMatch(imgIdx1, imgIdx2, mask);
}

void MainWindow::handleReconstruct(){
	statusBar()->showMessage(tr("reconstructing..."));
	QList<bool> mask;
	matchPanel->getMask(mask);
	int imgIdx1, imgIdx2;
	matchPanel->getSelectedImages(imgIdx1, imgIdx2);
	coreInterface->reconstruct(imgIdx1, imgIdx2, mask);
}

void MainWindow::handleAddMoreMeasures(){
	statusBar()->showMessage(tr("adding more measures..."));
	int imgIdx1, imgIdx2;
	matchPanel->getSelectedImages(imgIdx1, imgIdx2);
	coreInterface->addMoreMeasures(imgIdx1);
}

void MainWindow::handleAddMorePoints(){
	statusBar()->showMessage(tr("adding more points..."));
	int imgIdx1, imgIdx2;
	matchPanel->getSelectedImages(imgIdx1, imgIdx2);
	coreInterface->addMorePoints(imgIdx1, imgIdx2);
}

void MainWindow::handlePoseOptimization(){
	statusBar()->showMessage(tr("not implemented"));
	// statusBar()->showMessage(tr("optimizing pose..."));
	// int imgIdx1, imgIdx2;
	// matchPanel->getSelectedImages(imgIdx1, imgIdx2);
	// coreInterface->poseOptimization(imgIdx1);
}

void MainWindow::handleBundleAdjustment(){
	statusBar()->showMessage(tr("bundle adjusting..."));
	coreInterface->bundleAdjust();
}

void MainWindow::handleRemoveBad(){
	statusBar()->showMessage(tr("removing bad points..."));
	coreInterface->removeBad();
}

void MainWindow::handleDeletePoints(const QList<int> idxs){
	statusBar()->showMessage(tr("deleting points..."));
	coreInterface->deletePointIdx(idxs, 0);
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

void MainWindow::displayPointCloud(bool resetView){
	statusBar()->showMessage(tr("refreshing cloud..."));

	cloudViewer->visualize(resetView);
	statusBar()->showMessage(tr("cloud refreshed"));

	//also update match panel image lists to highlight images
	vector<int>	useImgIdxs;
	coreInterface->getUsedImageIdxs(useImgIdxs);
	matchPanel->handleImagesUsed(useImgIdxs);
	epipolarPanel->handleImagesUsed(useImgIdxs);
	statusBar()->showMessage(tr("image list updated"));

}

void MainWindow::highlightPoints(const int imgIdx1, const int imgIdx2){
	displayPointCloud(false);	//to reset previous highlights just reload the point cloud

	cout<<imgIdx1<<" "<<imgIdx2<<endl;

	vector<int> imgIdxs;
	imgIdxs.push_back(imgIdx1);
	imgIdxs.push_back(imgIdx2);
	vector<Measurement::Ptr> ms;

	coreInterface->getMeasurementsByFrames(imgIdxs, ms);
	cloudViewer->visualizeMeasurements(ms);

}

void MainWindow::handleShowPointsMeasures(const QList<int> idxs){
	statusBar()->showMessage(tr("showing cameras..."));
	displayPointCloud(false);

	vector<int> lmIdxs;
	lmIdxs.reserve(idxs.size());
	for(int i=0; i<idxs.size(); i++){
		lmIdxs.push_back(idxs[i]);
	}
	vector<Measurement::Ptr> ms;

	coreInterface->getMeasurementsByLandMarks(lmIdxs, ms);
	cloudViewer->visualizeMeasurements(ms);
	visibleImagesPanel->showMeasurements(ms);

}

void MainWindow::handleLineCommand(){
	QString s = commandBox->text();
	commandBox->clear();
	QStringList tokens = s.split(' ', QString::SkipEmptyParts);

	if(tokens[0].toLower().compare("highlight")==0){

		int cnt = tokens.size() - 1;
		cout<<"command highlighing "<<cnt<<" cameras"<<endl;
		displayPointCloud(false);
		vector<int> imgIdxs;
		for(int i=1; i<tokens.size(); i++){
			int imgIdx = tokens[i].toInt();
			imgIdxs.push_back(imgIdx);
		}
		vector<Measurement::Ptr> ms;
		coreInterface->getMeasurementsByFrames(imgIdxs, ms);
		cloudViewer->visualizeMeasurements(ms);

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
	}else if(tokens[0].toLower().compare("keep")==0){
		std::vector<int> vals;
		for(int i=1; i<tokens.size(); i++){
			vals.push_back(tokens[i].toInt());
		}
		for(vector<int>::iterator it=vals.begin(); it!=vals.end(); ++it){
			cout<<(*it)<<" ";
		}
		cout<<endl;
		coreInterface->keepCameraByImageIdxs(vals);
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


void MainWindow::ShowStatus(const QString &message){
	statusBar()->showMessage(message);
}


void MainWindow::openFile()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open Project"), "/home/yoyo/Desktop/vmshare/videos/VID_20160505_112747/nvm",
		tr("NVM files (*.nvm)\nMAP files (*.map)\nTINY files (*.tiny)\nTINY2 files (*.tiny2)\nYAML files (*.yaml)\nPATCH files (*.patch)"));

    if (!fileName.isEmpty() && loadFile(fileName)==0){
    	coreInterface -> loadProject(fileName, 0);
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
	 Utils::readPaths(dir.toStdString(),sortedImageList);
	 QList<QString> imageList;
	 imageList.reserve(sortedImageList.size());
	 for(int i=0; i<sortedImageList.size(); i++){
		 imageList.push_back(QString::fromStdString(sortedImageList[i]));
	 }

	 coreInterface 	->setImagePaths(imageRoot, imageList);
	 matchPanel		->setImagePaths(imageRoot, imageList);
	 visibleImagesPanel		->setImagePaths(imageRoot, imageList);
	 epipolarPanel->setImagePaths(imageRoot, imageList);
}

// void MainWindow::openVideo(){
// 	QFileDialog fd(this);
// 	QString ext;
// 	QString fileName = fd.getOpenFileName(this,
// 		        tr("Open Video"), "/home/yoyo/Desktop/vmshare/videos",
// 		        tr("MP4 files (*.mp4)"), &ext);
// 	if (!fileName.isEmpty()){
// 		if(ext=="MP4 files (*.mp4)"){
// 			tabs->setCurrentWidget(videoPanel);
// 			videoPanel->play(fileName);
// 		}
// 	}
// }

void MainWindow::saveFileAs(){

	string tstr;
	Utils::getTimeStampAsString(tstr);
	QFileDialog fd(this);
	QString ext;
	QString fileName = fd.getSaveFileName(this,
	        tr("Save Project"), "/home/yoyo/Desktop/data/save",
	        tr("MAP files (*.map)\nTINY2 files (*.tiny2)\nTINY files (*.tiny)\nPLY files (*.ply)\nYAML files (*.yaml)\nNVM files (*.nvm)\nSKTXT files (*.sktxt)"), &ext);

	if (!fileName.isEmpty()){
		if(ext=="TINY2 files (*.tiny2)"){
			fileName+=".tiny2";
		}else if(ext=="TINY files (*.tiny)"){
			fileName+=".tiny";
		}else if(ext=="PLY files (*.ply)"){
			fileName+=".ply";
		}else if(ext=="YAML files (*.yaml)"){
			fileName+=".yaml";
		}else if(ext=="NVM files (*.nvm)"){
			fileName+=".nvm";
		}else if(ext=="SKTXT files (*.sktxt)"){
			fileName+=".sktxt";
		}else if(ext=="MAP files (*.map)"){
			fileName+=".map";
		}
		cout<<"saving file: "<<fileName.toStdString()<<endl;
		coreInterface -> saveProject(fileName, 0);
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

void MainWindow::handleProjectLoaded(){
	QString imgRoot;
	QList<QString> imgList;
	coreInterface->getImagePaths(imgRoot, imgList);
	matchPanel->setImagePaths(imgRoot, imgList);
	visibleImagesPanel->setImagePaths(imgRoot, imgList);
	epipolarPanel->setImagePaths(imgRoot, imgList);

}

// void MainWindow::handleSaveFeatures(){

	// QString dir = QFileDialog::getExistingDirectory(this,
	// 	 tr("Open Directory"), "/home/yoyo/Desktop/vmshare/videos",
	// 	 QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

	// if(dir==""){
	// 	return;
	// }
	// cout<<"save features to "<<dir.toStdString()<<endl;
	// coreInterface->saveFeatures(dir.toStdString());
// }


