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

	QSplitter *splitter =  new QSplitter;
	splitter->setOrientation(Qt::Horizontal);

	splitter->addWidget(matchPanel);
	splitter->addWidget(cloudViewer);

	setCentralWidget(splitter);
	
	setGeometry(0, 0, 800, 800);
}

void MainWindow::createActions()
{
    openAction = new QAction(tr("&Open"), this);
    //openAction->setIcon(QIcon(":/images/open.png"));
    openAction->setShortcut(tr("Ctrl+1"));
    openAction->setStatusTip(tr("Open image folder"));
    connect(openAction, SIGNAL(triggered()), this, SLOT(openDirectory()));

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
	saveAction->setStatusTip(tr("Save cloud"));
	connect(saveAction, SIGNAL(triggered()), this, SLOT(handleSave()));

	nextPairAction = new QAction(tr("&NextPair"),this);
	nextPairAction->setStatusTip(tr("Let the program choose the next pair to match"));
	connect(nextPairAction, SIGNAL(triggered()), this, SLOT(handleNextPair()));

	checkMatchAction = new QAction(tr("&CheckMatch"),this);
	checkMatchAction->setStatusTip(tr("Check quality of match"));
	connect(checkMatchAction, SIGNAL(triggered()), this, SLOT(handleCheckMatch()));

}

void MainWindow::createMenus()
{
    QMenu *fileMenu = menuBar()->addMenu(tr("&File"));
    fileMenu->addAction(openAction);
}

void MainWindow::createStatusBar()
{
    statusBar()->showMessage(tr(""));
}

void MainWindow::createToolBars()
{
    fileToolBar = addToolBar(tr("File"));
    fileToolBar->addAction(openAction);
    
    viewToolBar = addToolBar(tr("View"));

    viewToolBar->addAction(nextPairAction);
    viewToolBar->addAction(featureMatchAction);
    viewToolBar->addAction(checkMatchAction);
    viewToolBar->addAction(reconstructAction);
	viewToolBar->addAction(bundleAdjustmentAction);
	viewToolBar->addAction(saveAction);

    helpToolBar = addToolBar(tr("Help"));
    //helpToolBar->addAction(helpAction);
    //helpToolBar->addAction(aboutAction);
}

void MainWindow::connectWidgets(){
	connect(cloudViewer, SIGNAL(deletePointIdx(const QList<int>)), this, SLOT(handleDeletePointIdx(const QList<int>)));
	connect(coreInterface, SIGNAL(pointCloudReady()), this, SLOT(displayPointCloud()));
	connect(matchPanelModel, SIGNAL(matchChanged(const QList<QPointF> &, const QList<QPointF> &, const QList<bool> &)), matchPanel, SLOT(updateViews(const QList<QPointF> &, const QList<QPointF> &, const QList<bool> &)));
	connect(coreInterface, SIGNAL(matchResultReady(const QList<QPointF> &, const QList<QPointF> &)), matchPanelModel, SLOT(setMatches(const QList<QPointF> &, const QList<QPointF> &)));
	connect(coreInterface, SIGNAL(nextPairReady(const int, const int)), matchPanel, SLOT(setImagePair(const int, const int)));


	//connect(imgList1, SIGNAL(activated(int)), this, SLOT(handleFirstImageSelected(int)));
	//connect(imgList2, SIGNAL(activated(int)), this, SLOT(handleSecondImageSelected(int)));
}

// Slot functions
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
	statusBar()->showMessage(tr("writing point cloud to ply..."));
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
void MainWindow::displayPointCloud(){
	statusBar()->showMessage(tr("loading cloud..."));
	vector<Point3f> xyzs;
	coreInterface->getPointCloud(xyzs);
	cloudViewer->loadCloud(xyzs);
	statusBar()->showMessage(tr("cloud loaded"));
	
	//also update imgList2 to include only used images
	vector<int>	useImgIdxs;
	coreInterface->getUsedImageIdxs(useImgIdxs);
	matchPanel->handleImageUsed(useImgIdxs);

}

void MainWindow::displayMatchResult(){
	statusBar()->showMessage(tr("loading match result..."));
	/*
	QImage result;
	coreInterface->getMatchResult(result);
	matchViewer->setImage(result);
	statusBar()->showMessage(tr("match result loaded"));
	*/
}



void MainWindow::openCloud()
{
    QString fileName = QFileDialog::getOpenFileName(this,
        tr("Open cloud file"), ".",
		tr("PLY files (*.ply)"));
        
    if (!fileName.isEmpty() && loadFile(fileName)==0){
    	cloudViewer-> loadCloud(fileName);
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
	 PathReader::readPaths(dir.toStdString(),".jpg", sortedImageList);
	 QList<QString> imageList;
	 imageList.reserve(sortedImageList.size());
	 for(int i=0; i<sortedImageList.size(); i++){
		 imageList.push_back(QString::fromStdString(sortedImageList[i]));
	 }

	 coreInterface 	->setImagePaths(imageRoot, imageList);
	 matchPanel		->setImagePaths(imageRoot, imageList);

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


