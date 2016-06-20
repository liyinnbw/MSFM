/*
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#ifndef IMAGEVIEWER_H
#define IMAGEVIEWER_H

#include <QMainWindow>
#include <QWidget>
#include <QList>
// Class declaration without loading .h files. Faster compilation.
class QAction;
class CloudWidget;
class MatchPanelModel;
class VisibleImagesPanel;
class QToolBar;
class CoreInterfaceWidget;
class MatchPanel;
//class KeyFramePanel;
//class KeyFrameModel;
class QString;
class QLineEdit;

class MainWindow: public QMainWindow
{
    	Q_OBJECT

public:
    MainWindow();

private slots:
	void openFile();
	void openDirectory();
	void openGPSFile();
	void saveFileAs();
	void handleProjectLoaded();
	void handleReconstruct();
	void handleFeatureMatch();
	//void handleSave();
	void handleNextPair();
	void handleBundleAdjustment();
	void handleDeletePointIdx(const QList<int> idxs);
	void handleDeleteMeasures(const QList<QPair<int,int> > &measures);
	void handleShowCamerasSeeingPoints(const QList<int> idxs);
	void handleCheckMatch();
	void handleRemoveBad();
	void handleDense();
	void displayPointCloud(bool resetView);
	void highlightPoints(const int imgIdx1, const int imgIdx2);
	void handleLineCommand();
	void handleNormalRenderToggle();
	void handleDeleteCamera();
	void handleMinSpanCamera();

private:
    void createWidgets();
    void createActions();
    void createMenus();
    void createToolBars();
    void createStatusBar();
	void connectWidgets();
	int loadFile(const QString &fileName);

    QAction 				*openAction;
    QAction 				*openGPSAction;
    QAction 				*openSavedAction;
    QAction 				*featureMatchAction;
	QAction 				*reconstructAction;
	QAction 				*bundleAdjustmentAction;
	QAction 				*nextPairAction;
	QAction 				*checkMatchAction;
	QAction					*removeBadAction;
	QAction 				*saveAction;
	QAction 				*denseAction;
	QAction 				*renderNormalToggleAction;
	QAction 				*removeCameraAction;
	QAction 				*minSpanCamerasAction;
	//QAction					*keyframeAction;
	
	QToolBar 				*fileToolBar;
    QToolBar 				*sfmToolBar;
    //QToolBar 				*ptamToolBar;

	CloudWidget 			*cloudViewer;
	CoreInterfaceWidget 	*coreInterface;
	MatchPanel 				*matchPanel;
	VisibleImagesPanel		*visibleImagesPanel;
	MatchPanelModel 		*matchPanelModel;
	QLineEdit				*commandBox;
	//KeyFramePanel			*keyframePanel;
	//KeyFrameModel			*keyframeModel;

	QString 				imageRoot;
	bool					showNormal;
};

#endif
