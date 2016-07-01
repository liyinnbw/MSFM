/*
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#ifndef IMAGEVIEWER_H
#define IMAGEVIEWER_H

#include <QMainWindow>
#include <QWidget>
#include <QList>
#include <QVector3D>
// Class declaration without loading .h files. Faster compilation.
class QAction;
class CloudWidget;
class MatchPanelModel;
class VisibleImagesPanel;
class QToolBar;
class CoreInterfaceWidget;
class MatchPanel;
class KeyFramePanel;
class KeyFrameModel;
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
	void openPolygonFile();
	void saveFileAs();
	void handleProjectLoaded();
	void handleReconstruct();
	void handleFeatureMatch();
	void handleAddKeyFrame();
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
	void displayPointCloud2(bool resetView);
	void displayPolygon(bool resetView);
	void handleKeyFramePanelImageChange(int imgIdx);
	void highlightPoints(const int imgIdx1, const int imgIdx2);
	void handleLineCommand();
	void handleNormalRenderToggle();
	void handlePolygonRenderToggle();
	void handleDeleteCamera();
	void handleMinSpanCamera();
	void handleKeyframeImagePointsSelected(const int imgIdx, const QList<QPointF> &pts);

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
    QAction 				*openPolygonAction;
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
	QAction 				*renderPolygonToggleAction;
	QAction 				*removeCameraAction;
	QAction 				*minSpanCamerasAction;
	QAction					*addKeyFrameAction;
	//QAction					*keyframeAction;
	
	QToolBar 				*fileToolBar;
    QToolBar 				*sfmToolBar;
    //QToolBar 				*ptamToolBar;

	CloudWidget 			*cloudViewer;
	CloudWidget 			*cloudViewer2;
	CoreInterfaceWidget 	*coreInterface;
	MatchPanel 				*matchPanel;
	VisibleImagesPanel		*visibleImagesPanel;
	MatchPanelModel 		*matchPanelModel;
	QLineEdit				*commandBox;
	KeyFramePanel			*keyframePanel;
	KeyFrameModel			*keyframeModel;
	QTabWidget				*tabs;
	QTabWidget				*tabs2;

	QString 				imageRoot;
	bool					showNormal;
	bool					showPolygon;
	int						keyframeSelectedImgIdx;

	int						mImgIdx;
	QList<QPointF> 			mxys;
	QList<QVector3D> 		mxyzs;
	QList<QVector3D> 		mnorms;
	std::vector<bool> 		mstatus;
};

#endif
