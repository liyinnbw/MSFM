/*
 * Main UI
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
class EpipolarPanel;
class QString;
class QLineEdit;
class VideoPanel;
class FramePanel;
class GraphWidget;
class MainWindow: public QMainWindow
{
    	Q_OBJECT

public:
    MainWindow();

private slots:

	//file operations
	void openFile();
	// void openVideo();
	void openDirectory();
	void saveFileAs();
	// void handleSaveFeatures();
	void handleProjectLoaded();

	//sfm
	void handleNextPair();
	void handleFeatureMatch();
	void handleEpipolarFeatureMatch();
	void handleCheckMatch();
	void handleReconstruct();
	void handleAddMoreMeasures();
	void handleAddMorePoints();
	void handlePoseOptimization();
	void handleBundleAdjustment();

	//data operation
	void handleDeletePoints(const QList<int> idxs);
	void handleDeleteCamera();
	void handleRemoveBad();

	//visualization
	void displayPointCloud(bool resetView);
	void highlightPoints(const int imgIdx1, const int imgIdx2);
	void handleShowPointsMeasures(const QList<int> idxs);

	void handleLineCommand();

	void ShowStatus(const QString &);


private:
    void createWidgets();
    void createActions();
    void createMenus();
    void createToolBars();
    void createStatusBar();
	void connectWidgets();
	int loadFile(const QString &fileName);

    QAction 				*openAction;
    QAction 				*openVideoAction;
    QAction 				*openSavedAction;
    QAction 				*featureMatchAction;
    QAction 				*epipolarMatchAction;
	QAction 				*reconstructAction;
	QAction 				*addMoreMeasuresAction;
	QAction 				*addMorePointsAction;
	QAction 				*poseOptimizationAction;
	QAction 				*bundleAdjustmentAction;
	QAction 				*nextPairAction;
	QAction 				*checkMatchAction;
	QAction					*removeBadAction;
	QAction 				*saveAction;
	QAction 				*saveFeaturesAction;
	QAction 				*removeCameraAction;


	QToolBar 				*fileToolBar;
    QToolBar 				*sfmToolBar;


	CloudWidget 			*cloudViewer;
	CoreInterfaceWidget 	*coreInterface;
	MatchPanel 				*matchPanel;
	EpipolarPanel			*epipolarPanel;
	VisibleImagesPanel		*visibleImagesPanel;
	VideoPanel				*videoPanel;
	FramePanel				*framePanel;
	GraphWidget 			*graphWidget;
	MatchPanelModel 		*matchPanelModel;
	QLineEdit				*commandBox;
	QTabWidget				*tabs;
	QTabWidget				*tabs2;

	QString 				imageRoot;

	QList<QPointF> 			mxys;
	QList<QVector3D> 		mxyzs;
	QList<QVector3D> 		mnorms;
	std::vector<bool> 		mstatus;
};

#endif
