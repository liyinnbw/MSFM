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
class QToolBar;
class CoreInterfaceWidget;
class MatchPanel;
class QString;

class MainWindow: public QMainWindow
{
    	Q_OBJECT

public:
    MainWindow();

private slots:
	void openCloud();
	void openDirectory();
	void handleReconstruct();
	void handleFeatureMatch();
	void handleSave();
	void handleNextPair();
	void handleBundleAdjustment();
	void handleDeletePointIdx(const QList<int> idxs);
	void handleCheckMatch();
	void handleRemoveBad();
	void displayPointCloud();
	void displayMatchResult();

private:
    void createWidgets();
    void createActions();
    void createMenus();
    void createToolBars();
    void createStatusBar();
	void connectWidgets();
	int loadFile(const QString &fileName);

    QAction 				*openAction;
    QAction 				*featureMatchAction;
	QAction 				*reconstructAction;
	QAction 				*bundleAdjustmentAction;
	QAction 				*nextPairAction;
	QAction 				*checkMatchAction;
	QAction					*removeBadAction;
	QAction 				*saveAction;
	
	QToolBar 				*fileToolBar;
    QToolBar 				*viewToolBar;
    QToolBar 				*helpToolBar;

	CloudWidget 			*cloudViewer;
	CoreInterfaceWidget 	*coreInterface;
	MatchPanel 				*matchPanel;
	MatchPanelModel 		*matchPanelModel;

	QString 				imageRoot;
};

#endif
