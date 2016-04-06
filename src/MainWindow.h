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
	void displayPointCloud();
	void displayMatchResult();

private:
    void createWidgets();
    void createActions();
    void createMenus();
    void createToolBars();
    void createStatusBar();
	void connectWidgets();

    QAction *openAction;
    QAction *featureMatchAction;
	QAction *reconstructAction;
	QAction *bundleAdjustmentAction;
	QAction *nextPairAction;
	QAction *checkMatchAction;
	QAction *saveAction;
	
	QToolBar *fileToolBar;
    QToolBar *viewToolBar;
    QToolBar *helpToolBar;

    //QComboBox *imgList1;
    //QComboBox *imgList2;


    // Supporting methods
    int loadFile(const QString &fileName);
	CloudWidget *cloudViewer;
	CoreInterfaceWidget *coreInterface;
	QString 	imageRoot;
	MatchPanel 	*matchPanel;
	MatchPanelModel *matchPanelModel;
};

#endif
