#ifndef COREINTERFACEWIDGET_H
#define COREINTERFACEWIDGET_H

#include <QWidget>
#include <QThread>
#include <QList>

#include <vector>
#include <string>

#include <opencv2/core/core.hpp>

class SFMPipeline;
class QString;
class QImage;
class QPointF;

class TaskThread : public QThread
{
    Q_OBJECT

signals:
    void finished();

public:
	TaskThread(SFMPipeline *core);

	//Setters
	void setTask(const int &taskID);
	void setDeleteIdxs(const QList<int> &idxs);
	void setImagePair(const int &imgIdx1, const int &imgIdx2);
	void setReconstructionParameters(	const int 						&imgIdx1,
										const int 						&imgIdx2,
										const QList<bool> 				&mask);

	//Getters
	void getImagePair(int &imgIdx1, int &imgIdx2);

	const static int TASK_NONE 			= 0;
	const static int TASK_RECONSTRUCT 	= 1;
	const static int TASK_DELETEPOINTS	= 2;
	const static int TASK_BUNDLEADJUST	= 3;
	const static int TASK_NEXTPAIR		= 4;


protected:
    void run();
	
private:	
	SFMPipeline 				*mCore;
	QList<int> 					deleteIdxs;
	int							mImgIdx1;
	int							mImgIdx2;
	QList<bool>					mMask;
	int currentTask;
};

class CoreInterfaceWidget: public QWidget
{
	Q_OBJECT

signals:
	void pointCloudReady();
	void matchResultReady(const QList<QPointF> &, const QList<QPointF> &);
	void nextPairReady(const int, const int);

public slots:
	void reconstruct(const QList<bool> &);
	void handleReconstructFinished();
	void bundleAdjust();
	void handleBundleAdjustFinished();
	void saveCloud();
	void deletePointIdx(const QList<int> idxs);
	void handleDeletePointIdxFinished();
	void matchImages(	const int &imgIdx1,
						const int &imgIdx2);
	void handleNextPairFinished();

public:
	CoreInterfaceWidget();
	void setImagePaths(const QString &root, const QList<QString> &list);
	void getPointCloud(	std::vector<cv::Point3f> &xyzs);
	bool coreIsSet(){return core!=NULL;}
	void nextPair();
	void checkMatch(const QList<bool> &);

private:
	SFMPipeline 				*core;
	TaskThread 					*tt;
	int							imgIdx1, imgIdx2;
	//std::vector<cv::KeyPoint> 	kpts1, kpts2;
	//cv::Mat 					decs1, decs2;
	//std::vector<cv::DMatch> 	matches;

};

#endif
