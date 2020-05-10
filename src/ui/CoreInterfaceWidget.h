/*
 *  Widget to interface with core API
 */

#ifndef COREINTERFACEWIDGET_H
#define COREINTERFACEWIDGET_H

#include <QWidget>
#include <QThread>
#include <QList>

#include <vector>
#include <string>
#include <map>
#include <opencv2/core/core.hpp>

#include "core/datastructs/Data.h"

class SFMPipeline;
class QString;
class QImage;
class QPointF;
class QVector3D;

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
	void getImagePair(					int 							&imgIdx1,
										int 							&imgIdx2);


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
	int 						currentTask;
};

class CoreInterfaceWidget: public QWidget
{
	Q_OBJECT

signals:
	void projectLoaded();
	void pointCloudReady(bool resetView);
	void pointCloud2Ready(bool resetView);
	void polygonReady(bool resetView);
	void matchResultReady(const QList<QPointF> &, const QList<QPointF> &);
	void KeyFrameReady(const QList<QPointF> &);
	void nextPairReady(const int, const int);

public slots:
	void reconstruct(	const int& 			imgIdx1,
						const int& 			imgIdx2,
						const QList<bool> &	mask);
	void handleReconstructFinished();
	// void poseOptimization(	int 			imgIdx);
	void bundleAdjust();
	void handleBundleAdjustFinished();
	void saveProject(const QString &, const int);
	void loadProject(const QString &, const int);
	void deletePointIdx(const QList<int> idxs, const int cloudID);
	void handleDeletePointIdxFinished();
	void matchImages(	const int &imgIdx1,
						const int &imgIdx2);
	void matchImagesEpipolar(	const int &imgIdx1,
								const int &imgIdx2);
	void handleNextPairFinished();


public:
	CoreInterfaceWidget();
	void setImagePaths(const QString &root, const QList<QString> &list);

	void getImagePaths(QString &root, QList<QString> &list);

	void getUsedImageIdxs(std::vector<int> &usedImgIdxs);

	void getMeasurementsByFrames(	const std::vector<int> &		imgIdxs,
									std::vector<Measurement::Ptr>& 	measures);
	void getMeasurementsByLandMarks(const std::vector<int> &		landmarkIdxs,
									std::vector<Measurement::Ptr>&	measures);
	void getMeasuresToPoints(		const std::vector<int> 								&pt3DIdxs,
									std::vector<std::vector<std::pair<int,int> > >		&pt3D2Measures,
									std::vector<std::vector<QPointF> > 					&pt3D2pt2Ds);

	bool coreIsSet(){return core!=NULL;}
	void nextPair();
	void checkMatch(				const int& 			imgIdx1,
									const int& 			imgIdx2,
									const QList<bool> &	mask);

	void addMoreMeasures(			const int& 			imgIdx);

	void addMorePoints(				const int& 			imgIdx1,
									const int& 			imgIdx2);
	void removeBad();
	void deleteCameraByImageIdxs(const std::vector<int> &imgIdxs);
	void keepCameraByImageIdxs(const std::vector<int> &imgIdxs);
	void ApplyGlobalTransformation(const std::vector<double> &transformation);

	void saveFeatures(				const std::string& 	saveFolder);

private:
	SFMPipeline 				*core;
	TaskThread 					*tt;

};

#endif
