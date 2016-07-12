/*
 *  Created on: Mar 11, 2016
 *      Author: yoyo
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

class SFMPipeline;
class QString;
class QImage;
class QPointF;
class QVector3D;
struct KeyFrame;
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
	void setCloudID(const int cloudID){mCloudID = cloudID;}

	//Getters
	void getImagePair(					int 							&imgIdx1,
										int 							&imgIdx2);


	const static int TASK_NONE 			= 0;
	const static int TASK_RECONSTRUCT 	= 1;
	const static int TASK_DELETEPOINTS	= 2;
	const static int TASK_BUNDLEADJUST	= 3;
	const static int TASK_NEXTPAIR		= 4;

	int 						mCloudID;

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
	void reconstruct(const QList<bool> &);
	void handleReconstructFinished();
	void bundleAdjust();
	void handleBundleAdjustFinished();
	void saveProject(const QString &, const int);
	void loadProject(const QString &);
	void loadGPS(const QString &);
	void loadPolygon(const QString &);
	void deletePointIdx(const QList<int> idxs, const int cloudID);
	void handleDeletePointIdxFinished();
	void matchImages(	const int &imgIdx1,
						const int &imgIdx2);
	void handleNextPairFinished();
	void keepMinSpanCameras();

public:
	CoreInterfaceWidget();
	void setImagePaths(const QString &root, const QList<QString> &list);
	void getImagePaths(QString &root, QList<QString> &list);
	void getPointCloud(	std::vector<cv::Point3f> &xyzs);
	void getPointCloud2(	std::vector<cv::Point3f> &xyzs);
	void getPolygons(std::vector<cv::Point3f> &verts, std::vector<cv::Point3i> &faces);
	void getVisiblePolygons(const int imgIdx, std::vector<cv::Point3f> &verts, std::vector<cv::Point3i> &faces);
	void getPointNormals(	std::vector<cv::Point3f> &norms);
	void getPointNormals2(	std::vector<cv::Point3f> &norms);
	void getCameras(std::vector<cv::Matx34d> &cams);
	void getCameras2(std::vector<cv::Matx34d> &cams);
	void getUsedImageIdxs(std::vector<int> &usedImgIdxs);
	void getAll3DfromImage2D(	const int 					imgIdx,
								std::vector<cv::Point3f>	&pts3D,
								std::vector<int>			&pts3DIdxs);
	void getCameraIdx(			const int 					imgIdx,
								int 						&camIdx);
	bool coreIsSet(){return core!=NULL;}
	void nextPair();
	void checkMatch(const QList<bool> &);
	void removeBad();
	void deleteCameraByImageIdxs(const std::vector<int> &imgIdxs);
	void deleteMeasures(const QList<QPair<int,int> > &measures);
	void denseReconstruct();
	void computeKeyFrame(const int imgIdx, KeyFrame &kf);
	void getOverlappingImgs(	const int 								baseImgIdx,
								std::map<int,std::vector<int> > 		&img2pt3Didxs);
	void getBestOverlappingImgs(const int 								baseImgIdx,
								std::map<int,std::vector<int> > 		&img2pt3Didxs);
	void getMeasuresToPoints(	const std::vector<int> 								&pt3DIdxs,
								std::vector<std::vector<std::pair<int,int> > >		&pt3D2Measures,
								std::vector<std::vector<QPointF> > 					&pt3D2pt2Ds);
	void ApplyGlobalTransformation(const std::vector<double> &transformation);

	void projectImagePointsTo3DSurface(		const int 						imgIdx,
											const QList<QPointF> 			&xys,
											QList<QVector3D>				&xyzs,
											QList<QVector3D>				&norms,
											std::vector<bool>				&status);

	void projectPolygonToImage(				const int 						imgIdx,
											QList<QPointF>					&xys);	//3verts = 1 face

	void addKeyFrame(						const int 						imgIdx,
											const QList<QPointF> 			&xys,
											const QList<QVector3D>			&xyzs,
											const QList<QVector3D>			&norms,
											const std::vector<bool>			&status);
private:
	SFMPipeline 				*core;
	TaskThread 					*tt;
	int							imgIdx1, imgIdx2;
	//std::vector<cv::KeyPoint> 	kpts1, kpts2;
	//cv::Mat 					decs1, decs2;
	//std::vector<cv::DMatch> 	matches;

};

#endif
