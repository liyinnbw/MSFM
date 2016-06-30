/*
 * KeyFrameModel.h
 *
 *  Created on: May 3, 2016
 *      Author: yoyo
 */

#ifndef KEYFRAMEMODEL_H_
#define KEYFRAMEMODEL_H_
#include <QWidget>
#include <QList>
#include <QPointF>
class CoreInterfaceWidget;

class KeyFrameModel : public QWidget{
	Q_OBJECT

signals:
	void keyFrameCornersReady(const QList<QList<QPointF> > &);

public slots:
	void setImageIdx(const int idx);
	void computeKeyFrame(const int idx);

public:
	KeyFrameModel(CoreInterfaceWidget *_coreInterface);
	virtual ~KeyFrameModel();

private:
	CoreInterfaceWidget 	*coreInterface;
	int						imgIdx;
};

#endif /* KEYFRAMEMODEL_H_ */
