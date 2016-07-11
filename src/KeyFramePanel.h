/*
 * KeyFramePanel.h
 *
 *  Created on: May 3, 2016
 *      Author: yoyo
 */

#ifndef KEYFRAMEPANEL_H_
#define KEYFRAMEPANEL_H_
#include <QWidget>
#include <QList>
#include <QPointF>


class QComboBox;
class QPushButton;
//class ImageWidget;
class KeyFrameWidget;
class KeyFramePanel : public QWidget{
	Q_OBJECT

signals:
	void imageChanged(const int);
	void doComputeKeyFrame(const int idx);
	void imagePointSelected(const int, const QPointF &);
	void imagePointsSelected(const int, const QList<QPointF> &);

public slots:
	void handleImageSelected(int idx);
	void handleImageLevelSelected(int lvl);
	void handlePointsSelected(const QList<QPointF> & pts);
	void computeKeyFrame();
	void updateCorners(const QList<QList<QPointF> > &);
	void handleComputeKeyFrameClicked();

public:
	KeyFramePanel();
	virtual ~KeyFramePanel();
	void setImagePaths(const QString &root, const QList<QString> &list);
	void drawProjection(const QList<QPointF> &verts);	//3 verts = 1 face

private:
	void createWidgets();
	void connectWidgets();

	KeyFrameWidget 					*imageView;
	QComboBox 						*imgList;
	QComboBox 						*lvlList;
	QPushButton 					*computeKeyFrameButton;

	QString 						imgRoot;
	QList<QList<QPointF> > 			levelCorners;
	int								currentImgIdx;
};

#endif /* KEYFRAMEPANEL_H_ */
