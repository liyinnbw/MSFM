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

class ImageWidget;
class QComboBox;
class KeyFramePanel : public QWidget{
	Q_OBJECT

signals:
	void imageChanged(int);
	void doComputeKeyFrame(const int idx);
	void imagePointSelected(const int, const QPointF &);
	void imagePointsSelected(const int, const QList<QPointF> &);

public slots:
	void handleImageSelected(int idx);
	void handleImageLevelSelected(int lvl);
	void handlePointsSelected(const QList<QPointF> & pts);
	void computeKeyFrame();
	void updateCorners(const QList<QList<QPointF> > &);

public:
	KeyFramePanel();
	virtual ~KeyFramePanel();
	void setImagePaths(const QString &root, const QList<QString> &list);

private:
	void createWidgets();
	void connectWidgets();

	ImageWidget 					*imageView;
	QComboBox 						*imgList;
	QComboBox 						*lvlList;

	QString 						imgRoot;
	QList<QList<QPointF> > 			levelCorners;
};

#endif /* KEYFRAMEPANEL_H_ */
