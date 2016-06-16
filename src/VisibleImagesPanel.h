/*
 *  Created on: Jun 16, 2016
 *      Author: yoyo
 */

#ifndef VISIBLEIMAGESPANEL_H_
#define VISIBLEIMAGESPANEL_H_
#include <QWidget>

class ImageTileWidget;
class VisibleImagesPanel : public QWidget{
	Q_OBJECT

signals:


public slots:


public:
	VisibleImagesPanel(QWidget *parent = 0);
	virtual ~VisibleImagesPanel();
	void setImagePaths(const QString &root, const QList<QString> &list);
	void setVisibleImagesAndMeasures( QMap<int, QList<QPointF> > img2pt2Ds);


private:
	void createWidgets();
	void connectWidgets();

	ImageTileWidget		*imageTileView;

	QString 			imgRoot;
	QList<QString>		imgs;

};

#endif /* MATCHPANEL_H_ */
