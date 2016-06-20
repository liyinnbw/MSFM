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
	void deleteSelectedMeasures(const QList<QPair<int,int> > &);

public slots:
	void deleteMeasures();

public:
	VisibleImagesPanel(QWidget *parent = 0);
	virtual ~VisibleImagesPanel();
	void setImagePaths(const QString &root, const QList<QString> &list);
	void setVisibleImagesAndMeasures( 	const std::vector<std::vector<std::pair<int,int> > > 	&pt3D2Measures,
										const std::vector<std::vector<QPointF> > 				&pt3D2pt2Ds);


private:
	void createWidgets();
	void connectWidgets();

	ImageTileWidget		*imageTileView;

	QString 			imgRoot;
	QList<QString>		imgs;

};

#endif /* MATCHPANEL_H_ */
