/*
 * UI Widget to display back projected 3D points on 2D images 
 */

#ifndef VISIBLEIMAGESPANEL_H_
#define VISIBLEIMAGESPANEL_H_
#include <vector>
#include <QWidget>
#include "core/datastructs/Data.h"

class ImageTileWidget;
class VisibleImagesPanel : public QWidget{
	Q_OBJECT

signals:
	void measuresDeleted();

public slots:
	void deleteMeasures();
	void showProjections();

public:
	VisibleImagesPanel(QWidget *parent = 0);
	virtual ~VisibleImagesPanel();
	void setImagePaths(const QString &root, const QList<QString> &list);
	void showMeasurements(std::vector<Measurement::Ptr> &ms);
	

private:
	void createWidgets();
	void connectWidgets();

	ImageTileWidget		*imageTileView;

	QString 			imgRoot;
	QList<QString>		imgs;

};

#endif /* MATCHPANEL_H_ */
