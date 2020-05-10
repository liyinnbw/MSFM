/*
 * Base widget for displaying zoomable images
 */

#ifndef IMAGEWIDGET_H
#define IMAGEWIDGET_H

#include <QMainWindow>
#include <QWidget>
#include <QList>

class QImage;
class ImageWidget: public QWidget
{
	Q_OBJECT
	//property, no ";" at the back
	Q_PROPERTY(float zoomFactor READ zoomFactor WRITE setZoomFactor)

signals:
	void markSelected(const int);
	void pointsMarked(const QList<QPointF> &);
	void selectionWindowChanged(const QRect &);
	void markDeleted(const int);
	void imageLoaded(const QImage &);
	void deletePressed();
	void doubleClicked(const QPointF &);

public slots:
	void setImage(const QString &fileName);
	void setImage(const QImage &img);
	void setMarks(const QList<QPointF> &,  const QList<bool> &_mask = QList<bool>());
	void setLines(const QList<QPointF> &);
	void setRect(const QRect &);
public:
	ImageWidget(QWidget *parent = 0);
	QSize sizeHint() const;
	//QString getImagePath(){return imagePath;}
	float imgWidth(){ return image.width();}
	float imgHeight(){return image.height();}
	void setZoomFactor(float newZoom);
	float zoomFactor() const { return zoom; }
	QList<QPointF> getMarks(){return marks;}
	void setSelected(const int);
	int getMarkToHightlight(){return markToHighlight;}
	void deleteSelected();
	bool isVisible(int idx);
	void setMask(QList<bool> m);
	QList<bool> getMask(){return mask;}

	
protected:
	void fitPixmapToWidgetSize();
	void reset();
	virtual void drawMarks();
	virtual void drawLines();
	virtual void drawSelectionWindow();
	void paintEvent(QPaintEvent *event);
	void wheelEvent(QWheelEvent *event);
	virtual void mouseMoveEvent(QMouseEvent *event);
	virtual void mousePressEvent(QMouseEvent *event);
	virtual void mouseReleaseEvent(QMouseEvent *event);
	virtual void mouseDoubleClickEvent(QMouseEvent *event);
	void keyPressEvent(QKeyEvent *event);
	virtual void  distToNearestMark(const QPointF &pos, float &minDist, int &minIdx);
	virtual QRect  pointsToRect(const QPointF &p1, const QPointF &p2);
	
private:
	QImage 					image;
	//QString 				imagePath;
	QPixmap 				pixmap;
	const float 			minZoom;
	float 					zoom;
	QList<QPointF> 			marks;
	QList<QPointF>			lines;
	QList<bool> 			mask;
	int 					markToHighlight;
	bool					showSelectWin;
	QPointF					selectWinStart;
	QPointF					selectWinEnd;
	const QSize				defaultSize;
};

#endif
