/*
 * Interactive widget display feature matches between two 2D images
 */

#ifndef MATCHPANEL_H_
#define MATCHPANEL_H_
#include <QWidget>

class ImageWidget;
class MatchWidget;
class QComboBox;
class MatchPanel : public QWidget{
	Q_OBJECT

signals:
	void updateImageView1(const QList<QPointF> &);
	void updateImageView2(const QList<QPointF> &);
	void updateMatchView(const QList<QPointF> &, const QList<QPointF> &);
	void firstImageSelected(const int imgIdx);
	void imageChanged(const int imgIdx1, const int imgIdx2);

public slots:
	void handleFirstImageSelected(int idx);
	void handleSecondImageSelected(int idx);
	void updateViews(const QList<QPointF> &, const QList<QPointF> &, const QList<bool> &);
	void updateSelected(const int);
	void handleDelete();
	void setImagePair(const int, const int);

public:
	MatchPanel(QWidget *parent = 0);
	virtual ~MatchPanel();
	void setImagePaths(const QString &root, const QList<QString> &list);
	void getSelectedImages(int &idx1, int &idx2);
	void getMask(QList<bool> &);
	void handleImagesUsed(std::vector<int> &usedImgIdxs);


private:
	void createWidgets();
	void connectWidgets();

	ImageWidget *imageView1;
	ImageWidget *imageView2;
	MatchWidget *matchView;
	QComboBox 	*imgList1;
	QComboBox 	*imgList2;

	QString imgRoot;
};

#endif /* MATCHPANEL_H_ */
