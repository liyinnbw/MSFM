/*
 * Interactive Widget to draw epipolar line between two images
 */

#ifndef SRC_EPIPOLARPANEL_H_
#define SRC_EPIPOLARPANEL_H_
#include <QWidget>

class ImageWidget;
class QComboBox;
class EpipolarPanel : public QWidget{
	Q_OBJECT

signals:
	void updateImageView1(const QList<QPointF> &);
	void updateImageView2(const QList<QPointF> &);
	void firstImageSelected(const int imgIdx);
	void imageChanged(const int imgIdx1, const int imgIdx2);

public slots:
	void handleFirstImageSelected(int idx);
	void handleSecondImageSelected(int idx);
	void handleImageView1PointMarked(const QList<QPointF> &);
	void handleImageView1PointSelected(const int);
	void setImagePair(const int, const int);

public:
	EpipolarPanel(QWidget *parent = 0);
	virtual ~EpipolarPanel();

	void setImagePaths(const QString &root, const QList<QString> &list);
	void getSelectedImages(int &idx1, int &idx2);
	void getMask(QList<bool> &);
	void handleImagesUsed(std::vector<int> &usedImgIdxs);
private:
	void createWidgets();
	void connectWidgets();

	ImageWidget *imageView1;
	ImageWidget *imageView2;
	QComboBox 	*imgList1;
	QComboBox 	*imgList2;

	QString imgRoot;
};

#endif /* SRC_EPIPOLARPANEL_H_ */
