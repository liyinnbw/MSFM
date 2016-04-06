/*
 * MatchPanelModel.h
 *
 *  Created on: Apr 3, 2016
 *      Author: yoyo
 */

#ifndef MATCHPANELMODEL_H_
#define MATCHPANELMODEL_H_
#include <QWidget>
#include <QList>
class QPointF;

class MatchPanelModel : public QWidget{
	Q_OBJECT

signals:
	void matchChanged(const QList<QPointF> &, const QList<QPointF> &, const QList<bool> &);

public slots:
	void setMatches(const QList<QPointF> &, const QList<QPointF> &);
	void deleteMatch(const int);

public:
	MatchPanelModel();
	virtual ~MatchPanelModel();

private:
	QList<QPointF> 		img1Points;
	QList<QPointF> 		img2Points;
	QList<bool>	   		mask;
};

#endif /* MATCHPANELMODEL_H_ */
