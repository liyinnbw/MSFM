/*
 * MatchPanelModel.cpp
 *
 *  Created on: Apr 3, 2016
 *      Author: yoyo
 */
#include <QtGlobal>

#include "MatchPanelModel.h"
#include "CoreInterfaceWidget.h"

MatchPanelModel::MatchPanelModel() {
	// TODO Auto-generated constructor stub

}

MatchPanelModel::~MatchPanelModel() {
	// TODO Auto-generated destructor stub
}

void MatchPanelModel::setMatches(const QList<QPointF> &pts1, const QList<QPointF> &pts2){
	assert(pts1.size() == pts2.size());
	img1Points = pts1;
	img2Points = pts2;
	mask.clear();
	for(int i=0; i<pts1.size(); i++){
		mask.push_back(true);
	}
/*
	//TODO: replace with get call
	int matchCnt = 3;
	img1Points.push_back(QPointF(10.0, 20.0));
	img1Points.push_back(QPointF(50.0, 80.0));
	img1Points.push_back(QPointF(100.0, 40.0));
	img2Points.push_back(QPointF(12.0, 205.0));
	img2Points.push_back(QPointF(56.0, 81.0));
	img2Points.push_back(QPointF(207.0, 72.0));
	mask.reserve(matchCnt);
	for(int i=0; i<matchCnt; i++){
		mask.push_back(true);
	}
*/

	emit matchChanged(img1Points, img2Points, mask);

}

void MatchPanelModel::deleteMatch(const int idx){
	assert(idx>=0 && idx<mask.size());
	mask[idx] = false;
	emit matchChanged(img1Points, img2Points, mask);
}

void MatchPanelModel::setImages(const int imgIdx1, const int imgIdx2){
	img1Points.clear();
	img2Points.clear();
	mask.clear();
	emit matchChanged(img1Points, img2Points, mask);
}

