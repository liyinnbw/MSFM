/*
* Centralized object containg all sfm data
*/

#ifndef SRC_CORE_DATASTRUCTS_DATA_H_
#define SRC_CORE_DATASTRUCTS_DATA_H_

#include <vector>
#include <list>
#include <map>
#include <iostream>
#include "Frame.h"
#include "LandMark.h"
#include "Measurement.h"

// #include <DBoW3/DBoW3.h>

class Data {
public:
	struct LandMarkPtrCompare {
		bool operator()(const LandMark::Ptr& a, const LandMark::Ptr& b) const {
			return a.get() < b.get();
		}
	};
	struct FramePtrCompare{
		bool operator()(const Frame::Ptr& a, const Frame::Ptr& b) const {
			return a.get() < b.get();
		}
	};
	static bool FrameMatchComparator(const std::pair<Frame::Ptr,double> &i, const std::pair<Frame::Ptr,double> &j)
	{
		return i.second>j.second;
	}

	//singleton
	static Data& GetInstance(){
		static Data instance;
		return instance;
	}

	void reset(){
		nonKeyFrames.clear();
		frames.clear();
		landmarks.clear();
		measures.clear();
		landmarkMeasures.clear();
		frameMeasures.clear();
		// if(vocabDB){
		// 	delete vocabDB;
		// 	vocabDB = nullptr;
		// }
		dbIdx2frameIdx.clear();
		frameIdx2dbIdx.clear();
	}

	//this is called manually when necessary to delete data that has been marked for deletion
	void deleteTrashes(){

		int measuresBefore = measures.size();
		int landmarksBefore= landmarks.size();
		int cntMeasure1 = 0;
		int cntMeasure2 = 0;
		int cntMeasure3 = 0;
		int cntLandmark1= 0;
		int cntLandmark2= 0;
		int cntFrame	= 0;

		for(std::vector<Measurement::Ptr>::iterator it = measures.begin(); it!=measures.end(); ){
			if((*it)->deleted){
				it = measures.erase(it);
				cntMeasure1++;
			}
			else ++it;
		}
		for(std::vector<LandMark::Ptr>::iterator it = landmarks.begin(); it!=landmarks.end(); ){
			if((*it)->deleted){
				it = landmarks.erase(it);
				cntLandmark1++;
			}
			else ++it;
		}

		//XXX:depending on usage, the lookup tables may not need to be cleaned of trashes
		//because for frameMeasures, the trash size is limited by the amount of features in each frame
		//and new measurement may replace these trashes anyway.
		//for landmarkMeasures, the trash size is limited by the total number of frames
		//and new measurement may replace these trashes anyway.
		for(MeasureLUTLandMark::iterator it = landmarkMeasures.begin(); it!=landmarkMeasures.end();){

			if(it->first->deleted){
				assert(it->first->measuresCnt == 0);
				MeasurePerFrame& mpf = it->second;
				for(MeasurePerFrame::iterator jt = mpf.begin(); jt!=mpf.end();++jt){
					assert(jt->second->deleted);
				}
				cntMeasure2+=mpf.size();
				it = landmarkMeasures.erase(it);
				cntLandmark2++;
			}else{
				MeasurePerFrame& mpf = it->second;
				for(MeasurePerFrame::iterator jt = mpf.begin(); jt!=mpf.end();){
					if(jt->second->deleted){
						jt=mpf.erase(jt);
						cntMeasure2++;
					}
					else ++jt;
				}
				assert(!mpf.empty());
				++it;
			}
		}
		for(MeasureLUTFrame::iterator it = frameMeasures.begin(); it!=frameMeasures.end();){

			if(it->first->measuresCnt == 0){
				MeasurePerFeature& mpfe = it->second;
				for(MeasurePerFeature::iterator jt = mpfe.begin(); jt!=mpfe.end();++jt){
					assert(jt->second->deleted);
				}
				cntMeasure3+=mpfe.size();
				it = frameMeasures.erase(it);
				cntFrame++;
			}else{
				MeasurePerFeature& mpfe = it->second;
				for(MeasurePerFeature::iterator jt = mpfe.begin(); jt!=mpfe.end();){
					if(jt->second->deleted){
						jt=mpfe.erase(jt);
						cntMeasure3++;
					}
					else ++jt;
				}
				assert(!mpfe.empty());
				++it;
			}
		}

		//std::cout<<"before: measures:"<<measuresBefore<<" landmarks:"<<landmarksBefore<<std::endl;
		//std::cout<<"after: measures:"<<measures.size()<<" landmarks:"<<landmarks.size()<<std::endl;
		//std::cout<<"measures deleted: total:"<<cntMeasure1<<" landmark lut:"<<cntMeasure2<<" frame lut:"<<cntMeasure3<<std::endl;
		//std::cout<<"landmark deleted: total:"<<cntLandmark1<<" landmark lut:"<<cntLandmark2<<std::endl;
		//std::cout<<"frames deleted: frame lut:"<<cntFrame<<std::endl;
	}

	inline void addFrame(Frame::Ptr frame)
	{
		assert(frame);
		assert(!getFrame(frame->imgIdx));
		frames.push_back(frame);
	}
	inline void deleteFrame(Frame::Ptr frame)
	{
		assert(frame);
		//clear all measures associated with the frame, then delete it from frameMeasures
		//do not delete from landmarkMeasures or measures immediately for performance consideration.
		//just mark them deleted. we will do the actual cleanup by explicitly
		//calling deleteTrashes() at appropriate times.
		MeasureLUTFrame::iterator itfrm = frameMeasures.find(frame);
		if(itfrm!=frameMeasures.end()){
			MeasurePerFeature& mpfe	= itfrm->second;
			for(MeasurePerFeature::iterator it = mpfe.begin(); it != mpfe.end(); ++it){
				it->second->clear();
			}
			assert(itfrm->first->measuresCnt == 0);
			frameMeasures.erase(itfrm);
		}
		//delete frame from array
		for(std::vector<Frame::Ptr>::iterator it=frames.begin(); it!=frames.end();){
			if((*it).get() == frame.get()){
				it = frames.erase(it);
				return;
			}else{
				++it;
			}
		}
	}
	inline void addLandMark(LandMark::Ptr landmark)
	{
		assert(landmark && !landmark->deleted);
		landmarks.push_back(landmark);
	}
	inline void deleteLandMark(LandMark::Ptr landmark)
	{
		assert(landmark);
		MeasureLUTLandMark::iterator itlmk = landmarkMeasures.find(landmark);
		assert(itlmk != landmarkMeasures.end());
		MeasurePerFrame& mpf = itlmk->second;
		for(MeasurePerFrame::iterator it = mpf.begin(); it!=mpf.end(); ++it){
			it->second->clear();
		}
		//landmark measureCnt should be automatically zero if all measures tied to it were cleared
		//landmark should be automatically marked deleted if all measures tied to it were cleared
		assert(itlmk->first->measuresCnt == 0 && landmark->deleted);
		landmarkMeasures.erase(itlmk);

		//many landmarks are deleted together, which may not be efficient to erase from vector
		//just mark it as delete. we will do the actual cleanup by explicitly
		//calling deleteTrashes() at appropriate times.

	}
	inline void addMeasurement(Measurement::Ptr measure)
	{
		assert(measure && !measure->deleted);

		MeasureLUTLandMark::iterator itlmk = landmarkMeasures.find(measure->landmark);
		MeasureLUTFrame::iterator 	 itfrm = frameMeasures.find(measure->frame);

		if(itlmk == landmarkMeasures.end()){
			MeasurePerFrame mpf;
			mpf[measure->frame] = measure;
			landmarkMeasures[measure->landmark] = mpf;	//copy occurs
		}else{
			MeasurePerFrame& mpf = itlmk->second;
			MeasurePerFrame::iterator itmpf = mpf.find(measure->frame);
			assert(itmpf == mpf.end() || itmpf->second->deleted);
			mpf[measure->frame] = measure;
		}

		if(itfrm == frameMeasures.end()){
			MeasurePerFeature mpfe;
			mpfe[measure->featureIdx] = measure;
			frameMeasures[measure->frame] = mpfe;	//copy occurs
		}else{
			MeasurePerFeature& mpfe = itfrm->second;
			MeasurePerFeature::iterator itmpfe = mpfe.find(measure->featureIdx);
			assert(itmpfe == mpfe.end() || itmpfe->second->deleted);
			mpfe[measure->featureIdx] = measure;
		}

		measures.push_back(measure);
	}
	inline void deleteMeasurement(Measurement::Ptr measure)
	{
		assert(measure);
		//just mark it delete in any of the lookup table
		//dont perform actual deletion for performance consideration
		//we will do the actual cleanup by explicitly
		//calling deleteTrashes() at appropriate times.

		MeasureLUTLandMark::iterator itlmk = landmarkMeasures.find(measure->landmark);
		MeasureLUTFrame::iterator 	 itfrm = frameMeasures.find(measure->frame);

		assert(itlmk != landmarkMeasures.end());
		assert(itfrm != frameMeasures.end());

		MeasurePerFrame& mpf = itlmk->second;
		MeasurePerFrame::iterator itmpf = mpf.find(measure->frame);
		assert(itmpf != mpf.end() && !(itmpf->second->deleted));

		MeasurePerFeature& mpfe = itfrm->second;
		MeasurePerFeature::iterator itmpfe = mpfe.find(measure->featureIdx);
		assert(itmpfe != mpfe.end() && !(itmpfe->second->deleted));

		measure->clear();
		assert(itmpf->second->deleted && itmpfe->second->deleted);

	}

	inline void deleteMeasurements(Frame::Ptr frame)
	{
		MeasureLUTFrame::iterator 	 itfrm = frameMeasures.find(frame);
		assert(itfrm != frameMeasures.end());
		MeasurePerFeature& mpfe = itfrm->second;
		for(MeasurePerFeature::iterator itmpef = mpfe.begin(); itmpef!=mpfe.end(); ++itmpef){
			itmpef->second->clear();
		}
	}

	//mass getters
	inline int 								countFrames(){return frames.size();}		//note this will count deleted ones also
	inline int								countLandMarks(){return landmarks.size();}	//note this will count deleted ones also
	inline int								countMeasurements(){return measures.size();}//note this will count deleted ones also
	inline const std::vector<Frame::Ptr>& 		getFrames(){return frames;}
	inline const std::vector<LandMark::Ptr>&	getLandMarks(){return landmarks;}
	inline const std::vector<Measurement::Ptr>&	getMeasurements(){return measures;}

	//randomnizer
	inline void								shuffleLandMarks()
	{
		std::random_shuffle(landmarks.begin(), landmarks.end());
	}

	//helper
	inline void 							fixAllFrames(){
		for(std::vector<Frame::Ptr>::iterator it = frames.begin(); it!=frames.end(); ++it){
			(*it)->fixed = true;
		}
	}

	//individual element getter
	inline Frame::Ptr 						getFrame(const int &idx)
	{
		for(int i=0; i<frames.size(); i++){
			if(idx == frames[i]->imgIdx){
				return frames[i];
			}
		}
		return nullptr;
	}

	/*inline std::vector<LandMark::Ptr>		getLandMarks( const Frame::Ptr &frame){
		std::vector<LandMark::Ptr> lms;
		const std::vector<Measurement::Ptr> &ms = getMeasurements(frame);
		lms.reserve(ms.size());
		for(std::vector<Measurement::Ptr>::const_iterator it = ms.begin(); it!=ms.end(); ++it){
			lms.push_back((*it)->landmark);
		}
		return lms;
	}*/

	inline Measurement::Ptr					getMeasurement(const Frame::Ptr& frame, const int& featureIdx)
	{
		MeasureLUTFrame::iterator itfrm = frameMeasures.find(frame);
		if(itfrm == frameMeasures.end()) return nullptr;
		MeasurePerFeature& mpfe = itfrm->second;
		MeasurePerFeature::iterator itmpfe = mpfe.find(featureIdx);
		if(itmpfe == mpfe.end() || itmpfe->second->deleted) return nullptr;
		return itmpfe->second;
	}

	inline Measurement::Ptr					getMeasurement(const LandMark::Ptr& landmark, const Frame::Ptr& frame)
	{
		MeasureLUTLandMark::iterator itlmk = landmarkMeasures.find(landmark);
		if(itlmk == landmarkMeasures.end()) return nullptr;
		MeasurePerFrame& mpf = itlmk->second;
		MeasurePerFrame::iterator itmpf = mpf.find(frame);
		if(itmpf == mpf.end() || itmpf->second->deleted) return nullptr;
		return itmpf->second;
	}

	inline std::vector<Measurement::Ptr>	getMeasurements(const Frame::Ptr& frame)
	{
		std::vector<Measurement::Ptr> mMeasures;
		MeasureLUTFrame::iterator itfrm = frameMeasures.find(frame);
		if(itfrm == frameMeasures.end()) return mMeasures;
		MeasurePerFeature& mpfe = itfrm->second;
		for(MeasurePerFeature::iterator it = mpfe.begin(); it != mpfe.end(); ++it){
			if(it->second->deleted) continue;
			mMeasures.push_back(it->second);
		}
		return mMeasures;
	}

	inline std::vector<Measurement::Ptr> 	getMeasurements(const LandMark::Ptr& landmark){
		std::vector<Measurement::Ptr> mMeasures;
		MeasureLUTLandMark::iterator itlmk = landmarkMeasures.find(landmark);
		if(itlmk == landmarkMeasures.end()) return mMeasures;
		MeasurePerFrame& mpf = itlmk->second;

		for(MeasurePerFrame::iterator it = mpf.begin(); it!=mpf.end(); ++it){
			if(it->second->deleted) continue;
			mMeasures.push_back(it->second);
		}
		return mMeasures;
	}

	// void makeVocabDB(){
	// 	std::cout<<"creating vocab DB ..."<<std::endl;
	// 	//vocabulary settings
	// 	const int k = 9;	// tree branching factor
	// 	const int L = 6;	// tree depth levels
	// 	const DBoW3::WeightingType weight = DBoW3::TF_IDF;
	// 	const DBoW3::ScoringType score = DBoW3::L1_NORM;
	// 	DBoW3::Vocabulary vocab(k, L, weight, score);

	// 	//create vocabulary from current data frames
	// 	std::vector<cv::Mat> vecDecs;
	// 	for(std::vector<Frame::Ptr>::const_iterator it = frames.begin(); it!= frames.end(); ++it){
	// 		assert(!((*it)->decs.empty()));
	// 		vecDecs.push_back((*it)->decs);
	// 	}
	// 	vocab.create(vecDecs);
	// 	std::cout<<vocab<<std::endl;

	// 	//create database
	// 	if(vocabDB){
	// 		delete vocabDB;
	// 		vocabDB = nullptr;
	// 	}

	// 	bool useDirectIdx 	= true;//false;
	// 	int directLvls		= 4;//0;

	// 	vocabDB = new DBoW3::Database(vocab, useDirectIdx, directLvls);	//false means no direct indexing
	// 	for(int i =0; i<frames.size(); i++){
	// 		vocabDB->add(frames[i]->decs);
	// 		dbIdx2frameIdx.push_back(frames[i]->imgIdx);
	// 		frameIdx2dbIdx[frames[i]->imgIdx] = i;
	// 	}
	// 	std::cout<<*vocabDB<<std::endl;

	// 	std::cout<<"Done"<<std::endl;
	// }

	// void saveVocabDB(std::string filename){
	// 	std::cout << "saving vocabulary DB file: "<<filename<< std::endl;
	// 	if(vocabDB){
	// 		vocabDB->save(filename);
	// 	}else{
	// 		std::cout<<"vocabulary DB not exist"<<std::endl;
	// 	}
	// 	std::cout << "Done"<<std::endl;

	// 	//todo: temp code, remove
	// 	for(int i=0; i<frames.size(); i++){
	// 		frames[i]->img = cv::imread(frames[i]->imgRoot+"/"+frames[i]->imgName,cv::IMREAD_GRAYSCALE);
	// 	}
	// }

	// void loadVocabDB(std::string filename){
	// 	std::cout << "loading vocabulary DB file: "<<filename<< std::endl;
	// 	if(vocabDB){
	// 		delete vocabDB;
	// 		vocabDB = nullptr;
	// 	}
	// 	vocabDB = new DBoW3::Database(filename);
	// 	std::cout<<*vocabDB<<std::endl;
	// 	assert(vocabDB->size() == frames.size());

	// 	dbIdx2frameIdx.clear();
	// 	frameIdx2dbIdx.clear();
	// 	for(int i =0; i<frames.size(); i++){
	// 		dbIdx2frameIdx.push_back(frames[i]->imgIdx);
	// 		frameIdx2dbIdx[frames[i]->imgIdx] = i;
	// 	}
	// 	std::cout << "Done"<< std::endl;
	// }

	// inline void addToVocabDB(Frame::Ptr frame){
	// 	vocabDB->add(frame->decs);
	// 	dbIdx2frameIdx.push_back(frame->imgIdx);
	// 	frameIdx2dbIdx[frame->imgIdx] = dbIdx2frameIdx.size()-1;
	// }

	// inline const DBoW3::FeatureVector& getFeatureVecs(Frame::Ptr frame){

	// 	assert(vocabDB);

	// 	std::map<int, int>::iterator it = frameIdx2dbIdx.find(frame->imgIdx);
	// 	assert(it!=frameIdx2dbIdx.end());

	// 	std::cout<<"retrieve features for entry "<<it->second<<std::endl;
	// 	return vocabDB->retrieveFeatures(it->second);
	// }

	// inline DBoW3::FeatureVector computeFeatureVecs(Frame::Ptr frame){

	// 	assert(vocabDB);
	// 	DBoW3::BowVector bowVec;
	// 	DBoW3::FeatureVector featureVec;
	// 	const DBoW3::Vocabulary* vocab = vocabDB->getVocabulary();

	// 	std::vector<cv::Mat> vDesc;
	// 	vDesc.reserve(frame->decs.rows);
	// 	for (int j=0;j<frame->decs.rows;j++){
	// 		vDesc.push_back(frame->decs.row(j));
	// 	}

	// 	vocab->transform(vDesc,bowVec,featureVec,vocabDB->getDirectIndexLevels());

	// 	return featureVec;
	// }

	// inline std::vector<std::pair<Frame::Ptr,double> > findNBestMatchingFramesFromVocab(Frame::Ptr inFrame, const int n){

	// 	std::vector<std::pair<Frame::Ptr,double> > frameMatches;

	// 	if(vocabDB){
	// 		DBoW3::QueryResults ret;
	// 		vocabDB->query(inFrame->decs, ret, n);

	// 		for(DBoW3::QueryResults::const_iterator it = ret.begin(); it!=ret.end(); ++it){
	// 			Frame::Ptr frame = getFrame(dbIdx2frameIdx[it->Id]);
	// 			frameMatches.push_back(std::make_pair(frame,it->Score));
	// 		}
	// 	}else{
	// 		std::cout<<"vocabulary DB not exist"<<std::endl;
	// 	}

	// 	return frameMatches;
	// }

	void applyGlobalTransformation(const cv::Mat 				&transformation)
	{
		//apply tranformation to landmarks
		Eigen::Map<Eigen::Matrix<double, 3,4, Eigen::RowMajor>> 	trans((double *) transformation.data);	//no copy data
		for(std::vector<LandMark::Ptr>::iterator it = landmarks.begin(); it!=landmarks.end(); ++it){
			Eigen::Vector4d ptH 	= (*it)->pt.homogeneous();
			//ptH			 			= trans*ptH;
			(*it)->pt				= trans*ptH;//ptH.colwise().hnormalized().topRows(3);
		}

		//apply transformation to frames
		for(std::vector<Frame::Ptr>::iterator it = frames.begin(); it!=frames.end(); ++it){
			Eigen::Vector4d ptH 	= (*it)->position.homogeneous();
			(*it)->position			= trans*ptH;
			Eigen::Matrix3d R		= (*it)->rotation.conjugate().toRotationMatrix();
			R						= trans.block<3,3>(0,0)*R;
			(*it)->rotation			= Eigen::Quaterniond(R).conjugate();
			(*it)->rotation.normalize();
		}
	}

	std::vector<Frame::Ptr>			nonKeyFrames;		//record tracked frames, only pose, no measures

private:
	Data()
	// :vocabDB(nullptr)
	{}
	~Data(){
		// if(vocabDB){
		// 	delete vocabDB;
		// 	vocabDB = nullptr;
		// }
	}

	//The default Compare object of std::map is std::less<Key>, we redefine less operation
	//so that the key search is on the actual object rather than the pointer. this is due
	//to many instances of pointer could exist that points to the same object

	typedef std::map<Frame::Ptr,Measurement::Ptr, FramePtrCompare> 		MeasurePerFrame;
	typedef std::map<int,Measurement::Ptr>								MeasurePerFeature;
	typedef std::map<LandMark::Ptr, MeasurePerFrame,LandMarkPtrCompare>	MeasureLUTLandMark;
	typedef std::map<Frame::Ptr, MeasurePerFeature,FramePtrCompare>		MeasureLUTFrame;

	std::vector<Frame::Ptr> 		frames;		//no info about landmark
	std::vector<LandMark::Ptr> 		landmarks;	//no info about frame or feature
	std::vector<Measurement::Ptr>	measures;	//needed for more efficient bundle adjustment data iteration
	MeasureLUTLandMark 				landmarkMeasures;	//lookup table
	MeasureLUTFrame					frameMeasures;		//lookup table
	// DBoW3::Database					*vocabDB;			//for fast image retrival based on BOW query
	std::vector<int> 				dbIdx2frameIdx;		//the db frame index and data frame imgIdx may be different
	std::map<int,int> 				frameIdx2dbIdx;		//vise versa
};


#endif /* SRC_CORE_DATASTRUCTS_DATA_H_ */
