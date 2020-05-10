/*
* Unified API for various feature detector/extractors
* currently only ORB is usable
*/

#include "DetectorExtractor.h"

// #include <brisk/brisk.h>
// #include <brisk/scale-space-feature-detector.h>
// #include <brisk/harris-score-calculator.h>

#include <opencv2/imgproc/imgproc.hpp>


DetectorExtractor& DetectorExtractor::GetInstance(){
    static DetectorExtractor instance;
    return instance;
}

DetectorExtractor::DetectorExtractor()
:octaves_(3),
briskDetectionThreshold_(20),//10.0),
briskDetectionAbsoluteThreshold_(800.0),
maxFeatures_(800),//2000),
briskDescriptionRotationInvariance_(true),
briskDescriptionScaleInvariance_(true),
briskMatchingThreshold_(60.0)
{
    // detector.reset(new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
    //                                                                    briskDetectionThreshold_, octaves_,
    //                                                                    briskDetectionAbsoluteThreshold_,
	// 																   maxFeatures_));

    // detector_noOctave.reset(new brisk::ScaleSpaceFeatureDetector<brisk::HarrisScoreCalculator>(
    //                                                                    briskDetectionThreshold_, 0,
    //                                                                    briskDetectionAbsoluteThreshold_,
	// 																   maxFeatures_));

    // //detector.reset(new cv::GridAdaptedFeatureDetector(new cv::FastFeatureDetector(briskDetectionThreshold_),briskDetectionMaximumKeypoints_, 7, 4 ));
    // extractor.reset(new brisk::BriskDescriptorExtractor(briskDescriptionRotationInvariance_,briskDescriptionScaleInvariance_));


    float scalef 		= 2.0f; //1.2f;
    int edgeThreshold 	= 31;	//pixels
    int patchSize		= 31;

	detectorExtractor = cv::ORB::create(maxFeatures_,scalef, octaves_, edgeThreshold, 0, 2, cv::ORB::HARRIS_SCORE, patchSize);

	int thresh 			= 30;

	//detectorExtractor = cv::BRISK::create(thresh, octaves_, scalef);
}

DetectorExtractor::~DetectorExtractor(){
}

void DetectorExtractor::detectORB(const cv::Mat &img, std::vector<cv::KeyPoint> &kpts, const cv::Mat &mask){
	kpts.clear();
	detectorExtractor->detect(img,kpts,mask);
}
void DetectorExtractor::computeORB(const cv::Mat &img, std::vector<cv::KeyPoint> &kpts, cv::Mat &decs){
	detectorExtractor->compute(img,kpts,decs);
}
void DetectorExtractor::detectBRISK(const cv::Mat &img, std::vector<cv::KeyPoint> &kpts, const cv::Mat &mask){
	// kpts.clear();
	// //XXX:for brisk, mask is not yet supported
	// detector->detect(img,kpts, mask);
}

void DetectorExtractor::detectBRISK_hasKpts(const cv::Mat &img, std::vector<cv::KeyPoint> &kpts, const cv::Mat &mask){
	// //XXX:do not clear kpts, cos we use kpts to specify kepoints locations to detect
	// //XXX:for brisk, mask is not yet supported
	// detector_noOctave->detect(img,kpts, mask);
}

void DetectorExtractor::computeBRISK(const cv::Mat &img, std::vector<cv::KeyPoint> &kpts, cv::Mat &decs){
	// //XXX:keypoints list will be modified! keypoits for which descriptor cannot be computed are removed
	// extractor->compute(img, kpts, decs);
}

// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int DetectorExtractor::descriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}
