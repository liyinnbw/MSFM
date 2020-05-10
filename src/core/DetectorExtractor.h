/*
* Unified API for various feature detector/extractors
* currently only ORB is usable
*/

#ifndef DetectorExtractor_hpp
#define DetectorExtractor_hpp

#include <opencv2/opencv.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <memory>
#include <vector>

class DetectorExtractor
{
public:

    static DetectorExtractor& GetInstance();
    static int descriptorDistance(const cv::Mat &a, const cv::Mat &b);
    
    void detectORB(const cv::Mat &img, std::vector<cv::KeyPoint> &kpts, const cv::Mat &mask = cv::Mat());
    void computeORB(const cv::Mat &img, std::vector<cv::KeyPoint> &kpts, cv::Mat &decs);
    void detectBRISK(const cv::Mat &img, std::vector<cv::KeyPoint> &kpts, const cv::Mat &mask = cv::Mat());
    void detectBRISK_hasKpts(const cv::Mat &img, std::vector<cv::KeyPoint> &kpts, const cv::Mat &mask = cv::Mat());
    void computeBRISK(const cv::Mat &img, std::vector<cv::KeyPoint> &kpts, cv::Mat &decs);

private:
    DetectorExtractor();
    virtual ~DetectorExtractor();

    int             octaves_;
	double          briskDetectionThreshold_ ;
	double          briskDetectionAbsoluteThreshold_;
	int             maxFeatures_;
	bool            briskDescriptionRotationInvariance_;
	bool            briskDescriptionScaleInvariance_;
	double          briskMatchingThreshold_;

	std::shared_ptr<cv::FeatureDetector>        detector;
	std::shared_ptr<cv::FeatureDetector>        detector_noOctave;
	std::shared_ptr<cv::DescriptorExtractor>    extractor;
	cv::Ptr<cv::Feature2D> 						detectorExtractor;

};

#endif /* DetectorExtractor_hpp */
