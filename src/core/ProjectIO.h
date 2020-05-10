/*
 * To support saving and loading sfm project
 */

#ifndef PROJECTIO_H_
#define PROJECTIO_H_

#include <string>
#include <vector>
#include <opencv2/core/core.hpp>

class PtCloud;
class PolygonModel;
class ProjectIO {
public:
	ProjectIO();
	virtual ~ProjectIO();

	static void writeProject(	const std::string				&fname,		//full path
								const std::string 				&imgRoot,
								const std::vector<std::string>	&imgNames);

	static void readProject(	const std::string			&fname,			//full path
								std::string 				&imgRoot,
								std::vector<std::string>	&imgNames);

	static void writeFeatures(	const std::string 				&featureRoot,
								const std::string				&imgName,
								const std::vector<cv::KeyPoint>	&kpts,
								const cv::Mat					&decs);

	static void readFeatures(	const std::string 				&featureRoot,
								const std::string				&imgName,
								std::vector<cv::KeyPoint>		&kpts,
								cv::Mat							&decs);

	static void writeMatches(	const std::string				&fname,
								const std::string 				&imgName1,
								const std::string				&imgName2,
								const std::vector<cv::DMatch> 	&matches);
};

#endif /* PROJECTIO_H_ */
