#ifndef PLYIO_H
#define PLYIO_H

#include <vector>
#include <string>
#include <opencv2/core/core.hpp>

class PlyIO{

public:
	PlyIO();
	virtual ~PlyIO();
	static void readPLY(	const std::string 				&path,
							std::vector<cv::Point3f> 		&xyzs);
};

#endif
