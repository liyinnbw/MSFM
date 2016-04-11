/*
 *  Created on: Apr 9, 2016
 *      Author: yoyo
 */

#ifndef PLYIO_H
#define PLYIO_H

#include <vector>
#include <string>
#include <opencv2/core/core.hpp>


class PtCloud;
class PlyIO{

public:
	PlyIO();

	virtual ~PlyIO();

	static void readPLY(	const std::string 				&path,
							std::vector<cv::Point3f> 		&xyzs);

	static void writePLY(	const std::string				&root,
							const std::string 				&fname,
							PtCloud 					&ptCloud);
};

#endif
