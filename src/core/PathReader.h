/*
 * PathReader.h
 *
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#ifndef PATHREADER_H_
#define PATHREADER_H_
#include <vector>
#include <string>

class PathReader {
public:
	PathReader();
	virtual ~PathReader();
	void static readPaths(	const std::string 				&root,
							std::vector<std::string> 		&paths
							);
};

#endif /* PATHREADER_H_ */
