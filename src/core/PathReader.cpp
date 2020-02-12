/*
 * PathReader.cpp
 *
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#include <dirent.h>
#include <cstring>
#include <cstdio>

#include "PathReader.h"
#include "Utils.h"


using namespace std;
PathReader::PathReader() {
	// TODO Auto-generated constructor stub

}

PathReader::~PathReader() {
	// TODO Auto-generated destructor stub
}
void PathReader::readPaths(	const string 				&root,
							vector<std::string> 		&paths){
	string rootPath = root;
	paths.clear();
	DIR *dir;
	struct dirent *ent;
	vector<string> imgExts;
	imgExts.push_back("JPG"); imgExts.push_back("jpg");
	imgExts.push_back("PNG"); imgExts.push_back("png");

	if ((dir = opendir (root.c_str())) != NULL) {
		vector<string> tmpPaths;
		while ((ent = readdir (dir)) != NULL) {
			if(strcmp(ent->d_name,".")==0 || strcmp(ent->d_name,"..")==0) continue;
			string fileName = string(ent->d_name);
			for(int i=0; i<imgExts.size(); i++){
				if(Utils::endsWith(fileName,imgExts[i])){
					tmpPaths.push_back(fileName);
				}
			}
		}
		//sort file name in ascending order
		while(!tmpPaths.empty()){
			vector<string>::iterator minFileName = tmpPaths.begin();
			for(vector<string>::iterator i= tmpPaths.begin();i!=tmpPaths.end();i++){
				if(Utils::extractInt(*i)< Utils::extractInt(*minFileName)) minFileName=i;
			}
			//paths.push_back(rootPath+"/"+*minFileName);
			paths.push_back(*minFileName);
			tmpPaths.erase(minFileName);
		}
		closedir(dir);
	} else {
	  /* could not open directory */
	  perror ("");
	}
}
