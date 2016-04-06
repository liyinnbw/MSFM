#include "PlyIO.h"
#include <fstream>

using namespace std;
using namespace cv;
PlyIO::PlyIO(){
}
PlyIO::~PlyIO() {
}

void PlyIO::readPLY(	const string 			&path,
						vector<Point3f> 		&xyzs){
	xyzs.clear();

	ifstream infile(path.c_str());
	string line;
	int numVertex	= 0;
	while (getline(infile, line)){
		istringstream iss(line);
		string s;
		iss>>s;
		if(s.compare("element")==0){
			iss>>s;
			if(s.compare("vertex")==0){
				iss>>numVertex;
			}
		}
		if(s.compare("end_header")==0){
			break;
		}
	}

	xyzs.reserve(numVertex);
	for(int i=0; i<numVertex; i++){
		getline(infile, line);
		istringstream iss(line);
		float x,y,z;
		int r,g,b;
		iss>>x>>y>>z>>r>>g>>b;
		xyzs.push_back(Point3f(x,y,z));
	}
}
