/*
 *  Created on: Apr 9, 2016
 *      Author: yoyo
 */


#include <fstream>

#include "PlyIO.h"
#include "PtCloud.h"
#include "Utils.h"

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

void PlyIO::writePLY(	const string				&root,
						const string 				&fname,
						PtCloud 				&ptCloud)
{
	const vector<Matx34d> &cameras = ptCloud.camMats;
	vector<Point3f> pts;
	ptCloud.getXYZs(pts);
	vector<Vec3b> color(pts.size(),Vec3b(255,255,255));
	string filename;
	if(fname.compare("")==0){
		string tstr;
		Utils::getTimeStampAsString(tstr);
		filename = root+"/"+tstr+".ply";
	}else{
		filename = root+"/"+fname+".ply";
	}
	cout<<filename<<endl;

	ofstream myfile;
	myfile.open (filename.c_str());

	int numCameras = cameras.size();

	//write point cloud to .ply file
	myfile <<"ply"<<endl;
	myfile <<"format ascii 1.0"<<endl;

	//write descriptors
	vector<Mat> decs;
	ptCloud.getAverageDecs(decs);
	assert(decs.size() == pts.size());
	myfile <<"comment cloudSize: "<<decs.size()<<endl;
	for(int i=0; i<decs.size(); i++){
		myfile <<"comment dec: "<<i<<" "<<decs[i]<<endl;
	}

	myfile <<"element vertex "<<pts.size()+numCameras*4<<endl;
	myfile <<"property float x"<<endl;
	myfile <<"property float y"<<endl;
	myfile <<"property float z"<<endl;
	myfile <<"property uchar red"<<endl;
	myfile <<"property uchar green"<<endl;
	myfile <<"property uchar blue"<<endl;
	myfile <<"element face 0"<<endl;
	myfile <<"property list uchar int vertex_indices"<<endl;
	myfile <<"element edge "<<numCameras*3<<endl;            // 3 axis
	myfile <<"property int vertex1"<<endl;
	myfile <<"property int vertex2"<<endl;
	myfile <<"property uchar red"<<endl;
	myfile <<"property uchar green"<<endl;
	myfile <<"property uchar blue"<<endl;
	myfile <<"end_header"<<endl;

	for (int n=0 ; n<pts.size() ; n++)
	{
			float x=pts[n].x;
			float y= pts[n].y;
			float z=pts[n].z;
			int r=color[n][0];
			int g=color[n][1];
			int b=color[n][2];
			myfile <<x<<" "<<y<<" "<<z<<" "<<b<<" "<<g<<" "<<r<<endl;
	}
	for (int n=0 ; n<numCameras ; n++)
	{
		double TRx = cameras[n](0,3);
		double TRy = cameras[n](1,3);
		double TRz = cameras[n](2,3);

		double Ix0  = cameras[n](0,0);
		double Iy0  = cameras[n](0,1);
		double Iz0  = cameras[n](0,2);

		double Jx0  = cameras[n](1,0);
		double Jy0  = cameras[n](1,1);
		double Jz0  = cameras[n](1,2);

		double Kx0  = cameras[n](2,0);
		double Ky0  = cameras[n](2,1);
		double Kz0  = cameras[n](2,2);

		double Tx  = -TRx*Ix0 -TRy*Jx0 -TRz*Kx0;
		double Ty  = -TRx*Iy0 -TRy*Jy0 -TRz*Ky0;
		double Tz  = -TRx*Iz0 -TRy*Jz0 -TRz*Kz0;

		double Ix  = Tx + cameras[n](0,0);
		double Iy  = Ty + cameras[n](0,1);
		double Iz  = Tz + cameras[n](0,2);

		double Jx  = Tx + cameras[n](1,0);
		double Jy  = Ty + cameras[n](1,1);
		double Jz  = Tz + cameras[n](1,2);

		double Kx  = Tx + cameras[n](2,0);
		double Ky  = Ty + cameras[n](2,1);
		double Kz  = Tz + cameras[n](2,2);

		myfile <<Tx<<" "<<Ty<<" "<<Tz<<" "<<255<<" "<<255<<" "<<255<<endl;
		myfile <<Ix<<" "<<Iy<<" "<<Iz<<" "<<255<<" "<<0<<" "<<0<<endl;
		myfile <<Jx<<" "<<Jy<<" "<<Jz<<" "<<0<<" "<<255<<" "<<0<<endl;
		myfile <<Kx<<" "<<Ky<<" "<<Kz<<" "<<0<<" "<<0<<" "<<255<<endl;

	}

	// draw the axis
	int  offset = (int)pts.size();
	for (int n=0 ; n<numCameras ; n++)
	{
		myfile << n*4+offset << " " << n*4+1+offset << " " << 255 << " "<< 0   << " " << 0   << endl;
		myfile << n*4+offset << " " << n*4+2+offset << " " << 0   << " "<< 255 << " " << 0   << endl;
		myfile << n*4+offset << " " << n*4+3+offset << " " << 0   << " "<< 0   << " " << 255 << endl;

	}

	myfile.close();
}
