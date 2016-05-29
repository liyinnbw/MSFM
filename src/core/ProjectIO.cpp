/*
 * ProjectIO.cpp
 *
 *  Created on: Apr 9, 2016
 *      Author: yoyo
 */

#include <iostream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "ProjectIO.h"
#include "PtCloud.h"
#include "Utils.h"

using namespace std;
using namespace cv;

ProjectIO::ProjectIO() {
	// TODO Auto-generated constructor stub

}

ProjectIO::~ProjectIO() {
	// TODO Auto-generated destructor stub
}

void ProjectIO::writeProject(	const string			&root,
								const string			&fname,
								const Mat				&camIntrinsicMat,
								const Mat 				&camDistortionMat,
								const int 				&lastAddedImgIdx,
								const PtCloud 			&ptCloud)
{
	string filename;

	//if empty file name, using timestamp
	if(fname.compare("")==0){
		string tstr;
		Utils::getTimeStampAsString(tstr);
		filename = root+"/"+tstr+".yaml";
	}else{
		filename = root+"/"+fname+".yaml";
	}
	cout<<filename<<endl;

	FileStorage fs(filename, FileStorage::WRITE);

	fs<<"camIntrinsicMat"<<camIntrinsicMat;
	fs<<"camDistortionMat"<<camDistortionMat;
	fs<<"lastAddedImgIdx"<<lastAddedImgIdx;
	fs<<"imgRoot"<<ptCloud.imgRoot;
	fs<<"imgs"<<"[";
	for(int i=0; i<ptCloud.imgs.size(); i++){
		fs<<ptCloud.imgs[i];
	}
	fs<<"]";
	cout<<ptCloud.imgs.size()<<" images written"<<endl;

	//camera projection matrixs
	fs<<"img2camMat"<<"[";
	const map<int, int>& img2camMat = ptCloud.img2camMat;
	for(map<int, int>::const_iterator it = img2camMat.begin(); it != img2camMat.end(); it++){
		fs << "{";
		fs << "imgIdx" << (*it).first;
		const Matx34d& camProjMat = ptCloud.camMats[(*it).second];
		fs << "camProjMat"<< Mat(camProjMat);
		fs << "}";
	}
	fs<<"]";
	cout<<ptCloud.camMats.size()<<" cameras written"<<endl;

	//2d features
	fs<<"img2pt2Ds"<<"[";
	const map<int, vector<Pt2D> >& img2pt2Ds = ptCloud.img2pt2Ds;
	for(map<int, vector<Pt2D> >::const_iterator it = img2pt2Ds.begin(); it!=img2pt2Ds.end(); it++){
		const int imgIdx = (*it).first;
		const vector<Pt2D> &pt2Ds = (*it).second;
		fs<<"{";
			fs<<"imgIdx"<<imgIdx;
			fs<<"pt2Ds"<<"[";
			for (int i=0; i<pt2Ds.size(); i++){
				fs<<"{";
				fs<<"pt"<<pt2Ds[i].pt;
				fs<<"dec"<<pt2Ds[i].dec;
				fs<<"}";
			}
			fs<<"]";
		fs<<"}";
	}
	fs<<"]";
	cout<<ptCloud.img2pt2Ds.size()<<" image feature sets written"<<endl;

	//3d points
	fs<<"pt3Ds"<<"[";
	for(int i=0; i<ptCloud.pt3Ds.size(); i++){
		const Pt3D &pt3D = ptCloud.pt3Ds[i];
		const map<int,int>& img2ptIdx = pt3D.img2ptIdx;
		fs<<"{";
			fs<<"pt"<<pt3D.pt;

			fs<<"img2ptIdx"<<"[";
			for(map<int,int>::const_iterator it = img2ptIdx.begin(); it != img2ptIdx.end(); it++ ){
				fs <<"{";
				fs <<"imgIdx"  << (*it).first;
				fs <<"pt2dIdx" << (*it).second;
				fs <<"}";
			}
			fs<<"]";

		fs<<"}";
	}
	fs<<"]";
	cout<<ptCloud.pt3Ds.size()<<" 3D points written"<<endl;

	fs.release();
}

void ProjectIO::readProject(	const string			&fname,				//including root
								Mat						&camIntrinsicMat,
								Mat 					&camDistortionMat,
								int 					&lastAddedImgIdx,
								PtCloud 				&ptCloud)
{
	cout<<fname<<endl;
	ptCloud.clear();

	//if yaml
	if(fname.find("yaml")!=string::npos){
		FileStorage fs;
		fs.open(fname, FileStorage::READ);
		fs["camIntrinsicMat"]>>camIntrinsicMat;
		fs["camDistortionMat"]>>camDistortionMat;
		fs["lastAddedImgIdx"]>>lastAddedImgIdx;
		fs["imgRoot"]>>ptCloud.imgRoot;

		//images
		FileNode nodeImgs = fs["imgs"];
		if (nodeImgs.type() == FileNode::SEQ){
			ptCloud.imgs.reserve(nodeImgs.size());
			for(FileNodeIterator it = nodeImgs.begin(); it!= nodeImgs.end(); it++){
				ptCloud.imgs.push_back((string)*it);
			}
		}
		cout<<ptCloud.imgs.size()<<" images read"<<endl;

		//cameras
		FileNode nodeImg2camMat = fs["img2camMat"];
		if (nodeImg2camMat.type() == FileNode::SEQ){
			ptCloud.camMats.reserve(nodeImg2camMat.size());
			for(FileNodeIterator it = nodeImg2camMat.begin(); it!= nodeImg2camMat.end(); it++){
				FileNode n = (FileNode) (*it);
				int imgIdx = (int) n["imgIdx"];
				Mat m;
				n["camProjMat"]>>m;
				Matx34d camMat((double*)m.ptr());
				ptCloud.camMats.push_back(camMat);
				ptCloud.img2camMat[imgIdx] = ptCloud.camMats.size()-1;
				ptCloud.camMat2img[ptCloud.camMats.size()-1] = imgIdx;
			}
		}
		cout<<ptCloud.camMats.size()<<" cameras read"<<endl;

		//2d points
		FileNode nodeImg2pt2Ds = fs["img2pt2Ds"];
		if (nodeImg2pt2Ds.type() == FileNode::SEQ){
			for(FileNodeIterator it = nodeImg2pt2Ds.begin(); it!= nodeImg2pt2Ds.end(); it++){
				FileNode n = (FileNode) (*it);
				int imgIdx = (int) n["imgIdx"];
				ptCloud.img2pt2Ds[imgIdx] = vector<Pt2D>();
				FileNode pt2Ds = (FileNode) n["pt2Ds"];
				if (pt2Ds.type() == FileNode::SEQ){
					ptCloud.img2pt2Ds[imgIdx].reserve(pt2Ds.size());
					for(FileNodeIterator jt = pt2Ds.begin(); jt!= pt2Ds.end(); jt++){
						FileNode nn = (FileNode) (*jt);
						Pt2D pt2D;
						nn["pt"]>>pt2D.pt;
						nn["dec"]>>pt2D.dec;
						pt2D.img_idx = imgIdx;
						pt2D.pt3D_idx= -1;
						ptCloud.img2pt2Ds[imgIdx].push_back(pt2D);
					}
				}
				cout<<"img "<<imgIdx<<" "<<ptCloud.img2pt2Ds[imgIdx].size()<<" features read"<<endl;
			}
		}
		cout<<ptCloud.img2pt2Ds.size()<<" 2D feature sets read"<<endl;

		//3d points
		FileNode nodePt3Ds = fs["pt3Ds"];
		if (nodePt3Ds.type() == FileNode::SEQ){
			ptCloud.pt3Ds.reserve(nodePt3Ds.size());
			for(FileNodeIterator it = nodePt3Ds.begin(); it!= nodePt3Ds.end(); it++){
				FileNode n = (FileNode) (*it);
				Pt3D pt3D;
				n["pt"]>>pt3D.pt;
				FileNode nodeImg2ptIdx = (FileNode) n["img2ptIdx"];
				if (nodeImg2ptIdx.type() == FileNode::SEQ){
					for(FileNodeIterator jt = nodeImg2ptIdx.begin(); jt!= nodeImg2ptIdx.end(); jt++){
						FileNode nn = (FileNode) (*jt);
						int imgIdx 	= (int) nn["imgIdx"];
						int pt2dIdx = (int) nn["pt2dIdx"];
						pt3D.img2ptIdx[imgIdx] = pt2dIdx;
						pt3D.img2error[imgIdx] = 0.0f;
						//link up image 2d features
						assert(imgIdx == ptCloud.img2pt2Ds[imgIdx][pt2dIdx].img_idx);
						ptCloud.img2pt2Ds[imgIdx][pt2dIdx].pt3D_idx = ptCloud.pt3Ds.size(); //note, this pt3D has not been pushed to cloud yet
					}
				}
				ptCloud.pt3Ds.push_back(pt3D);
			}
		}
		cout<<ptCloud.pt3Ds.size()<<" 3D points read"<<endl;


		fs.release();
	}else if(fname.find("nvm")!=string::npos){
		cout<<"file format: nvm"<<endl;

		const int LINE_INTRINSICS	= 1;
		const int LINE_IMGROOT		= 2;
		const int LINE_IMGCNT		= 3;
		int LINE_PTCNT				= 10000000;
		int currLine				= 0;
		int imgW_Half				= -1;
		int imgH_Half				= -1;

		ifstream infile(fname.c_str());
		string line;
		while (getline(infile, line)){
			currLine++;
			switch (currLine){
				case LINE_INTRINSICS:
				{
					istringstream iss(line);
					string version,infoType;
					double fx,ppx,fy,ppy,r;	//r for radio distortion
					iss>>version>>infoType>>fx>>ppx>>fy>>ppy>>r;
					double camMatArr[9] 		= { fx, 		0.0, 		ppx,
													0.0, 		fy, 		ppy,
													0.0,		0.0,		1.0 };
					double distortCoeffArr[5] 	= { r, 0, 0, 0, 0 };
					Mat CM(3, 3, CV_64F, camMatArr);
					Mat DM(1, 5, CV_64F, distortCoeffArr);
					camIntrinsicMat = CM.clone();	//must copy the data else they will be destroyed after constructor
					camDistortionMat= DM.clone();	//must copy the data else they will be destroyed after constructor
					cout<<camIntrinsicMat<<endl;
					cout<<camDistortionMat<<endl;
				}
					break;
				case LINE_IMGROOT:
				{
					istringstream iss(line);
					iss>>ptCloud.imgRoot;
					cout<<"imgRoot: "<<ptCloud.imgRoot<<endl;
				}
					break;
				case LINE_IMGCNT:
				{
					istringstream iss(line);
					int imgcnt;
					iss>>imgcnt;
					LINE_PTCNT = LINE_IMGCNT+imgcnt+2;
					ptCloud.imgs.reserve(imgcnt);
					ptCloud.camMats.reserve(imgcnt);
					for(int i = 0; i<imgcnt; i++){
						getline(infile, line);
						currLine++;
						istringstream iss2(line);
						string imgName;
						double f,qw,qx,qy,qz,t0,t1,t2,r;

						iss2>>imgName>>f>>qw>>qx>>qy>>qz>>t0>>t1>>t2>>r;
						//cout<<imgName<<" "<<f<<" "<<qw<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<t0<<" "<<t1<<" "<<t2<<" "<<r<<endl;
						if(imgW_Half <0){
							Mat tmp = imread(ptCloud.imgRoot+"/"+imgName,IMREAD_COLOR);
							imgW_Half 	= tmp.cols/2;
							imgH_Half	= tmp.rows/2;
							cout<<"img size = "<<imgW_Half*2<<"x"<<imgH_Half*2<<endl;
						}
						Matx33d R_matx(	1-2*qy*qy-2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw,
										2*qx*qy + 2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz - 2*qx*qw,
										2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1-2*qx*qx-2*qy*qy);
						Matx31d C_matx(t0,
										t1,
										t2);
						Mat t = -1*Mat(R_matx)*Mat(C_matx);

						Matx34d cm(R_matx(0,0),R_matx(0,1),R_matx(0,2),t.at<double>(0),
									R_matx(1,0),R_matx(1,1),R_matx(1,2),t.at<double>(1),
									R_matx(2,0),R_matx(2,1),R_matx(2,2),t.at<double>(2));


						ptCloud.imgs.push_back(imgName);
						ptCloud.camMats.push_back(cm);
						ptCloud.img2camMat[i] = i;
						ptCloud.camMat2img[i] = i;

					}
					cout<<ptCloud.camMats.size()<<" images and cameras read"<<endl;
				}
					break;
				default:
				{
					if(currLine == LINE_PTCNT){
						istringstream iss(line);
						int ptcnt;
						iss>>ptcnt;
						//3d points
						ptCloud.pt3Ds.reserve(ptcnt);
						for(int i=0; i<ptcnt; i++){
							getline(infile, line);
							currLine++;
							istringstream iss2(line);
							double x,y,z;
							int r,g,b;
							int measurementCnt;
							iss2>>x>>y>>z>>r>>g>>b>>measurementCnt;
							Pt3D pt3D;
							pt3D.pt = Point3f(x,y,z);
							//2D measurements
							while(measurementCnt-->0){
								int imgIdx, pt2DIdx;	//pt2DIdx is useless, we will create our own idx
								double x_2d, y_2d;
								iss2>>imgIdx>>pt2DIdx>>x_2d>>y_2d;
								//XXX: NVM file 2d coordinates are relative to image center (not principle point) rather than top-left!
								//convert coordinates to top-left
								x_2d+=imgW_Half;
								y_2d+=imgH_Half;
								Pt2D pt2D;
								pt2D.pt = Point2f(x_2d, y_2d);
								pt2D.img_idx = imgIdx;
								pt2D.pt3D_idx = ptCloud.pt3Ds.size();
								//leave out descriptor field
								ptCloud.img2pt2Ds[imgIdx].push_back(pt2D);
								assert(pt3D.img2ptIdx.find(imgIdx) == pt3D.img2ptIdx.end());
								pt3D.img2ptIdx[imgIdx] = ptCloud.img2pt2Ds[imgIdx].size()-1;
							}
							ptCloud.pt3Ds.push_back(pt3D);
						}
						cout<<ptCloud.pt3Ds.size()<<" 3D points read"<<endl;

					}
				}
					break;
			}

		}
		infile.close();

	}
}
