/*
 * ProjectIO.cpp
 *
 *  Created on: Apr 9, 2016
 *      Author: yoyo
 */

#include <iostream>
#include <fstream>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "ProjectIO.h"
#include "PtCloud.h"
#include "PolygonModel.h"
#include "PathReader.h"
#include "Utils.h"

using namespace std;
using namespace cv;

ProjectIO::ProjectIO() {
	// TODO Auto-generated constructor stub

}

ProjectIO::~ProjectIO() {
	// TODO Auto-generated destructor stub
}

void ProjectIO::writeProject(	const string			&fname,
								const Mat				&camIntrinsicMat,
								const Mat 				&camDistortionMat,
								const PtCloud 			&ptCloud)
{
	string filename = fname;
	cout<<filename<<endl;
	//if .yaml
	if(Utils::endsWith(fname,".yaml")){
		FileStorage fs(filename, FileStorage::WRITE);

		fs<<"camIntrinsicMat"<<camIntrinsicMat;
		fs<<"camDistortionMat"<<camDistortionMat;
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
	//if .ply
	else if(Utils::endsWith(fname,".ply")){
		const vector<Matx34d> &cameras = ptCloud.camMats;
		vector<Point3f> pts;
		vector<Point3f> norms;
		bool saveNormal = ptCloud.hasPointNormal;
		if(saveNormal){
			ptCloud.getPointNormals(norms);
		}
		ptCloud.getXYZs(pts);
		vector<Vec3b> color(pts.size(),Vec3b(255,255,255));

		ofstream myfile;
		myfile.open (filename.c_str());

		int numCameras = cameras.size();

		//write point cloud to .ply file
		myfile <<"ply"<<endl;
		myfile <<"format ascii 1.0"<<endl;

		//write descriptors
		//vector<Mat> decs;
		//ptCloud.getAverageDecs(decs);
		//assert(decs.size() == pts.size());
		//myfile <<"comment cloudSize: "<<decs.size()<<endl;
		//for(int i=0; i<decs.size(); i++){
		//	myfile <<"comment dec: "<<i<<" "<<decs[i]<<endl;
		//}

		myfile <<"element vertex "<<pts.size()+numCameras*4<<endl;
		myfile <<"property float x"<<endl;
		myfile <<"property float y"<<endl;
		myfile <<"property float z"<<endl;
		if(saveNormal){
			myfile <<"property float nx"<<endl;
			myfile <<"property float ny"<<endl;
			myfile <<"property float nz"<<endl;
		}
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
				myfile <<x<<" "<<y<<" "<<z<<" ";
				if(saveNormal){
					float nx = norms[n].x;
					float ny = norms[n].y;
					float nz = norms[n].z;
					myfile <<nx<<" "<<ny<<" "<<nz<<" ";
				}
				myfile <<b<<" "<<g<<" "<<r<<endl;
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

			if(saveNormal){
				myfile <<Tx<<" "<<Ty<<" "<<Tz<<" "<<0<<" "<<0<<" "<<0<<" "<<255<<" "<<255<<" "<<255<<endl;
				myfile <<Ix<<" "<<Iy<<" "<<Iz<<" "<<0<<" "<<0<<" "<<0<<" "<<255<<" "<<0<<" "<<0<<endl;
				myfile <<Jx<<" "<<Jy<<" "<<Jz<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<255<<" "<<0<<endl;
				myfile <<Kx<<" "<<Ky<<" "<<Kz<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<255<<endl;
			}else{
				myfile <<Tx<<" "<<Ty<<" "<<Tz<<" "<<255<<" "<<255<<" "<<255<<endl;
				myfile <<Ix<<" "<<Iy<<" "<<Iz<<" "<<255<<" "<<0<<" "<<0<<endl;
				myfile <<Jx<<" "<<Jy<<" "<<Jz<<" "<<0<<" "<<255<<" "<<0<<endl;
				myfile <<Kx<<" "<<Ky<<" "<<Kz<<" "<<0<<" "<<0<<" "<<255<<endl;
			}

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
	//if .tiny
	else if(Utils::endsWith(fname,".tiny")){
		ofstream myfile;
		myfile.open (filename.c_str());

		//save calibration
		//XXX: assumed principle is the image center, assumed same x,y focal
		//TODO: change the small distortion to zero
		int imgW = camIntrinsicMat.at<double>(0,2)*2;
		int imgH = camIntrinsicMat.at<double>(1,2)*2;
		double f = camIntrinsicMat.at<double>(0,0);
		double fx= f/imgW;
		double fy= f/imgH;
		myfile<<"CamParam: "<<imgW<<" "<<imgH<<" "<<setprecision(17)<<fx<<" "<<fy<<" "<<0.5<<" "<<0.5<<" "<<1e-6<<endl;
		if(ptCloud.hasPointNormal)
			myfile<<"HasNormal: 1"<<endl;
		else
			myfile<<"HasNormal: 0"<<endl;
		myfile<<"ImgRoot: "<<ptCloud.imgRoot<<endl;
		//save points
		myfile<<"Points_start"<<endl;
		for(int i=0; i<ptCloud.pt3Ds.size(); i++){
			//point id
			myfile<<i<<" ";
			//point coords
			myfile	<<setprecision(17)
					<<ptCloud.pt3Ds[i].pt.x<<" "<<ptCloud.pt3Ds[i].pt.y<<" "<<ptCloud.pt3Ds[i].pt.z<<" ";
			if(ptCloud.hasPointNormal)
				myfile <<ptCloud.pt3Ds[i].norm.x<<" "<<ptCloud.pt3Ds[i].norm.y<<" "<<ptCloud.pt3Ds[i].norm.z;
			myfile 	<<endl;
		}
		myfile<<"Points_end"<<endl;

		//save camera
		myfile<<"Cams_start"<<endl;
		const map<int, int>& camMat2img 		= ptCloud.camMat2img;
		const map<int, pair<double, double> > &img2GPS = ptCloud.img2GPS;
		for(int i=0; i<ptCloud.camMats.size(); i++){
			int imgIdx = -1;
			assert(ptCloud.getImageIdxByCameraIdx(i, imgIdx));
			//cam id
			myfile<<i<<" ";

			//lat lon
			double lat, lon;
			assert(ptCloud.getImageGPS(imgIdx, lat, lon));
			myfile<<setprecision(17)<<lat<<" "<<lon<<" ";
			//image name
			string imgName = ptCloud.imgs[imgIdx];
			myfile<<imgName<<" ";
			//pose
			Mat r, t;
			ptCloud.getCamRvecAndT(i, r, t);
			myfile<<setprecision(17)<<r.at<double>(0)<<" "<<r.at<double>(1)<<" "<<r.at<double>(2)<<" "<<t.at<double>(0)<<" "<<t.at<double>(1)<<" "<<t.at<double>(2)<<" ";
			//measurements
			vector<Point2f> xys;
			vector<int>		pt3DIdxs;
			ptCloud.getImageMeasurements(imgIdx,xys,pt3DIdxs);
			assert(xys.size() == pt3DIdxs.size());
			myfile<<pt3DIdxs.size()<<" ";
			for(int j=0; j<pt3DIdxs.size(); j++){
				myfile<<setprecision(17)<<pt3DIdxs[j]<<" "<<xys[j].x<<" "<<xys[j].y<<" ";
			}
			myfile<<endl;
		}
		myfile<<"Cams_end"<<endl;



		myfile.close();
	}
	//if .sktxt
	else if(Utils::endsWith(fname,".sktxt")){
		ofstream myfile;
		myfile.open (filename.c_str());

		myfile<<"root: "<<"<put images folder here and don't forget the last slash>"<<endl;
		myfile<<"cameras: "<<ptCloud.camMats.size()<<endl;

		for(int i=0; i<ptCloud.camMats.size(); i++){
			int imgIdx;
			ptCloud.getImageIdxByCameraIdx(i, imgIdx);
			string imgName = ptCloud.imgs[imgIdx];
			double f = camIntrinsicMat.at<double>(0,0);
			double imghalfh = camIntrinsicMat.at<double>(1,2);
			double fov = (atan (imghalfh/f) * 180 / M_PI)*2;
			const Matx34d & cam = ptCloud.camMats[i];
			double TRx = cam(0,3);
			double TRy = cam(1,3);
			double TRz = cam(2,3);

			double Ix0  = cam(0,0);
			double Iy0  = cam(0,1);
			double Iz0  = cam(0,2);

			double Jx0  = cam(1,0);
			double Jy0  = cam(1,1);
			double Jz0  = cam(1,2);

			double Kx0  = cam(2,0);
			double Ky0  = cam(2,1);
			double Kz0  = cam(2,2);

			double Tx  = -TRx*Ix0 -TRy*Jx0 -TRz*Kx0;
			double Ty  = -TRx*Iy0 -TRy*Jy0 -TRz*Ky0;
			double Tz  = -TRx*Iz0 -TRy*Jz0 -TRz*Kz0;

			double Ix  = Tx + cam(0,0);
			double Iy  = Ty + cam(0,1);
			double Iz  = Tz + cam(0,2);

			double Jx  = Tx + cam(1,0);
			double Jy  = Ty + cam(1,1);
			double Jz  = Tz + cam(1,2);

			double Kx  = Tx + Kx0;
			double Ky  = Ty + Ky0;
			double Kz  = Tz + Kz0;

			myfile<<imgName<<" "<<Tx<<" "<<Ty<<" "<<Tz<<" "<<Kx<<" "<<Ky<<" "<<Kz<<" "<<-Jx0<<" "<<-Jy0<<" "<<-Jz0<<" "<<fov<<endl;

		}
		vector<cv::Point3f> pts;
		ptCloud.getXYZs(pts);
		myfile<<"points: "<<pts.size()<<endl;
		for(int i=0; i<pts.size(); i++){
			myfile<<pts[i].x<<" "<<pts[i].y<<" "<<pts[i].z<<endl;
		}

		myfile.close();
	}
}

void ProjectIO::readProject(	const string			&fname,				//including root
								Mat						&camIntrinsicMat,
								Mat 					&camDistortionMat,
								PtCloud 				&ptCloud)
{
	cout<<fname<<endl;
	ptCloud.clear();

	//if yaml
	if(Utils::endsWith(fname,".yaml")){
		ptCloud.hasPointNormal = false;
		FileStorage fs;
		fs.open(fname, FileStorage::READ);
		fs["camIntrinsicMat"]>>camIntrinsicMat;
		fs["camDistortionMat"]>>camDistortionMat;
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
				pt3D.norm = Point3f(0,0,0);
				n["pt"]>>pt3D.pt;
				FileNode nodeImg2ptIdx = (FileNode) n["img2ptIdx"];
				if (nodeImg2ptIdx.type() == FileNode::SEQ){
					for(FileNodeIterator jt = nodeImg2ptIdx.begin(); jt!= nodeImg2ptIdx.end(); jt++){
						FileNode nn = (FileNode) (*jt);
						int imgIdx 	= (int) nn["imgIdx"];
						int pt2dIdx = (int) nn["pt2dIdx"];
						pt3D.img2ptIdx[imgIdx] = pt2dIdx;
						//pt3D.img2error[imgIdx] = 0.0f;
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
	}else if(Utils::endsWith(fname,".nvm")){
		cout<<"file format: nvm"<<endl;
		ptCloud.hasPointNormal = false;
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
						double sqw = qw*qw;
						double sqx = qx*qx;
						double sqy = qy*qy;
						double sqz = qz*qz;

						// invs (inverse square length) is only required if quaternion is not already normalised
						double invs = 1 / (sqx + sqy + sqz + sqw);

						double m00 = ( sqx - sqy - sqz + sqw)*invs ; // since sqw + sqx + sqy + sqz =1/invs*invs
						double m11 = (-sqx + sqy - sqz + sqw)*invs ;
						double m22 = (-sqx - sqy + sqz + sqw)*invs ;

						double tmp1 = qx*qy;
						double tmp2 = qz*qw;
						double m10 = 2.0 * (tmp1 + tmp2)*invs ;
						double m01 = 2.0 * (tmp1 - tmp2)*invs ;

						tmp1 = qx*qz;
						tmp2 = qy*qw;
						double m20 = 2.0 * (tmp1 - tmp2)*invs ;
						double m02 = 2.0 * (tmp1 + tmp2)*invs ;
						tmp1 = qy*qz;
						tmp2 = qx*qw;
						double m21 = 2.0 * (tmp1 + tmp2)*invs ;
						double m12 = 2.0 * (tmp1 - tmp2)*invs ;

						Matx33d R_matx(	m00, m01, m02,
										m10, m11, m12,
										m20, m21, m22);

						//Matx33d R_matx(	1-2*qy*qy-2*qz*qz, 2*qx*qy - 2*qz*qw, 2*qx*qz + 2*qy*qw,
						//				2*qx*qy + 2*qz*qw, 1-2*qx*qx-2*qz*qz, 2*qy*qz - 2*qx*qw,
						//				2*qx*qz - 2*qy*qw, 2*qy*qz + 2*qx*qw, 1-2*qx*qx-2*qy*qy);

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
							pt3D.norm = Point3f(0,0,0);
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
								pt2D.dec = Mat(1,32, CV_8UC1, Scalar(0)); // dummy orb descriptor
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

	}else if(Utils::endsWith(fname,".patch")){
		cout<<"file format: patch"<<endl;
		ptCloud.hasPointNormal = true;
		ifstream infile(fname.c_str());
		string line;
		bool readIntrinsics = false;
		while (getline(infile, line)){
			if(line.find("IMGROOT")!=std::string::npos){
				getline(infile, line);
				istringstream iss(line);
				iss>>ptCloud.imgRoot;
				cout<<"imgRoot: "<<ptCloud.imgRoot<<endl;
				//PathReader::readPaths(ptCloud.imgRoot,ptCloud.imgs);
			}else if(line.find("CAMERA_START")!=std::string::npos){
				cout<<"camera start"<<endl;
				getline(infile, line);
				while(line.find("CAMERA_END")==std::string::npos){
					//cout<<line;
					if(line.find(".jpg")!=std::string::npos){
						//get image name and idx
						istringstream iss0(line);
						string imgName;
						iss0>>imgName;
						int imgIdx = Utils::extractInt(line);

						//get original image name
						getline(infile, line);
						istringstream iss00(line);
						string originalImgName;
						iss00>>originalImgName;

						//get focal length
						getline(infile, line);
						istringstream iss(line);
						double f;
						iss>>f;

						//get principle
						getline(infile, line);
						istringstream iss2(line);
						double ppx, ppy;
						iss2>>ppx>>ppy;

						//get camera translation
						getline(infile, line);
						istringstream iss3(line);
						double t0,t1,t2;
						iss3>>t0>>t1>>t2;

						//ignore lines
						getline(infile, line);
						getline(infile, line);
						getline(infile, line);

						//get camera rotation
						getline(infile, line);
						istringstream iss4(line);
						double r00,r01,r02;
						iss4>>r00>>r01>>r02;
						getline(infile, line);
						istringstream iss5(line);
						double r10,r11,r12;
						iss5>>r10>>r11>>r12;
						getline(infile, line);
						istringstream iss6(line);
						double r20,r21,r22;
						iss6>>r20>>r21>>r22;

						//get distortion
						getline(infile, line);
						istringstream iss7(line);
						double r;
						iss7>>r;

						//ignore line
						getline(infile, line);

						//save camera intrinsics if not done so
						if(!readIntrinsics){
							double camMatArr[9] 		= { f, 			0.0, 		ppx,
															0.0, 		f, 			ppy,
															0.0,		0.0,		1.0 };
							double distortCoeffArr[5] 	= { r, 0, 0, 0, 0 };
							Mat CM(3, 3, CV_64F, camMatArr);
							Mat DM(1, 5, CV_64F, distortCoeffArr);
							camIntrinsicMat = CM.clone();	//must copy the data else they will be destroyed after constructor
							camDistortionMat= DM.clone();	//must copy the data else they will be destroyed after constructor
							cout<<camIntrinsicMat<<endl;
							cout<<camDistortionMat<<endl;
							readIntrinsics = true;
						}

						//add camMat
						Matx34d cm(	r00,r01,r02,t0,
									r10,r11,r12,t1,
									r20,r21,r22,t2);
						ptCloud.imgs.push_back(originalImgName);//imgName);
						ptCloud.camMats.push_back(cm);
						ptCloud.img2camMat[imgIdx] = imgIdx;
						ptCloud.camMat2img[imgIdx] = imgIdx;
						cout<<"read img & camera ["<<imgIdx<<"] "<<imgName<<endl;
					}
					getline(infile, line);
				}
				cout<<ptCloud.imgs.size()<<" images read"<<endl;
				cout<<ptCloud.camMats.size()<<" cameras read"<<endl;
			}else if(line.find("PATCHS")!=std::string::npos){
				//get 3d coordinates
				getline(infile, line);
				istringstream iss(line);
				double x,y,z;
				iss>>x>>y>>z;
				Pt3D pt3D;
				pt3D.pt = Point3f(x,y,z);

				//get point normal
				getline(infile, line);
				istringstream iss1(line);
				double nx, ny, nz;
				iss1>>nx>>ny>>nz;
				pt3D.norm = Point3f(nx,ny,nz);	//by right this should already been normalized

				//ignore a line
				getline(infile, line);

				//get number of measurements
				getline(infile, line);
				int numMeasures;
				istringstream iss2(line);
				iss2>>numMeasures;

				//get measurements
				getline(infile, line);
				istringstream iss3(line);
				while(numMeasures-->0){
					int imgIdx;
					double x_2d, y_2d;
					iss3>>imgIdx>>x_2d>>y_2d;
					Pt2D pt2D;
					pt2D.pt = Point2f(x_2d, y_2d);
					pt2D.img_idx = imgIdx;
					pt2D.pt3D_idx = ptCloud.pt3Ds.size();
					pt2D.dec = Mat(1,32, CV_8UC1, Scalar(0)); // dummy orb descriptor
					ptCloud.img2pt2Ds[imgIdx].push_back(pt2D);
					assert(pt3D.img2ptIdx.find(imgIdx) == pt3D.img2ptIdx.end());
					pt3D.img2ptIdx[imgIdx] = ptCloud.img2pt2Ds[imgIdx].size()-1;
				}

				//get number of visible measurements
				getline(infile, line);
				istringstream iss4(line);
				iss4>>numMeasures;

				//get measurements
				getline(infile, line);
				istringstream iss5(line);
				while(numMeasures-->0){
					int imgIdx;
					double x_2d, y_2d;
					iss5>>imgIdx>>x_2d>>y_2d;
					Pt2D pt2D;
					pt2D.pt = Point2f(x_2d, y_2d);
					pt2D.img_idx = imgIdx;
					pt2D.pt3D_idx = ptCloud.pt3Ds.size();
					pt2D.dec = Mat(1,32, CV_8UC1, Scalar(0)); // dummy orb descriptor
					ptCloud.img2pt2Ds[imgIdx].push_back(pt2D);
					assert(pt3D.img2ptIdx.find(imgIdx) == pt3D.img2ptIdx.end());
					pt3D.img2ptIdx[imgIdx] = ptCloud.img2pt2Ds[imgIdx].size()-1;
				}
				ptCloud.pt3Ds.push_back(pt3D);
			}

		}
		cout<<ptCloud.pt3Ds.size()<<" 3D points read"<<endl;

	}else if(Utils::endsWith(fname,".tiny")){
		cout<<"file format: patch"<<endl;
		ifstream infile(fname.c_str());
		string line;
		while (getline(infile, line)){
			if(line.find("CamParam:")!=line.npos){
				istringstream iss(line);
				string nouse;
				int imgW,imgH;
				double fx,fy,ppx,ppy,r;
				iss>>nouse>>imgW>>imgH>>fx>>fy>>ppx>>ppy>>r;
				double camMatArr[9] 		= { fx*imgW, 	0.0, 		ppx*imgW,
												0.0, 		fy*imgH, 	ppy*imgH,
												0.0,		0.0,		1.0 };
				double distortCoeffArr[5] 	= { 0, 0, 0, 0, 0 };	//assume no distortion
				Mat CM(3, 3, CV_64F, camMatArr);
				Mat DM(1, 5, CV_64F, distortCoeffArr);
				camIntrinsicMat = CM.clone();	//must copy the data else they will be destroyed after constructor
				camDistortionMat= DM.clone();	//must copy the data else they will be destroyed after constructor
				cout<<camIntrinsicMat<<endl;
				cout<<camDistortionMat<<endl;

			}else if(line.find("ImgRoot:")!=line.npos){
				istringstream iss(line);
				string nouse;
				iss>>nouse>>ptCloud.imgRoot;
			}else if(line.find("HasNormal:")!=line.npos){
				istringstream iss(line);
				string nouse;
				iss>>nouse>>ptCloud.hasPointNormal;
			}else if(line == "Points_start"){
				string line2;
				getline(infile, line2);
				while(line2!="Points_end"){
					istringstream iss(line2);

					//get 3d coordinates
					int id;
					double x,y,z;
					iss>>id>>x>>y>>z;
					Pt3D pt3D;
					pt3D.pt = Point3f(x,y,z);

					//get point normal
					if(ptCloud.hasPointNormal){
						double nx, ny, nz;
						iss>>nx>>ny>>nz;
						pt3D.norm = Point3f(nx,ny,nz);	//by right this should already been normalized
					}

					ptCloud.pt3Ds.push_back(pt3D);
					getline(infile, line2);
				}
				cout<<"total "<<ptCloud.pt3Ds.size()<<" points read"<<endl;

			}else if(line == "Cams_start"){
				string line2;
				getline(infile, line2);
				while(line2!="Cams_end"){
					istringstream iss(line2);
					int id, measureCnt;
					double lat, lon;
					double r[3];
					double t[3];
					string imgName;
					iss>>id>>lat>>lon>>imgName>>r[0]>>r[1]>>r[2]>>t[0]>>t[1]>>t[2]>>measureCnt;

					//cam mat
					Mat rvec = Mat(3,1,CV_64F,r);
					Mat R;
					Rodrigues(rvec, R);
					Matx34d camMat = Matx34d(	R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2), t[0],
												R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2), t[1],
												R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2), t[2]);

					int imgIdx = ptCloud.imgs.size();
					int camIdx = ptCloud.camMats.size();

					//add image, camMat
					ptCloud.imgs.push_back(imgName);
					ptCloud.camMats.push_back(camMat);

					//add data associations
					ptCloud.img2camMat[imgIdx] = camIdx;
					ptCloud.camMat2img[camIdx] = imgIdx;

					//add GPS
					ptCloud.img2GPS[imgIdx] = make_pair(lat,lon);

					//add measures
					ptCloud.img2pt2Ds[imgIdx] = vector<Pt2D>();
					while(measureCnt-->0){
						int pt3DIdx;
						float x,y;
						iss>>pt3DIdx>>x>>y;
						Pt2D pt2D;
						pt2D.pt = Point2f(x, y);
						pt2D.img_idx = imgIdx;
						pt2D.pt3D_idx = pt3DIdx;
						pt2D.dec = Mat(1,32, CV_8UC1, Scalar(0)); // dummy orb descriptor
						ptCloud.img2pt2Ds[imgIdx].push_back(pt2D);
						assert(ptCloud.pt3Ds[pt3DIdx].img2ptIdx.find(imgIdx) == ptCloud.pt3Ds[pt3DIdx].img2ptIdx.end());
						ptCloud.pt3Ds[pt3DIdx].img2ptIdx[imgIdx] = ptCloud.img2pt2Ds[imgIdx].size()-1;
					}

					getline(infile, line2);
				}
				cout<<"total "<<ptCloud.imgs.size()<<" images & cameras read"<<endl;
			}
		}
	}

	if(!Utils::endsWith(fname,".tiny")){
		for(int i=0; i<ptCloud.camMats.size(); i++){
			int imgIdx = ptCloud.camMat2img[i];
			ptCloud.img2GPS[imgIdx] = make_pair(0,0);	//dummy
		}
	}

	ptCloud.removeRedundancy();
	cout<<"stats:"<<endl;
	cout<<"cloud points = "<<ptCloud.pt3Ds.size()<<endl;
	cout<<"images       = "<<ptCloud.imgs.size()<<endl;
	cout<<"cameras      = "<<ptCloud.camMats.size()<<endl;

}

void ProjectIO::readGPS(	const std::string			&fname,
							PtCloud						&ptCloud)
{
	//if csv
	cout<<fname<<endl;
	if(Utils::endsWith(fname,".csv")){
		int currLine				= 0;
		ifstream infile(fname.c_str());
		string line;
		while (getline(infile, line)){
			if(currLine>0){
				std::replace( line.begin(), line.end(), ',', ' ');
				istringstream iss(line);
				string imgName;
				double lat,lon,lat_ignore,lon_ignore,id;
				iss>>lon>>lat>>lon_ignore>>lat_ignore>>id>>imgName;	//XXX: note the order is (lon,lat)
				for(int i=0; i<ptCloud.imgs.size(); i++){

					//if (ptCloud.imgs[i].compare(imgName) == 0){
					if (ptCloud.imgs[i]==imgName){
						ptCloud.img2GPS[i] = make_pair(lat, lon);
					}
				}
			}
			currLine++;
		}
		for(map<int, pair<double,double> >::iterator it = ptCloud.img2GPS.begin(); it!=ptCloud.img2GPS.end(); ++it){
			cout<<setprecision(20)<<ptCloud.imgs[it->first]<<" ("<<it->second.first<<","<<it->second.second<<")"<<endl;
		}
	}
}
void ProjectIO::readPolygon(	const std::string			&fname,
								PolygonModel				&poly)
{
	cout<<"reading polydata"<<endl;
	poly.reset();
	ifstream infile(fname.c_str());
	string line;
	int numVerts	= 0;
	int numFaces	= 0;
	while (getline(infile, line)){
		istringstream iss(line);
		string s;
		iss>>s;
		if(s.compare("element")==0){
			iss>>s;
			if(s.compare("vertex")==0){
				iss>>numVerts;
			}else if(s.compare("face")==0){
				iss>>numFaces;
			}
		}
		if(s.compare("end_header")==0){
			break;
		}
	}

	poly.verts.reserve(numVerts);
	poly.faces.reserve(numFaces);
	for(int i=0; i<numVerts; i++){
		getline(infile, line);
		istringstream iss(line);
		float x,y,z;
		iss>>x>>y>>z;
		poly.verts.push_back(Point3f(x,y,z));
	}
	cout<<poly.verts.size()<<" vertices read"<<endl;
	for(int i=0; i<numFaces; i++){
		getline(infile, line);
		istringstream iss(line);
		int cnt, a,b,c;
		iss>>cnt>>a>>b>>c;
		poly.faces.push_back(Point3i(a,b,c));
	}
	cout<<poly.faces.size()<<" faces read"<<endl;
	infile.close();
}
