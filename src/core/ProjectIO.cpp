/*
 * To support saving and loading sfm project
 */

#include <iostream>
#include <fstream>
#include <iomanip>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "ProjectIO.h"
#include "Utils.h"
#include "Camera.h"
#include "datastructs/Data.h"

using namespace std;
using namespace cv;
using namespace Eigen;


static bool KptLmkPairComparator(const pair<KeyPoint, LandMark::Ptr> &i, const pair<KeyPoint, LandMark::Ptr> &j)
{

	if(i.first.pt.y<j.first.pt.y){
		return true;
	}else if(i.first.pt.y>j.first.pt.y){
		return false;
	}else{
		if(i.first.pt.x<j.first.pt.x){
			return true;
		}else{
			return false;
		}
	}

}

ProjectIO::ProjectIO() {
	// TODO Auto-generated constructor stub

}

ProjectIO::~ProjectIO() {
	// TODO Auto-generated destructor stub
}

void ProjectIO::writeProject(	const string			&fname,
								const string 			&imgRoot,
								const vector<string>	&imgNames)
{
	string filename = fname;
	cout<<filename<<endl;
	Camera &camera = Camera::GetInstance();
	Data &data = Data::GetInstance();
	data.deleteTrashes();	//important

	const vector<Frame::Ptr> &frames 	= data.getFrames();
	const vector<LandMark::Ptr> &lms	= data.getLandMarks();
	const vector<Measurement::Ptr> &ms= data.getMeasurements();


	//if .yaml
	if(Utils::endsWith(fname,".yaml")){
		FileStorage fs(filename, FileStorage::WRITE);
		fs<<"imgW"<<camera.w;
		fs<<"imgH"<<camera.h;
		fs<<"camIntrinsicMat"<<camera.camMat;
		fs<<"camDistortionMat"<<camera.distortionMat;
		fs<<"imgRoot"<<imgRoot;

		//imgs, including nonused ones
		fs<<"imgs"<<"[";
		for(vector<string>::const_iterator it = imgNames.begin(); it!=imgNames.end(); ++it){
			fs<<(*it);
		}
		fs<<"]";
		cout<<frames.size()<<" images written"<<endl;

		//frames
		fs<<"frames"<<"[";
		for(vector<Frame::Ptr>::const_iterator it = frames.begin(); it!=frames.end(); ++it){
			const vector<KeyPoint> 	&kpts = (*it)->kpts;
			const Mat				&decs = (*it)->decs;
			fs << "{";
			fs << "imgIdx"		<< (*it)->imgIdx;
			fs << "imgRoot"		<< (*it)->imgRoot;
			fs << "imgName"		<< (*it)->imgName;
			fs << "fixed" 		<< (*it)->fixed;
			fs << "cvtransform"	<< Mat((*it)->getCVTransform());
			fs << "features"	<< "[";
			for (unsigned int i=0; i<(*it)->kpts.size(); i++){
				fs<<"{";
				fs<<"pt"		<<kpts[i].pt;
				fs<<"octave"	<<kpts[i].octave;
				fs<<"angle"		<<kpts[i].angle;
				fs<<"size"		<<kpts[i].size;
				fs<<"class_id"	<<kpts[i].class_id;
				fs<<"response"	<<kpts[i].response;
				fs<<"}";
			}
			fs << "]";
			fs << "decs" << decs;
			fs << "}";

		}
		fs<<"]";
		cout<<frames.size()<<" frames written"<<endl;

		//landmarks
		map<LandMark::Ptr, int, Data::LandMarkPtrCompare> lm2idx;
		unsigned int landmarksCnt = 0;
		fs<<"landmarks"<<"[";
		for(vector<LandMark::Ptr>::const_iterator it =lms.begin(); it!=lms.end(); ++it, ++landmarksCnt){
			Point3f pt3D((*it)->pt[0],(*it)->pt[1],(*it)->pt[2]);
			fs<<pt3D;
			lm2idx[*it] = landmarksCnt;
		}
		fs<<"]";
		cout<<lms.size()<<" landmarks written"<<endl;

		//measurements
		fs<<"measurements"<<"[";
		for(vector<Measurement::Ptr>::const_iterator it = ms.begin(); it!=ms.end(); ++it){
			//cout<<(*it)->toString()<<endl;
			fs << "{";
			fs << "imgIdx" 		<< (*it)->frame->imgIdx;
			fs << "featureIdx"	<< (*it)->featureIdx;
			fs << "landmarkIdx"	<< lm2idx[(*it)->landmark];
			fs << "}";
		}
		fs<<"]";
		cout<<ms.size()<<" measures written";

		fs.release();
	}
	//if .ply
	else if(Utils::endsWith(fname,".ply")){

		vector<Point3f> norms;
		bool saveNormal = false;

		ofstream myfile;
		myfile.open (filename.c_str());

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

		myfile <<"element vertex "<<lms.size()+frames.size()*4<<endl;
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
		myfile <<"element edge "<<frames.size()*3<<endl;            // 3 axis
		myfile <<"property int vertex1"<<endl;
		myfile <<"property int vertex2"<<endl;
		myfile <<"property uchar red"<<endl;
		myfile <<"property uchar green"<<endl;
		myfile <<"property uchar blue"<<endl;
		myfile <<"end_header"<<endl;

		for (vector<LandMark::Ptr>::const_iterator it = lms.begin(); it!=lms.end(); ++it)
		{
				myfile <<(*it)->pt.transpose();
				if(saveNormal){
					float nx = 0;//norms[n].x;
					float ny = 0;//norms[n].y;
					float nz = 0;//norms[n].z;
					myfile <<nx<<" "<<ny<<" "<<nz<<" ";
				}
				myfile <<" "<<255<<" "<<255<<" "<<255<<endl;
		}
		for (vector<Frame::Ptr>::const_iterator it = frames.begin(); it!=frames.end(); ++it)
		{

			Vector3d origin = (*it)->position;
			Quaterniond qc	= (*it)->rotation.conjugate();
			Vector3d axisX	= qc*Vector3d(1,0,0)+origin;
			Vector3d axisY	= qc*Vector3d(0,1,0)+origin;
			Vector3d axisZ	= qc*Vector3d(0,0,1)+origin;

			if(saveNormal){
				myfile <<origin.transpose()<<" "<<0<<" "<<0<<" "<<0<<" "<<255<<" "<<255<<" "<<255<<endl;
				myfile <<axisX.transpose()<<" "<<0<<" "<<0<<" "<<0<<" "<<255<<" "<<0<<" "<<0<<endl;
				myfile <<axisY.transpose()<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<255<<" "<<0<<endl;
				myfile <<axisZ.transpose()<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<0<<" "<<255<<endl;
			}else{
				myfile <<origin.transpose()<<" "<<255<<" "<<255<<" "<<255<<endl;
				myfile <<axisX.transpose()<<" "<<255<<" "<<0<<" "<<0<<endl;
				myfile <<axisY.transpose()<<" "<<0<<" "<<255<<" "<<0<<endl;
				myfile <<axisZ.transpose()<<" "<<0<<" "<<0<<" "<<255<<endl;
			}

		}

		// draw the axis
		unsigned int  offset = lms.size();
		for (unsigned int n=0 ; n<frames.size() ; n++)
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
		double f = camera.getCamFocal();
		double fx= f/camera.w;
		double fy= f/camera.h;
		bool hasNormal 	= false;
		bool hasGPS	  	= false;
		//TODO: set last value 0 instead of 1e-6
		myfile<<"CamParam: "<<camera.w<<" "<<camera.h<<" "<<setprecision(17)<<fx<<" "<<fy<<" "<<0.5<<" "<<0.5<<" "<<1e-6<<endl;
		myfile<<"HasNormal: "<<hasNormal<<endl;
		myfile<<"HasGPS: "<<hasGPS<<endl;
		myfile<<"ImgRoot: "<<imgRoot<<endl;
		//save points
		map<LandMark::Ptr, int, Data::LandMarkPtrCompare> lm2idx;
		myfile<<"Points_start"<<endl;
		for(int i=0; i<lms.size(); i++){
			//point id
			myfile<<i<<" ";
			//point coords
			myfile	<<setprecision(17)<<lms[i]->pt.transpose()<<" ";
			if(hasNormal){
				//myfile <<ptCloud.pt3Ds[i].norm.x<<" "<<ptCloud.pt3Ds[i].norm.y<<" "<<ptCloud.pt3Ds[i].norm.z;
			}
			myfile 	<<endl;
			lm2idx[lms[i]] = i;
		}
		myfile<<"Points_end"<<endl;

		//save camera
		myfile<<"Cams_start"<<endl;
		cout<<" projectIO, write tiny, camera cnt = "<<frames.size()<<endl;
		for(unsigned int i=0; i<frames.size(); i++){
			const Frame::Ptr &frame = frames[i];
			cout<<"cam "<<i<<" img "<<frame->imgIdx<<endl;
			//cam id
			myfile<<i<<" ";

			//lat lon
			//double lat, lon;
			//assert(ptCloud.getImageGPS(imgIdx, lat, lon));
			//myfile<<setprecision(17)<<lat<<" "<<lon<<" ";

			//image name
			myfile<<imgNames[frame->imgIdx]<<" ";

			//pose
			Vector3d rvec 	= AngleAxisd(frame->rotation).axis();
			Vector3d t		= -(frame->rotation*frame->position);
			myfile<<setprecision(17)<<rvec.transpose()<<" "<<t.transpose()<<" ";

			//measurements
			const vector<Measurement::Ptr> fms = data.getMeasurements(frame);
			for(vector<Measurement::Ptr>::const_iterator jt=fms.begin(); jt!=fms.end(); ++jt){
				Point2f &xy = frame->kpts[(*jt)->featureIdx].pt;
				myfile<<setprecision(17)<<lm2idx[(*jt)->landmark]<<" "<<xy.x<<" "<<xy.y<<" ";
			}
			myfile<<endl;
		}
		myfile<<"Cams_end"<<endl;

		myfile.close();
	}
	//if .tiny2
	else if(Utils::endsWith(fname,".tiny2")){
		ofstream myfile;
		myfile.open (filename.c_str());

		//save calibration
		//XXX: assumed principle is the image center, assumed same x,y focal
		double f = camera.getCamFocal();
		double fx= f/camera.w;
		double fy= f/camera.h;
		bool hasNormal 	= false;
		bool hasGPS	  	= false;

		myfile<<"CamParam: "<<camera.w<<" "<<camera.h<<" "<<setprecision(17)<<fx<<" "<<fy<<" "<<0.5<<" "<<0.5<<" "<<1e-6<<endl;
		myfile<<"HasNormal: "<<hasNormal<<endl;
		myfile<<"HasGPS: "<<hasGPS<<endl;
		myfile<<"ImgRoot: "<<imgRoot<<endl;

		//save camera
		myfile<<"Cams_start"<<endl;
		cout<<" projectIO, write tiny2, camera cnt = "<<frames.size()<<endl;
		for(unsigned int i=0; i<frames.size(); i++){
			const Frame::Ptr &frame = frames[i];
			cout<<"cam "<<i<<" img "<<frame->imgIdx<<endl;
			//cam id
			myfile<<i<<" ";

			//lat lon
			//double lat, lon;
			//assert(ptCloud.getImageGPS(imgIdx, lat, lon));
			//myfile<<setprecision(17)<<lat<<" "<<lon<<" ";

			//image name
			myfile<<imgNames[frame->imgIdx]<<" ";

			//pose
			Vector3d rvec 	= AngleAxisd(frame->rotation).axis();
			Vector3d t		= -(frame->rotation*frame->position);
			myfile<<setprecision(17)<<rvec.transpose()<<" "<<t.transpose()<<" ";

			//measurements 2d and 3d
			vector<Measurement::Ptr> fms = data.getMeasurements(frame);
			for(vector<Measurement::Ptr>::const_iterator jt=fms.begin(); jt!=fms.end(); ++jt){
				Point2f &xy = frame->kpts[(*jt)->featureIdx].pt;
				myfile<<setprecision(17)<<xy.x<<" "<<xy.y<<" "<<(*jt)->landmark->pt.transpose()<<" ";
				if(hasNormal){
					//const Point3f &norm = ptCloud.pt3Ds[pt3DIdxs[j]].norm;
					//myfile<<norm.x<<" "<<norm.y<<" "<<norm.z<<" ";
				}
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
		myfile<<"cameras: "<<frames.size()<<endl;

		double f 	= camera.getCamFocal();
		double fov 	= camera.getFOVH();
		for(unsigned int i=0; i<frames.size(); i++){
			const Frame::Ptr &frame = frames[i];

			Vector3d origin = frame->position;
			Quaterniond qc	= frame->rotation.conjugate();
			Vector3d axisX	= qc*Vector3d(1,0,0)+origin;
			Vector3d axisY	= qc*Vector3d(0,1,0)+origin;
			Vector3d axisZ	= qc*Vector3d(0,0,1)+origin;

			myfile<<imgNames[i]<<" "<<origin.transpose()<<" "<<axisZ.transpose()<<" "<<-axisY.transpose()<<" "<<fov<<endl;

		}

		myfile<<"points: "<<lms.size()<<endl;
		for(unsigned int i=0; i<lms.size(); i++){
			myfile<<lms[i]->pt.transpose()<<endl;
		}

		myfile.close();
	}
	//if .map an image-less format
	else if(Utils::endsWith(fname,".map")){
		ofstream myfile;
		myfile.open (filename.c_str());

		//vocab db
		string vocabDBfname = fname.substr(0,fname.length()-4)+"_vocabDB.yaml.gz";
		myfile<<1<<endl;
		myfile<<vocabDBfname<<endl;

		//camera
		double f = camera.getCamFocal();
		Point2d pp = camera.getCamPrinciple();
		double r = camera.distortionMat.at<double>(0);
		myfile<<camera.w<<" "<<camera.h<<" "<<f<<" "<<pp.x<<" "<<pp.y<<" "<<r<<endl;

		//frames
		map<Frame::Ptr, int, Data::FramePtrCompare> frame2idx;
		const int frameCnt	= frames.size();
		myfile<<frameCnt<<endl;
		for(unsigned int i=0; i<frameCnt; i++){
			frame2idx[frames[i]] 			= i;
			const Vector3d 	&position 		= frames[i]->position;
			const Quaterniond	&rotation 	= frames[i]->rotation;
			const vector<KeyPoint>& kpts 	= frames[i]->kpts;
			const Mat& decs 				= frames[i]->decs;
			const int featureCnt		  	= kpts.size();
			const int decLen				= decs.cols;
			myfile	<<position[0]<<" "<<position[1]<<" "<<position[2]<<" "
					<<rotation.w()<<" "<<rotation.x()<<" "<<rotation.y()<<" "<<rotation.z()<<" "
					<<featureCnt<<" "<<decLen<<endl;

			for(vector<KeyPoint>::const_iterator jt = kpts.begin(); jt!= kpts.end(); ++jt){
				myfile	<<jt->pt.x<<" "<<jt->pt.y<<" "<<jt->size<<" "<<jt->angle<<" "<<jt->response<<" "
						<<jt->octave<<" "<<jt->class_id<<endl;
			}
			for(unsigned int j =0; j<featureCnt; j++){
				for(unsigned int k =0; k<decLen; k++){
					myfile	<<(int)decs.at<unsigned char>(j,k)<<" ";
				}
				myfile<<endl;
			}

		}

		//landmarks
		map<LandMark::Ptr, int, Data::LandMarkPtrCompare> landmark2idx;
		const int landmarkCnt	= lms.size();
		myfile<<landmarkCnt<<endl;
		for(unsigned int i=0; i<landmarkCnt; i++){
			landmark2idx[lms[i]] 			= i;
			const Vector3d	&point			= lms[i]->pt;
			myfile	<<point[0]<<" "<<point[1]<<" "<<point[2]<<endl;
		}

		//measures
		const int measureCnt	= ms.size();
		myfile<<measureCnt<<endl;
		for(vector<Measurement::Ptr>::const_iterator it = ms.begin(); it!=ms.end(); ++it){
			myfile<<frame2idx[(*it)->frame]<<" "<<(*it)->featureIdx<<" "<<landmark2idx[(*it)->landmark]<<endl;
		}

		myfile.close();
		// data.makeVocabDB();
		// data.saveVocabDB(vocabDBfname);
	}
}

void ProjectIO::readProject(	const string			&fname,
								string 					&imgRoot,
								vector<string>			&imgNames)
{
	cout<<fname<<endl;
	Camera &camera	= Camera::GetInstance();
	Data &data 		= Data::GetInstance();
	data.reset();
	imgNames.clear();

	map<int, Frame::Ptr> 		idx2frame;
	map<int, LandMark::Ptr> 	idx2landmark;

	//if yaml
	if(Utils::endsWith(fname,".yaml")){
		bool hasNormal = false;
		FileStorage fs;
		fs.open(fname, FileStorage::READ);
		fs["imgW"]>>camera.w;
		fs["imgH"]>>camera.h;
		fs["camIntrinsicMat"]>>camera.camMat;
		fs["camDistortionMat"]>>camera.distortionMat;
		fs["imgRoot"]>>imgRoot;

		//images
		FileNode nodeImgs = fs["imgs"];
		if (nodeImgs.type() == FileNode::SEQ){
			imgNames.reserve(nodeImgs.size());
			for(FileNodeIterator it = nodeImgs.begin(); it!= nodeImgs.end(); ++it){
				imgNames.push_back((string)*it);
			}
		}
		cout<<imgNames.size()<<" images read"<<endl;

		//frames
		FileNode nodeFrames = fs["frames"];
		if (nodeFrames.type() == FileNode::SEQ){
			for(FileNodeIterator it = nodeFrames.begin(); it!= nodeFrames.end(); ++it){
				FileNode n 		= (FileNode) (*it);
				int imgIdx;
				string imgRoot, imgName;
				n["imgIdx"]		>> imgIdx;
				n["imgRoot"]	>> imgRoot;
				n["imgName"]	>> imgName;
				Frame::Ptr frame(new Frame(imgIdx, imgRoot, imgName));
				n["fixed"]		>> frame->fixed;
				Mat m;
				n["cvtransform"]>> m;
				Matrix3d R;
				Vector3d t;
				for(int r=0; r<3; r++){
					for(int c=0; c<3; c++){
						R(r,c) = m.at<double>(r,c);
					}
					t(r) = m.at<double>(r,3);
				}

				frame->rotation = Quaterniond(R);
				frame->position = R.transpose()*(-t);
				idx2frame[imgIdx] = frame;
				FileNode features = (FileNode) n["features"];
				if (features.type() == FileNode::SEQ){
					frame->kpts.reserve(features.size());
					for(FileNodeIterator jt = features.begin(); jt!= features.end(); jt++){
						FileNode nn = (FileNode) (*jt);
						KeyPoint kpt;
						nn["pt"]		>>kpt.pt;
						nn["octave"]	>>kpt.octave;
						nn["angle"]		>>kpt.angle;
						nn["size"]		>>kpt.size;
						nn["class_id"]	>>kpt.class_id;
						nn["response"]	>>kpt.response;
						frame->kpts.push_back(kpt);
					}
				}
				n["decs"]>>frame->decs;
				frame->makeLUT(camera.h);
				frame->resetMeasures();
				data.addFrame(frame);
				idx2frame[frame->imgIdx] = frame;
			}
		}
		cout<<data.countFrames()<<" frames read"<<endl;

		//landmarks
		FileNode landmarks = fs["landmarks"];
		if (landmarks.type() == FileNode::SEQ){
			int lmkIdx = 0;
			for(FileNodeIterator it = landmarks.begin(); it!= landmarks.end(); ++it, ++lmkIdx){
				FileNode n = (FileNode) (*it);
				Point3f pt;
				n>>pt;
				Vector3d pt_eigen(pt.x, pt.y, pt.z);
				LandMark::Ptr lm(new LandMark(pt_eigen));
				data.addLandMark(lm);
				idx2landmark[lmkIdx] = lm;
			}
		}
		cout<<data.countLandMarks()<<" landmarks read"<<endl;

		//measurements
		FileNode measurements = fs["measurements"];
		if(measurements.type() == FileNode::SEQ){
			cout<<measurements.size()<<endl;
			for(FileNodeIterator it = measurements.begin(); it != measurements.end(); ++it){
				FileNode n = (FileNode) (*it);
				int imgIdx, lmkIdx, featureIdx;
				n["imgIdx"]		>> imgIdx;
				n["featureIdx"]	>> featureIdx;
				n["landmarkIdx"]>> lmkIdx;
				Measurement::Ptr m(new Measurement(idx2frame[imgIdx], featureIdx, idx2landmark[lmkIdx]));
				//cout<<m->toString()<<endl;
				data.addMeasurement(m);
			}
		}
		cout<<data.countMeasurements()<<" Measurements read"<<endl;
		fs.release();
	}
	else if(Utils::endsWith(fname,".nvm")){
		cout<<"file format: nvm"<<endl;
		bool hasNormal = false;
		const int LINE_INTRINSICS	= 1;
		const int LINE_IMGROOT		= 2;
		const int LINE_IMGCNT		= 3;
		int LINE_PTCNT				= 10000000;
		int currLine				= 0;
		int imgW_Half				= -1;
		int imgH_Half				= -1;

		//imgIdx, vector<keypoint, landmark >
		map<int, vector<pair<KeyPoint, LandMark::Ptr > > > measuresMap;

		string featureRoot = "";
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
					CM.copyTo(camera.camMat);		//must copy to retain data
					DM.copyTo(camera.distortionMat);//must copy to retain data
					cout<<"camera intrinsics read"<<endl;
					cout<<camera.camMat<<endl;
					cout<<camera.distortionMat<<endl;
				}
					break;
				case LINE_IMGROOT:
				{
					istringstream iss(line);
					iss>>imgRoot>>featureRoot;
					cout<<"imgRoot    : "<<imgRoot<<endl;
					cout<<"featureRoot: "<<featureRoot<<endl;
				}
					break;
				case LINE_IMGCNT:
				{
					istringstream iss(line);
					int imgcnt;
					iss>>imgcnt;
					LINE_PTCNT = LINE_IMGCNT+imgcnt+2;
					for(int i = 0; i<imgcnt; i++){
						getline(infile, line);
						currLine++;

						istringstream iss2(line);
						string imgName;
						double f,qw,qx,qy,qz,t0,t1,t2,r;
						iss2>>imgName>>f>>qw>>qx>>qy>>qz>>t0>>t1>>t2>>r;
						//cout<<imgName<<" "<<f<<" "<<qw<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<t0<<" "<<t1<<" "<<t2<<" "<<r<<endl;

						if(imgW_Half <0){
							Mat tmp = imread(imgRoot+"/"+imgName,IMREAD_COLOR);
							camera.w	= tmp.cols;
							camera.h 	= tmp.rows;
							imgW_Half 	= tmp.cols/2;
							imgH_Half	= tmp.rows/2;
							cout<<"img size = "<<camera.w<<"x"<<camera.h<<endl;
						}
						imgNames.push_back(imgName);
						//assume sequential image idx
						Frame::Ptr frame(new Frame(i, imgRoot, imgName));
						frame->rotation = Quaterniond(qw,qx,qy,qz);
						frame->rotation.normalize();
						frame->position = Vector3d(t0, t1, t2);
						/*double sqw = qw*qw;
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

						*/

						if(featureRoot.compare("") != 0){
							readFeatures(featureRoot, frame->imgName, frame->kpts, frame->decs);
						}

						data.addFrame(frame);
						idx2frame[frame->imgIdx] = frame;
					}
					cout<<data.countFrames()<<" frames read"<<endl;
				}
					break;
				default:
				{
					if(currLine == LINE_PTCNT){
						istringstream iss(line);
						int ptcnt;
						iss>>ptcnt;
						//landmarks
						for(int i=0; i<ptcnt; i++){
							getline(infile, line);
							currLine++;
							istringstream iss2(line);
							double x,y,z;
							int r,g,b;
							int measurementCnt;
							iss2>>x>>y>>z>>r>>g>>b>>measurementCnt;
							Vector3d 		pt(x,y,z);
							LandMark::Ptr 	lm(new LandMark(pt));
							data.addLandMark(lm);
							//measurements
							while(measurementCnt-->0){
								int imgIdx, featureIdx;	//pt2DIdx is useless, we will create our own idx
								double x_2d, y_2d;
								iss2>>imgIdx>>featureIdx>>x_2d>>y_2d;
								//XXX: NVM file 2d coordinates are relative to image center (not principle point) rather than top-left!
								//convert coordinates to top-left
								//XXX: nvm may have a feature measured to two landmarks which is prohibited by MSFM
								//work around is to save duplicated features
								x_2d+=imgW_Half;
								y_2d+=imgH_Half;

								KeyPoint kpt;
								if(featureRoot.compare("")==0){
									//no feature file
									kpt = KeyPoint(x_2d, y_2d, 5);
								}else{
									//has feature file
									kpt = data.getFrame(imgIdx)->kpts[featureIdx];
									Point2f pt2D(x_2d, y_2d);
									assert(cv::norm(pt2D-kpt.pt)<3);	//ensure the indexing was correct
								}

								map<int, vector<pair<KeyPoint, LandMark::Ptr > > >::iterator it = measuresMap.find(imgIdx);
								if(it == measuresMap.end()){
									vector<pair<KeyPoint, LandMark::Ptr > > kpts;
									kpts.push_back(make_pair(kpt, lm));
									measuresMap[imgIdx] = kpts;
								}else{
									it->second.push_back(make_pair(kpt, lm));
								}
							}
						}
						cout<<data.countLandMarks()<<" landmarks read"<<endl;

						for(map<int, vector<pair<KeyPoint, LandMark::Ptr > > >::iterator it = measuresMap.begin(); it!=measuresMap.end(); ++it){
							Frame::Ptr &frame = idx2frame[it->first];
							frame->clearFeatures();
							vector<pair<KeyPoint, LandMark::Ptr > > &kpts = it->second;
							//cout<<"frame:"<<it->first<<endl;
							sort(kpts.begin(), kpts.end(), KptLmkPairComparator);
							unsigned int kptsCnt = kpts.size();
							frame->kpts.reserve(kptsCnt);
							for(unsigned int j = 0; j<kptsCnt; ++j){
								frame->kpts.push_back(kpts[j].first);
							}

							frame->resetMeasures();
							frame->makeLUT(camera.h);
							frame->describeFeatures();
							//brisk describe will discard undiscribable kpt, make sure this has not happened
							//TODO: handle when some kpts are discarded in the description process
							assert(frame->kpts.size() == kptsCnt);

							for(unsigned int j = 0; j<kptsCnt; ++j){
								Measurement::Ptr m(new Measurement(frame,j,kpts[j].second));
								data.addMeasurement(m);
							}
						}

						cout<<data.countMeasurements()<<" measurements read"<<endl;

					}
				}
					break;
			}

		}
		infile.close();
	}
	//if .map an image-less format
	else if(Utils::endsWith(fname,".map")){
		ifstream infile(fname.c_str());

		//db
		int hasDB;
		string dbfile;
		infile>>hasDB;
		if(hasDB){
			infile>>dbfile;
			string folder 	= fname.substr(0,fname.find_last_of("/"));
			dbfile 			= folder+"/"+dbfile;
		}

		//camera
		double w, h, f, ppx, ppy, r;
		infile>>w>>h>>f>>ppx>>ppy>>r;
		double camMatArr[9] 		= { f, 			0.0, 		ppx,
										0.0, 		f, 			ppy,
										0.0,		0.0,		1.0 };
		double distortCoeffArr[5] 	= { r, 0, 0, 0, 0 };
		Mat CM(3, 3, CV_64F, camMatArr);
		Mat DM(1, 5, CV_64F, distortCoeffArr);
		CM.copyTo(camera.camMat);		//must copy to retain data
		DM.copyTo(camera.distortionMat);//must copy to retain data
		camera.w = w;
		camera.h = h;
		cout<<"camera intrinsics read"<<endl;
		cout<<camera.camMat<<endl;
		cout<<camera.distortionMat<<endl;

		//frames
		int frameCnt;
		infile>>frameCnt;
		imgRoot							= "";
		for(unsigned int i=0; i<frameCnt; i++){
			stringstream	ss;
			ss<<"img_"<<i;
			string imgName				= ss.str();
			imgNames.push_back(imgName);
			Frame::Ptr frame(new Frame(i,imgRoot,imgName));
			idx2frame[i] = frame;
			Vector3d 	&position 		= frame->position;
			Quaterniond	&rotation		= frame->rotation;
			int featureCnt;
			int decLen;
			infile	>>position[0]>>position[1]>>position[2]
					>>rotation.coeffs()[3]>>rotation.coeffs()[0]>>rotation.coeffs()[1]>>rotation.coeffs()[2]
					>>featureCnt>>decLen;
			for(unsigned int j=0; j<featureCnt; j++){
				KeyPoint kpt;
				infile>>kpt.pt.x>>kpt.pt.y>>kpt.size>>kpt.angle>>kpt.response>>kpt.octave>>kpt.class_id;
				frame->kpts.push_back(kpt);
			}
			frame->decs = Mat(featureCnt, decLen, CV_8UC1);
			for(unsigned int j =0; j<featureCnt; j++){
				int val;
				for(unsigned int k =0; k<decLen; k++){
					infile>>val;
					frame->decs.at<unsigned char>(j,k) = (unsigned char)val;
				}
			}
			frame->makeLUT(camera.h);
			frame->resetMeasures();
			data.addFrame(frame);
		}
		cout<<frameCnt<<" frames read"<<endl;

		//landmarks
		int landmarkCnt;
		infile>>landmarkCnt;
		for(unsigned int i=0; i<landmarkCnt; i++){
			LandMark::Ptr lm(new LandMark());
			idx2landmark[i] 			= lm;
			Vector3d	&point			= lm->pt;
			infile	>>point[0]>>point[1]>>point[2];
			data.addLandMark(lm);
		}
		cout<<landmarkCnt<<" landmarks read"<<endl;

		//measures
		int measureCnt;
		infile>>measureCnt;
		for(unsigned int i=0; i<measureCnt; i++){
			int frameIdx, featureIdx, landmarkIdx;
			infile>>frameIdx>>featureIdx>>landmarkIdx;
			Measurement::Ptr m(new Measurement(idx2frame[frameIdx],featureIdx, idx2landmark[landmarkIdx]));
			data.addMeasurement(m);
		}
		cout<<measureCnt<<" measurements read"<<endl;

		idx2frame.clear();
		idx2landmark.clear();
		infile.close();

		// if(hasDB){
		// 	data.loadVocabDB(dbfile);
		// }else{
		// 	data.makeVocabDB();
		// }
	}
	/*else if(Utils::endsWith(fname,".patch")){
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
				//Utils::readPaths(ptCloud.imgRoot,ptCloud.imgs);
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
		cout<<"file format: tiny"<<endl;
		ifstream infile(fname.c_str());
		string line;
		bool readCam = false;
		bool readPts = false;
		int camCount = 0;
		int ptsCount = 0;
		while (getline(infile, line)){
			if(readPts){
				if(line=="Points_end"){	//simple equal comparison to minize computation
					readPts = false;
					assert(ptsCount == ptCloud.pt3Ds.size());
					cout<<"total "<<ptsCount<<" points read"<<endl;
				}else{
					istringstream iss(line);
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
					ptsCount++;
				}
			}else if(readCam){
				if(line=="Cams_end"){
					readCam = false;
					assert(camCount == ptCloud.imgs.size());
					cout<<"total "<<camCount<<" images & cameras read"<<endl;
				}else{
					istringstream iss(line);
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
					camCount++;
				}
			}else{
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
				}else if(line.find("HasGPS:")!=line.npos){
					istringstream iss(line);
					string nouse;
					iss>>nouse>>ptCloud.hasGPS;
				}else if(line.find("Points_start")!=line.npos){
					readPts = true;
				}else if(line.find("Cams_start")!=line.npos){
					readCam = true;
				}
			}
		}
	}else if(Utils::endsWith(fname,".tiny2")){
		cout<<"file format: tiny2"<<endl;
		ifstream infile(fname.c_str());
		string line;
		bool readCam = false;
		int camCount = 0;
		while (getline(infile, line)){
			if(readCam){
				if(line=="Cams_end"){	//simple equal comparison to minimize computation
					readCam = false;
					assert(camCount == ptCloud.imgs.size());
					cout<<"total "<<camCount<<" images & cameras read"<<endl;
				}else{
					istringstream iss(line);
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

					//add 2d & 3d
					ptCloud.img2pt2Ds[imgIdx] = vector<Pt2D>();
					while(measureCnt-->0){
						float x,y,x3,y3,z3;
						iss>>x>>y>>x3>>y3>>z3;

						//add 2d
						Pt2D pt2D;
						pt2D.pt = Point2f(x, y);
						pt2D.img_idx = imgIdx;
						pt2D.pt3D_idx = ptCloud.pt3Ds.size();
						pt2D.dec = Mat(1,32, CV_8UC1, Scalar(0)); // dummy orb descriptor
						ptCloud.img2pt2Ds[imgIdx].push_back(pt2D);

						//add 3d
						Pt3D pt3D;
						pt3D.pt = Point3f(x3,y3,z3);
						if(ptCloud.hasPointNormal){
							double nx, ny, nz;
							iss>>nx>>ny>>nz;
							pt3D.norm = Point3f(nx,ny,nz);	//by right this should already been normalized
						}
						pt3D.img2ptIdx[imgIdx] = ptCloud.img2pt2Ds[imgIdx].size()-1;
						ptCloud.pt3Ds.push_back(pt3D);
					}
					camCount++;
				}
			}else{
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
				}else if(line.find("HasGPS:")!=line.npos){
					istringstream iss(line);
					string nouse;
					iss>>nouse>>ptCloud.hasGPS;
				}else if(line.find("Cams_start")!=line.npos){
					readCam = true;
				}
			}
		}
	}

	if(!Utils::endsWith(fname,".tiny") && !Utils::endsWith(fname,".tiny2")){
		for(int i=0; i<ptCloud.camMats.size(); i++){
			int imgIdx = ptCloud.camMat2img[i];
			ptCloud.img2GPS[imgIdx] = make_pair(0,0);	//dummy
		}
	}
	*/


	cout<<"stats:"<<endl;
	cout<<"images       = "<<imgNames.size()<<endl;
	cout<<"frames       = "<<data.countFrames()<<endl;
	cout<<"landmarks    = "<<data.countLandMarks()<<endl;
	cout<<"measurements = "<<data.countMeasurements()<<endl;


}

/*
//for future use
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
		ptCloud.hasGPS = true;
	}
}
*/

void ProjectIO::writeFeatures(	const string 				&featureRoot,
								const string				&imgName,
								const vector<KeyPoint>		&kpts,
								const Mat					&decs)
{
	ofstream siftFile,cvFile;


	//write .sift file for visualSFM
	siftFile.open (featureRoot+"/"+imgName.substr(0,imgName.length()-4)+".sift");
	siftFile <<kpts.size()<<" "<<128<<endl;
	for(unsigned int i=0; i<kpts.size(); i++){
		siftFile <<kpts[i].pt.y<<" "<<kpts[i].pt.x<<" "<<kpts[i].size/8.0<<" "<<(360-kpts[i].angle)/180.0*M_PI<<endl; //sift uses different angle direction and uses radian instead of degree
		const Mat &dec = decs.row(i);
		for(unsigned int j=0; j<128; j++){
			int newVal = 0;
			if(j<dec.cols){
				newVal = dec.at<unsigned char>(j) * 2;
			}
			siftFile <<newVal<<" ";
			if(j!=0 && j%20 == 0){
				siftFile <<endl;
			}
		}
		siftFile<<endl;
	}
	siftFile.close();


	//write .cvfe file for MSFM
	cvFile.open (featureRoot+"/"+imgName.substr(0,imgName.length()-4)+".cvfe");
	cvFile 	 <<kpts.size()<<" "<<decs.cols<<endl;
	for(unsigned int i=0; i<kpts.size(); i++){
		cvFile	<<kpts[i].pt.x<<" "<<kpts[i].pt.y<<" "<<kpts[i].size<<" "<<kpts[i].angle<<" "<<kpts[i].response<<" "<<kpts[i].octave<<" "<<kpts[i].class_id<<endl;
		const Mat &dec = decs.row(i);
		for(unsigned int j=0; j<dec.cols; j++){
			int val = dec.at<unsigned char>(j);	//XXX: you should cast unsigned char to int otherwise, you will be writing weired char to file
			cvFile<<val<<" ";
		}
		cvFile<<endl;
	}
	cvFile.close();
}


void ProjectIO::readFeatures(	const std::string 				&featureRoot,
								const std::string				&imgName,
								std::vector<cv::KeyPoint>		&kpts,
								cv::Mat							&decs)
{
	string featureFileName = featureRoot+"/"+imgName.substr(0,imgName.length()-4)+".cvfe";
	ifstream infile(featureFileName.c_str());
	assert(infile.good());

	const int TASK_READ_HEADER 	= 0;
	const int TASK_READ_KPT		= 1;
	const int TASK_READ_DEC		= 2;
	const int TASK_END			= 3;
	int task = TASK_READ_HEADER;

	int featureCnt, decLen;

	while(task != TASK_END){
		if(task == TASK_READ_HEADER){
			if(infile>>featureCnt>>decLen){
				kpts.clear();
				kpts.reserve(featureCnt);
				decs = Mat(featureCnt,decLen, CV_8UC1);
				task = TASK_READ_KPT;
			}else{
				task = TASK_END;
			}
		}else if(task == TASK_READ_KPT){
			KeyPoint kpt;
			if(infile>>kpt.pt.x>>kpt.pt.y>>kpt.size>>kpt.angle>>kpt.response>>kpt.octave>>kpt.class_id){
				//cout<<"kpt:"<<kpt.pt.x<<" "<<kpt.pt.y<<" "<<kpt.size<<" "<<kpt.angle<<" "<<kpt.response<<" "<<kpt.octave<<" "<<kpt.class_id<<endl;
				kpts.push_back(kpt);
				task = TASK_READ_DEC;
			}else{
				task = TASK_END;
			}
		}else if(task == TASK_READ_DEC){
			int decRow = kpts.size()-1;
			bool isEOF = false;
			for(unsigned int i=0; i<decLen; i++){
				//xxx: do not directly read to chars, because it will treat the input as character rather than integer
				int val;
				if(infile>>val){
					decs.at<unsigned char>(decRow, i) = (unsigned char) val;
				}else{
					task = TASK_END;
					break;
				}
			}
			if(task != TASK_END){
				//cout<<"dec:"<<decs.row(decRow)<<endl;
				task = TASK_READ_KPT;
			}
		}
	}
}

void ProjectIO::writeMatches(	const std::string				&fname,
								const std::string 				&imgName1,
								const std::string				&imgName2,
								const std::vector<cv::DMatch> 	&matches)
{
	ofstream myfile;
	myfile.open (fname,std::ios_base::app);	//append

	myfile<<imgName1<<" "<<imgName2<<" "<<matches.size()<<endl;
	vector<int> kptIdxs1, kptIdxs2;
	for(vector<DMatch>::const_iterator it = matches.begin(); it!=matches.end(); ++it){
		kptIdxs1.push_back(it->queryIdx);
		kptIdxs2.push_back(it->trainIdx);
	}
	for(vector<int>::iterator it = kptIdxs1.begin(); it != kptIdxs1.end(); ++it){
		myfile<<*it<<" ";
	}
	myfile<<endl;
	for(vector<int>::iterator it = kptIdxs2.begin(); it != kptIdxs2.end(); ++it){
		myfile<<*it<<" ";
	}
	myfile<<endl;
	myfile.close();
}

