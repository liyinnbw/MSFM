/*
* Bundle adjustment using ceres
*/

//#define V3DLIB_ENABLE_SUITESPARSE
#include "BAHandler.h"

// #include "ba/v3d_linear.h"
// #include "ba/v3d_vrmlio.h"
// #include "ba/v3d_metricbundle.h"
// #include "ba/v3d_stereobundle.h"

#include "Camera.h"
// #include "ceres/EigenQuaternionParameterization.h"
#include "datastructs/Data.h"

#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <ceres/rotation.h>

#include <Eigen/Eigen>

// using namespace V3D;
using namespace std;
using namespace cv; 

// namespace
// {
// 	inline double
// 	showErrorStatistics(double const f0,
// 						StdDistortionFunction const& distortion,
// 						vector<CameraMatrix> const& cams,
// 						vector<Vector3d> const& Xs,
// 						vector<Vector2d> const& measurements,
// 						vector<int> const& correspondingView,
// 						vector<int> const& correspondingPoint)
// 	{
// 		int const K = measurements.size();
		
// 		double meanReprojectionError = 0.0;
// 		for (int k = 0; k < K; ++k)
// 		{
// 			int const i = correspondingView[k];
// 			int const j = correspondingPoint[k];
// 			Vector2d p = cams[i].projectPoint(distortion, Xs[j]);
			
// 			double reprojectionError = norm_L2(f0 * (p - measurements[k]));
// 			meanReprojectionError += reprojectionError;
// 		}
// 		//cout << "mean reprojection error (in pixels): " << meanReprojectionError/K << endl;

// 		return meanReprojectionError/K;
// 	}
// }

// void BAHandler::adjustBundle(){

// 	cout<<"Bundle Adjust Method: no copy"<<endl;
// 	Data 		&data 	= Data::GetInstance();
// 	Camera 		&camera	= Camera::GetInstance();
// 	//important
// 	data.deleteTrashes();

// 	Mat &cam_matrix 				= camera.camMat;
// 	Mat &distortion_coefficients	= camera.distortionMat;


// 	vector<int>			imgIdxs;
// 	vector<int>			pt3DIdxs;
// 	double ppx,ppy;
// 	double 	meanErrorBefore, meanErrorAfter;

// 	int N = data.countFrames(), M = data.countLandMarks(), K = data.countMeasurements();
// 	cout << "N (cams) = " << N << " M (points) = " << M << " K (measurements) = " << K << endl;

// 	StdDistortionFunction distortion;
// 	distortion.k1 = distortion_coefficients.at<double>(0,0);
// 	distortion.k2 = distortion_coefficients.at<double>(0,1);
// 	distortion.p1 = distortion_coefficients.at<double>(0,2);
// 	distortion.p2 = distortion_coefficients.at<double>(0,3);


// 	//convert camera intrinsics to BA datastructs
// 	Matrix3x3d KMat;
// 	makeIdentityMatrix(KMat);
// 	KMat[0][0] = cam_matrix.at<double>(0,0); //fx
// 	KMat[1][1] = cam_matrix.at<double>(1,1); //fy
// 	KMat[0][1] = cam_matrix.at<double>(0,1); //skew
// 	KMat[0][2] = cam_matrix.at<double>(0,2); //ppx
// 	KMat[1][2] = cam_matrix.at<double>(1,2); //ppy

// 	ppx = KMat[0][2];
// 	ppy = KMat[1][2];

// 	double const f0 = KMat[0][0];
// 	double const f0_inv = 1.0/f0;

// 	//cout << "Cam = "<<endl;
// 	//displayMatrix(KMat);

// 	Matrix3x3d Knorm = KMat;
// 	// Normalize the intrinsic to have unit focal length.
// 	scaleMatrixIP(f0_inv, Knorm);
// 	Knorm[2][2] = 1.0;

// 	//convert 3D point cloud to BA datastructs
// 	const vector<LandMark::Ptr> &lms = data.getLandMarks();
// 	map<LandMark::Ptr, int, Data::LandMarkPtrCompare> lm2idx;
// 	vector<Vector3d > Xs;
// 	int idx = 0;
// 	Xs.reserve(lms.size());
// 	for (vector<LandMark::Ptr>::const_iterator it = lms.begin(); it!=lms.end(); ++it, ++idx)
// 	{
// 		assert(!(*it)->deleted);
// 		Xs.push_back(Vector3d((*it)->pt[0], (*it)->pt[1], (*it)->pt[2]));
// 		lm2idx[*it] = idx;
// 	}


// 	//convert cameras to BA datastructs
// 	const vector<Frame::Ptr> &frames = data.getFrames();
// 	map<Frame::Ptr, int, Data::FramePtrCompare> frame2idx;
// 	vector<CameraMatrix> cams;
// 	cams.reserve(frames.size());
// 	idx = 0;
// 	for (vector<Frame::Ptr>::const_iterator it = frames.begin(); it!=frames.end(); ++it, ++idx)
// 	{
// 		Matrix3x3d R;
// 		Vector3d T;

// 		Matx34d P = (*it)->getCVTransform();

// 		R[0][0] = P(0,0); R[0][1] = P(0,1); R[0][2] = P(0,2); T[0] = P(0,3);
// 		R[1][0] = P(1,0); R[1][1] = P(1,1); R[1][2] = P(1,2); T[1] = P(1,3);
// 		R[2][0] = P(2,0); R[2][1] = P(2,1); R[2][2] = P(2,2); T[2] = P(2,3);

// 		CameraMatrix cam;
// 		cam.setIntrinsic(Knorm);
// 		cam.setRotation(R);
// 		cam.setTranslation(T);
// 		cams.push_back(cam);
// 		frame2idx[*it] = idx;
// 	}


// 	const vector<Measurement::Ptr> &ms = data.getMeasurements();
// 	vector<Vector2d > measurements;
// 	vector<int> correspondingView; 		//corresponding camera mat idx
// 	vector<int> correspondingPoint;		//corresponding 3d point idx
// 	measurements.reserve(K);
// 	correspondingView.reserve(K);
// 	correspondingPoint.reserve(K);

// 	//convert 2D measurements to BA datastructs
// 	for (vector<Measurement::Ptr>::const_iterator it = ms.begin(); it!=ms.end(); ++it){
// 		assert(!(*it)->deleted);
// 		LandMark::Ptr &lm = (*it)->landmark;
// 		Frame::Ptr &frame = (*it)->frame;
// 		Point2f &xy		  = frame->kpts[(*it)->featureIdx].pt;
// 		measurements.push_back(Vector2d(xy.x*f0_inv, xy.y*f0_inv));
// 		correspondingView.push_back(frame2idx[frame]);
// 		correspondingPoint.push_back(lm2idx[lm]);
// 	}

// 	meanErrorBefore = showErrorStatistics(f0, distortion, cams, Xs, measurements, correspondingView, correspondingPoint);

// 	double const inlierThreshold = 2.0 / fabs(f0);

// 	Matrix3x3d K0 = cams[0].getIntrinsic();
// 	//cout << "K0 = "; displayMatrix(K0);

// 	bool good_adjustment = false;
// 	{
// 		ScopedBundleExtrinsicNormalizer extNorm(cams, Xs);
// 		ScopedBundleIntrinsicNormalizer intNorm(cams,measurements,correspondingView);
// 		CommonInternalsMetricBundleOptimizer opt(V3D::FULL_BUNDLE_FOCAL_LENGTH_PP, inlierThreshold, K0, distortion, cams, Xs,
// 												 measurements, correspondingView, correspondingPoint);

// 		opt.tau = 1e-3;
// 		opt.maxIterations = 50;
// 		opt.minimize();

// 		cout << "optimizer status = " << opt.status << endl;

// 		good_adjustment = (opt.status != 2);
// 	}

// 	//cout << "refined K = "; displayMatrix(K0);

// 	for (int i = 0; i < N; ++i) cams[i].setIntrinsic(K0);

// 	Matrix3x3d Knew = K0;
// 	scaleMatrixIP(f0, Knew);
// 	Knew[2][2] = 1.0;

// 	//cout << "Cam new = "<<endl;
// 	//displayMatrix(Knew);

// 	meanErrorAfter = showErrorStatistics(f0, distortion, cams, Xs, measurements, correspondingView, correspondingPoint);

// 	if(good_adjustment) {

// 		//extract 3D points
// 		for (unsigned int j = 0; j < Xs.size(); ++j)
// 		{
// 			lms[j]->pt[0] = Xs[j][0];
// 			lms[j]->pt[1] = Xs[j][1];
// 			lms[j]->pt[2] = Xs[j][2];
// 		}

// 		//extract adjusted cameras
// 		for (unsigned int i = 0; i < N; ++i)
// 		{
// 			Matrix3x3d R = cams[i].getRotation();
// 			Vector3d T = cams[i].getTranslation();

// 			Eigen::Matrix3d R_eigen;
// 			Eigen::Vector3d t_eigen;
// 			for(unsigned int r = 0; r<3; r++){
// 				for(unsigned int c = 0; c<3; c++){
// 					R_eigen(r,c) = R[r][c];
// 				}
// 				t_eigen(r) = T[r];
// 			}
// 			frames[i]->rotation = Eigen::Quaterniond(R_eigen);
// 			frames[i]->position = -R_eigen.transpose()*t_eigen;
// 		}

// 		cam_matrix.at<double>(0,0) = Knew[0][0];
// 		cam_matrix.at<double>(0,1) = Knew[0][1];
// 		cam_matrix.at<double>(0,2) = Knew[0][2];
// 		cam_matrix.at<double>(1,1) = Knew[1][1];
// 		cam_matrix.at<double>(1,2) = Knew[1][2];
// 	}

// 	cout<<"focal :"<<f0<<" -> "<<Knew[0][0]<<endl;
// 	cout<<"center:("<<ppx<<","<<ppy<<") -> ("<<Knew[0][2]<<","<<Knew[1][2]<<")"<<endl;
// 	cout<<"error :"<<meanErrorBefore<<" -> "<<meanErrorAfter<<endl;
// }

struct ReprojectionError {
	// (u, v): the position of the observation with respect to the image top left corner
	  // u is rightward positive and v is downward positive
	ReprojectionError(double observed_x, double observed_y, double f, double ppx, double ppy)
	:observed_x(observed_x)
	,observed_y(observed_y)
	,f(f)
	,ppx(ppx)
	,ppy(ppy)
	{

	}

	  template <typename T>
	  bool operator()(const T* const camera_rotation,
			  	  	  const T* const camera_translation,
	                  const T* const point,
	                  T* residuals) const {

		  //XXX: quaternion must be normalized before!
		  // Map the T* array to an Eigen Quaternion (no copy)
		  Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T>>(camera_rotation);

		  // Map T* to Eigen Vector3 (no copy)
		  Eigen::Matrix<T,3,1> t = Eigen::Map<const Eigen::Matrix<T,3,1>>(camera_translation);

		  //copy point data
		  Eigen::Matrix<T,3,1> p;
		  p << T(point[0]), T(point[1]), T(point[2]);

		  //transform point to camera view space
		  p = q*(p-t);	//q is rotation of camera back to world axis, t is position of camera in world (opengv convention)
		  //p = q*p+t		//q is rotation of camera back to world axis, t is translation of camera before rotation (opencv convention)

	    // Compute the center of distortion. The sign change comes from
	    // the camera model that Noah Snavely's Bundler assumes, whereby
	    // the camera coordinate system has a negative z axis.
	    const T xp = p[0] / p[2];
	    const T yp = p[1] / p[2];

	    // Compute final projected point position.
	    const T predicted_x = f * xp+ppx;
	    const T predicted_y = f * yp+ppy;
	    //const T predicted_x = -f * xp+ppx;
	    //const T predicted_y = f * yp+ppy;

	    // The error is the difference between the predicted and observed position.
	    residuals[0] = predicted_x - observed_x;
	    residuals[1] = predicted_y - observed_y;

	    return true;
	  }

	  // Factory to hide the construction of the CostFunction object from
	  // the client code.
	  static ceres::CostFunction* Create(const double observed_x,
	                                     const double observed_y,
										 const double f,
										 const double ppx,
										 const double ppy) {
	    return (new ceres::AutoDiffCostFunction<
	    		ReprojectionError, 2, 4, 3, 3>(
	                new ReprojectionError(observed_x,
	                                      observed_y,
										  f,
										  ppx,
										  ppy)));
	  }

	  double observed_x;
	  double observed_y;
	  //since we fix camera paramters, use them as residule rather than parameter
	  //this avoids unnecessary jacobian calculation which speeds up a lot
	  double f;
	  double ppx;
	  double ppy;
};


// struct OctaveAwareReprojectionError {
// 	// (u, v): the position of the observation with respect to the image top left corner
// 	  // u is rightward positive and v is downward positive
// 	OctaveAwareReprojectionError(double observed_x, double observed_y, double f, double ppx, double ppy, double octave)
// 	:observed_x(observed_x)
// 	,observed_y(observed_y)
// 	,f(f)
// 	,ppx(ppx)
// 	,ppy(ppy)
// 	,octave(octave)
// 	{

// 	}

// 	  template <typename T>
// 	  bool operator()(const T* const camera_rotation,
// 			  	  	  const T* const camera_translation,
// 	                  const T* const point,
// 	                  T* residuals) const {

// 		  //XXX: quaternion must be normalized before!
// 		  // Map the T* array to an Eigen Quaternion (no copy)
// 		  Eigen::Quaternion<T> q = Eigen::Map<const Eigen::Quaternion<T>>(camera_rotation);

// 		  // Map T* to Eigen Vector3 (no copy)
// 		  Eigen::Matrix<T,3,1> t = Eigen::Map<const Eigen::Matrix<T,3,1>>(camera_translation);

// 		  //copy point data
// 		  Eigen::Matrix<T,3,1> p;
// 		  p << T(point[0]), T(point[1]), T(point[2]);

// 		  //transform point to camera view space
// 		  p = q*(p-t);	//q is rotation of camera back to world axis, t is position of camera in world (opengv convention)
// 		  //p = q*p+t		//q is rotation of camera back to world axis, t is translation of camera before rotation (opencv convention)

// 	    // Compute the center of distortion. The sign change comes from
// 	    // the camera model that Noah Snavely's Bundler assumes, whereby
// 	    // the camera coordinate system has a negative z axis.
// 	    const T xp = p[0] / p[2];
// 	    const T yp = p[1] / p[2];

// 	    // Compute final projected point position.
// 	    const T predicted_x = f * xp+ppx;
// 	    const T predicted_y = f * yp+ppy;
// 	    //const T predicted_x = -f * xp+ppx;
// 	    //const T predicted_y = f * yp+ppy;

// 	    // The error is the difference between the predicted and observed position.
// 	    residuals[0] = (predicted_x - observed_x)/octave;
// 	    residuals[1] = (predicted_y - observed_y)/octave;

// 	    return true;
// 	  }

// 	  // Factory to hide the construction of the CostFunction object from
// 	  // the client code.
// 	  static ceres::CostFunction* Create(const double observed_x,
// 	                                     const double observed_y,
// 										 const double f,
// 										 const double ppx,
// 										 const double ppy,
// 										 const double octave) {
// 	    return (new ceres::AutoDiffCostFunction<
// 	    		OctaveAwareReprojectionError, 2, 4, 3, 3>(
// 	                new OctaveAwareReprojectionError(observed_x,
// 	                                      observed_y,
// 										  f,
// 										  ppx,
// 										  ppy,
// 										  octave)));
// 	  }

// 	  double observed_x;
// 	  double observed_y;
// 	  //since we fix camera paramters, use them as residule rather than parameter
// 	  //this avoids unnecessary jacobian calculation which speeds up a lot
// 	  double f;
// 	  double ppx;
// 	  double ppy;
// 	  double octave;
// };


void BAHandler::adjustBundle_ceres_nocopy(){
	cout<<"Bundle Adjust Method: no copy"<<endl;

	clock_t				time;
	double				t_preprocess;

	time				= clock();
	Data 		&data 	= Data::GetInstance();
	Camera 		&camera	= Camera::GetInstance();
	//important
	data.deleteTrashes();

	double intrinsics[3];
	intrinsics[0] = camera.getCamFocal();
	intrinsics[1] = camera.getCamPrinciple().x;
	intrinsics[2] = camera.getCamPrinciple().y;

	ceres::Problem problem;
	ceres::LocalParameterization *eigenQuaternionParameterization = new ceres::EigenQuaternionParameterization;

	const vector<Measurement::Ptr> &ms = data.getMeasurements();
	for(vector<Measurement::Ptr>::const_iterator it = ms.begin(); it!=ms.end(); ++it){

		assert(!((*it)->deleted));

		Frame::Ptr 		&f 	= (*it)->frame;
		LandMark::Ptr 	&p 	= (*it)->landmark;
		int			&feIdx	= (*it)->featureIdx;

		Point2f &pt2D 	= f->kpts[feIdx].pt;
		double *q		= f->rotation.coeffs().data();
		double *t		= f->position.data();
		double *point	= p->pt.data();

		ceres::CostFunction* cost_function = ReprojectionError::Create( pt2D.x, pt2D.y, intrinsics[0], intrinsics[1], intrinsics[2]);
		problem.AddResidualBlock(cost_function, NULL, q, t, point);
		problem.SetParameterization(q,eigenQuaternionParameterization);
		if(f->fixed){
			//fix first camera transformation
			problem.SetParameterBlockConstant(q);
			problem.SetParameterBlockConstant(t);
		}
	}

	t_preprocess = double(clock()-time) / CLOCKS_PER_SEC;

	// Set a few options
	ceres::Solver::Options options;
	options.use_nonmonotonic_steps = true;
	options.preconditioner_type = ceres::SCHUR_JACOBI;
	options.linear_solver_type = ceres::SPARSE_SCHUR; //ceres::ITERATIVE_SCHUR; //
	options.max_num_iterations = 100;
	//options.max_solver_time_in_seconds = 0.015;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	//std::cout << "Final report:\n" << summary.FullReport();
	std::cout << "Time(s) preprocess :"<<t_preprocess<<endl;
	std::cout << "Time(s) BA :"<<summary.total_time_in_seconds<<endl;


}

void BAHandler::adjustBundle_ceres_local_nocopy(){

	cout<<"Bundle Adjust Method: local no copy"<<endl;

	clock_t				time;
	double				t_preprocess;

	time				= clock();
	Data 		&data 	= Data::GetInstance();
	Camera 		&camera	= Camera::GetInstance();
	//important
	data.deleteTrashes();


	double intrinsics[3];
	intrinsics[0] = camera.getCamFocal();
	intrinsics[1] = camera.getCamPrinciple().x;
	intrinsics[2] = camera.getCamPrinciple().y;

	ceres::Problem problem;
	ceres::LocalParameterization *eigenQuaternionParameterization = new ceres::EigenQuaternionParameterization;


	if(data.countFrames()<1) return;
	//fix all frames other than the last frame
	data.fixAllFrames();
	Frame::Ptr lastAddedFrame = data.getFrames().back();
	lastAddedFrame -> fixed = false;

	//for all measures in last added frame
	const vector<Measurement::Ptr> ms = data.getMeasurements(lastAddedFrame);
	for(vector<Measurement::Ptr>::const_iterator it = ms.begin(); it!=ms.end(); ++it){
		LandMark::Ptr &lmk = (*it)->landmark;
		//for all measurements tied to this landmark
		vector<Measurement::Ptr> lmkms = data.getMeasurements(lmk);
		for(vector<Measurement::Ptr>::const_iterator jt = lmkms.begin(); jt!=lmkms.end(); ++jt){
			assert(!((*jt)->deleted));

			Frame::Ptr 		&f 	= (*jt)->frame;
			LandMark::Ptr 	&p 	= (*jt)->landmark;
			int			&feIdx	= (*jt)->featureIdx;

			Point2f &pt2D 	= f->kpts[feIdx].pt;
			double *q		= f->rotation.coeffs().data();
			double *t		= f->position.data();
			double *point	= p->pt.data();

			ceres::CostFunction* cost_function = ReprojectionError::Create( pt2D.x, pt2D.y, intrinsics[0], intrinsics[1], intrinsics[2]);
			problem.AddResidualBlock(cost_function, NULL, q, t, point);
			problem.SetParameterization(q,eigenQuaternionParameterization);
			if(f->fixed){
				problem.SetParameterBlockConstant(q);
				problem.SetParameterBlockConstant(t);
			}
		}
	}

	t_preprocess = double(clock()-time) / CLOCKS_PER_SEC;

	// Set a few options
	ceres::Solver::Options options;
	options.use_nonmonotonic_steps = true;
	options.preconditioner_type = ceres::SCHUR_JACOBI;
	options.linear_solver_type = ceres::SPARSE_SCHUR; //ceres::ITERATIVE_SCHUR; //
	options.max_num_iterations = 100;
	//options.max_solver_time_in_seconds = 0.015;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	//std::cout << "Final report:\n" << summary.FullReport();
	std::cout << "Time(s) preprocess :"<<t_preprocess<<endl;
	std::cout << "Time(s) BA :"<<summary.total_time_in_seconds<<endl;
}

void BAHandler::adjustBundle_ceres_local_nocopy(vector<Measurement::Ptr> &ms){

	cout<<"Bundle Adjust Method: local no copy"<<endl;

	clock_t				time;
	double				t_preprocess;

	time				= clock();
	Data 		&data 	= Data::GetInstance();
	Camera 		&camera	= Camera::GetInstance();
	//important
	data.deleteTrashes();


	double intrinsics[3];
	intrinsics[0] = camera.getCamFocal();
	intrinsics[1] = camera.getCamPrinciple().x;
	intrinsics[2] = camera.getCamPrinciple().y;

	ceres::Problem problem;
	ceres::LocalParameterization *eigenQuaternionParameterization = new ceres::EigenQuaternionParameterization;


	if(data.countFrames()<1) return;
	if(ms.empty()) return;
	//fix all frames, the provided measurements should all generate from a frame that is not in data
	data.fixAllFrames();
	//get measures already added to data if exists
	Frame::Ptr &mFrame = ms[0]->frame;
	//unfix this measure frame
	mFrame->fixed = false;

	//for all given measures
	for(vector<Measurement::Ptr>::const_iterator it = ms.begin(); it!=ms.end(); ++it){
		assert((*it)->frame.get() == mFrame.get());
		LandMark::Ptr &lmk = (*it)->landmark;
		//for all measurements tied to this landmark
		vector<Measurement::Ptr> lmkms = data.getMeasurements(lmk);
		for(vector<Measurement::Ptr>::iterator jt = lmkms.begin(); jt!=lmkms.end(); ++jt){
			assert(!((*jt)->deleted));

			Frame::Ptr 		&f 	= (*jt)->frame;
			LandMark::Ptr 	&p 	= (*jt)->landmark;
			int			&feIdx	= (*jt)->featureIdx;

			Point2f &pt2D 	= f->kpts[feIdx].pt;
			double *q		= f->rotation.coeffs().data();
			double *t		= f->position.data();
			double *point	= p->pt.data();

			ceres::CostFunction* cost_function = ReprojectionError::Create( pt2D.x, pt2D.y, intrinsics[0], intrinsics[1], intrinsics[2]);
			problem.AddResidualBlock(cost_function, NULL, q, t, point);
			problem.SetParameterization(q,eigenQuaternionParameterization);
			if(f->fixed){
				problem.SetParameterBlockConstant(q);
				problem.SetParameterBlockConstant(t);
			}
		}
	}

	//if mFrame was already added to data, we add these measures as well for bundle adjustment
	vector<Measurement::Ptr> dataMs = data.getMeasurements(mFrame);
	for(vector<Measurement::Ptr>::const_iterator it = dataMs.begin(); it!=dataMs.end(); ++it){
		assert((*it)->frame.get() == mFrame.get());
		LandMark::Ptr &lmk = (*it)->landmark;
		//for all measurements tied to this landmark
		vector<Measurement::Ptr> lmkms = data.getMeasurements(lmk);
		for(vector<Measurement::Ptr>::iterator jt = lmkms.begin(); jt!=lmkms.end(); ++jt){
			assert(!((*jt)->deleted));

			Frame::Ptr 		&f 	= (*jt)->frame;
			LandMark::Ptr 	&p 	= (*jt)->landmark;
			int			&feIdx	= (*jt)->featureIdx;

			Point2f &pt2D 	= f->kpts[feIdx].pt;
			double *q		= f->rotation.coeffs().data();
			double *t		= f->position.data();
			double *point	= p->pt.data();

			ceres::CostFunction* cost_function = ReprojectionError::Create( pt2D.x, pt2D.y, intrinsics[0], intrinsics[1], intrinsics[2]);
			problem.AddResidualBlock(cost_function, NULL, q, t, point);
			problem.SetParameterization(q,eigenQuaternionParameterization);
			if(f->fixed){
				problem.SetParameterBlockConstant(q);
				problem.SetParameterBlockConstant(t);
			}
		}
	}


	t_preprocess = double(clock()-time) / CLOCKS_PER_SEC;

	// Set a few options
	ceres::Solver::Options options;
	options.use_nonmonotonic_steps = true;
	options.preconditioner_type = ceres::SCHUR_JACOBI;
	options.linear_solver_type = ceres::SPARSE_SCHUR; //ceres::ITERATIVE_SCHUR; //
	options.max_num_iterations = 100;
	//options.max_solver_time_in_seconds = 0.015;

	ceres::Solver::Summary summary;
	ceres::Solve(options, &problem, &summary);

	//std::cout << "Final report:\n" << summary.FullReport();
	std::cout << "Time(s) preprocess :"<<t_preprocess<<endl;
	std::cout << "Time(s) BA :"<<summary.total_time_in_seconds<<endl;
}

// void BAHandler::adjustBundle_ceres_local_fixPoints_nocopy(vector<Measurement::Ptr> &ms){

// 	cout<<"Bundle Adjust Method: local fix points no copy"<<endl;

// 	clock_t				time;
// 	double				t_preprocess;

// 	time				= clock();
// 	Data 		&data 	= Data::GetInstance();
// 	Camera 		&camera	= Camera::GetInstance();
// 	//important
// 	data.deleteTrashes();


// 	double intrinsics[3];
// 	intrinsics[0] = camera.getCamFocal();
// 	intrinsics[1] = camera.getCamPrinciple().x;
// 	intrinsics[2] = camera.getCamPrinciple().y;

// 	ceres::Problem problem;
// 	ceres::LocalParameterization *eigenQuaternionParameterization = new ceres::EigenQuaternionParameterization;


// 	if(data.countFrames()<1) return;
// 	if(ms.empty()) return;


// 	//for all given measures
// 	for(vector<Measurement::Ptr>::const_iterator it = ms.begin(); it!=ms.end(); ++it){

// 		Frame::Ptr 		&f 	= (*it)->frame;
// 		LandMark::Ptr 	&p 	= (*it)->landmark;
// 		int			&feIdx	= (*it)->featureIdx;
// 		double 		oct		= (double) (f->kpts[feIdx].octave + 1);

// 		Point2f &pt2D 	= f->kpts[feIdx].pt;
// 		double *q		= f->rotation.coeffs().data();
// 		double *t		= f->position.data();
// 		double *point	= p->pt.data();

// 		ceres::CostFunction* cost_function = OctaveAwareReprojectionError::Create( pt2D.x, pt2D.y, intrinsics[0], intrinsics[1], intrinsics[2], oct);
// 		problem.AddResidualBlock(cost_function, NULL, q, t, point);
// 		problem.SetParameterization(q,eigenQuaternionParameterization);
// 		problem.SetParameterBlockConstant(point);

// 	}


// /*

// 	//fix all frames, the provided measurements should all generate from a frame that is not in data
// 	data.fixAllFrames();
// 	//get measures already added to data if exists
// 	Frame::Ptr &mFrame = ms[0]->frame;
// 	//unfix this measure frame
// 	mFrame->fixed = false;

// 	//for all given measures
// 	for(vector<Measurement::Ptr>::const_iterator it = ms.begin(); it!=ms.end(); ++it){
// 		assert((*it)->frame.get() == mFrame.get());
// 		LandMark::Ptr &lmk = (*it)->landmark;
// 		//for all measurements tied to this landmark
// 		vector<Measurement::Ptr> lmkms = data.getMeasurements(lmk);
// 		for(vector<Measurement::Ptr>::iterator jt = lmkms.begin(); jt!=lmkms.end(); ++jt){
// 			assert(!((*jt)->deleted));

// 			Frame::Ptr 		&f 	= (*jt)->frame;
// 			LandMark::Ptr 	&p 	= (*jt)->landmark;
// 			int			&feIdx	= (*jt)->featureIdx;

// 			Point2f &pt2D 	= f->kpts[feIdx].pt;
// 			double *q		= f->rotation.coeffs().data();
// 			double *t		= f->position.data();
// 			double *point	= p->pt.data();

// 			ceres::CostFunction* cost_function = ReprojectionError::Create( pt2D.x, pt2D.y, intrinsics[0], intrinsics[1], intrinsics[2]);
// 			problem.AddResidualBlock(cost_function, NULL, q, t, point);
// 			problem.SetParameterization(q,eigenQuaternionParameterization);
// 			problem.SetParameterBlockConstant(point);
// 			if(f->fixed){
// 				problem.SetParameterBlockConstant(q);
// 				problem.SetParameterBlockConstant(t);
// 			}
// 		}
// 	}

// 	//if mFrame was already added to data, we add these measures as well for bundle adjustment
// 	vector<Measurement::Ptr> dataMs = data.getMeasurements(mFrame);
// 	for(vector<Measurement::Ptr>::const_iterator it = dataMs.begin(); it!=dataMs.end(); ++it){
// 		assert((*it)->frame.get() == mFrame.get());
// 		LandMark::Ptr &lmk = (*it)->landmark;
// 		//for all measurements tied to this landmark
// 		vector<Measurement::Ptr> lmkms = data.getMeasurements(lmk);
// 		for(vector<Measurement::Ptr>::iterator jt = lmkms.begin(); jt!=lmkms.end(); ++jt){
// 			assert(!((*jt)->deleted));

// 			Frame::Ptr 		&f 	= (*jt)->frame;
// 			LandMark::Ptr 	&p 	= (*jt)->landmark;
// 			int			&feIdx	= (*jt)->featureIdx;

// 			Point2f &pt2D 	= f->kpts[feIdx].pt;
// 			double *q		= f->rotation.coeffs().data();
// 			double *t		= f->position.data();
// 			double *point	= p->pt.data();

// 			ceres::CostFunction* cost_function = ReprojectionError::Create( pt2D.x, pt2D.y, intrinsics[0], intrinsics[1], intrinsics[2]);
// 			problem.AddResidualBlock(cost_function, NULL, q, t, point);
// 			problem.SetParameterization(q,eigenQuaternionParameterization);
// 			problem.SetParameterBlockConstant(point);
// 			if(f->fixed){
// 				problem.SetParameterBlockConstant(q);
// 				problem.SetParameterBlockConstant(t);
// 			}
// 		}
// 	}
// */

// 	t_preprocess = double(clock()-time) / CLOCKS_PER_SEC;

// 	// Set a few options
// 	ceres::Solver::Options options;
// 	options.use_nonmonotonic_steps = true;
// 	options.preconditioner_type = ceres::SCHUR_JACOBI;
// 	options.linear_solver_type = ceres::SPARSE_SCHUR; //ceres::ITERATIVE_SCHUR; //
// 	options.max_num_iterations = 100;
// 	//options.max_solver_time_in_seconds = 0.015;

// 	ceres::Solver::Summary summary;
// 	ceres::Solve(options, &problem, &summary);

// 	std::cout << "Final report:\n" << summary.FullReport();
// 	std::cout << "Time(s) preprocess :"<<t_preprocess<<endl;
// 	std::cout << "Time(s) BA :"<<summary.total_time_in_seconds<<endl;
// }


