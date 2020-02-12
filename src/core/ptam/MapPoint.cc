// Copyright 2008 Isis Innovation Limited
#include "MapPoint.h"
#include "KeyFrame.h"
//#include "MapMaker.h"


void MapPoint::RefreshPixelVectors()
{
  KeyFrame &k = *pPatchSourceKF;

  // Find patch pos in KF camera coords
  // Actually this might not exactly correspond to the patch pos!
  // Treat it as a general point on the plane.
  Vector<3> v3PlanePoint_C = k.se3CfromW * v3WorldPos;

  // Find the height of this above the plane.
  // Assumes the normal is  pointing toward the camera.
  double dCamHeight = fabs(v3PlanePoint_C * v3Normal_NC);

  double dPixelRate = fabs(v3Center_NC * v3Normal_NC);
  double dOneRightRate = fabs(v3OneRightFromCenter_NC * v3Normal_NC);
  double dOneDownRate = fabs(v3OneDownFromCenter_NC * v3Normal_NC);

  // Find projections onto plane
  Vector<3> v3CenterOnPlane_C = v3Center_NC * dCamHeight / dPixelRate;
  Vector<3> v3OneRightOnPlane_C = v3OneRightFromCenter_NC * dCamHeight / dOneRightRate;
  Vector<3> v3OneDownOnPlane_C = v3OneDownFromCenter_NC * dCamHeight / dOneDownRate;

  // Find differences of these projections in the world frame
  v3PixelRight_W = k.se3CfromW.get_rotation().inverse() * (v3OneRightOnPlane_C - v3CenterOnPlane_C);
  v3PixelDown_W = k.se3CfromW.get_rotation().inverse() * (v3OneDownOnPlane_C - v3CenterOnPlane_C);
}  
/*
XMLElement* MapPoint::save(MapSerializationHelper& helper, int mpid)
{
  XMLDocument* doc = helper.GetXMLDocument();
  XMLElement *mappoint = doc->NewElement("MapPoint");
  mappoint->SetAttribute("ID",mpid);
  mappoint->SetAttribute("v3WorldPos",helper.saveVector(v3WorldPos).c_str());
  mappoint->SetAttribute("bBad",bBad);
  mappoint->SetAttribute("pPatchSourceKF",helper.GetKeyFrameID(pPatchSourceKF));
  mappoint->SetAttribute("nSourceLevel",nSourceLevel);
  mappoint->SetAttribute("irCenter",helper.saveImageRef(irCenter).c_str());
  mappoint->SetAttribute("v3Center_NC",helper.saveVector(v3Center_NC).c_str());
  mappoint->SetAttribute("v3OneDownFromCenter_NC",helper.saveVector(v3OneDownFromCenter_NC).c_str());
  mappoint->SetAttribute("v3OneRightFromCenter_NC",helper.saveVector(v3OneRightFromCenter_NC).c_str());
  mappoint->SetAttribute("v3Normal_NC",helper.saveVector(v3Normal_NC).c_str());
  mappoint->SetAttribute("v3PixelDown_W",helper.saveVector(v3PixelDown_W).c_str());
  mappoint->SetAttribute("v3PixelRight_W",helper.saveVector(v3PixelRight_W).c_str());

  mappoint->SetAttribute("nMEstimatorOutlierCount",nMEstimatorOutlierCount);
  mappoint->SetAttribute("nMEstimatorInlierCount",nMEstimatorInlierCount);

  if(pMMData!=NULL)
  {
	  XMLElement *mapmakerdata = pMMData->save(helper);
	  mappoint->InsertEndChild(mapmakerdata);
  }

  return mappoint;
}

void MapPoint::load(const XMLElement* mappoint, MapSerializationHelper& helper)
{
  XMLDocument* doc = helper.GetXMLDocument();

  v3WorldPos = helper.loadVector<3>(string(mappoint->FindAttribute("v3WorldPos")->Value()));
  mappoint->QueryAttribute("bBad",&bBad);
  int kfid;
  mappoint->QueryAttribute("pPatchSourceKF",&kfid);
  pPatchSourceKF = KeyFrame::Ptr(helper.GetKeyFrame(kfid));
  mappoint->QueryAttribute("nSourceLevel",&nSourceLevel);
  irCenter = helper.loadImageRef(string(mappoint->FindAttribute("irCenter")->Value()));
  v3Center_NC = helper.loadVector<3>(string(mappoint->FindAttribute("v3Center_NC")->Value()));
  v3OneDownFromCenter_NC = helper.loadVector<3>(string(mappoint->FindAttribute("v3OneDownFromCenter_NC")->Value()));
  v3OneRightFromCenter_NC = helper.loadVector<3>(string(mappoint->FindAttribute("v3OneRightFromCenter_NC")->Value()));
  v3Normal_NC = helper.loadVector<3>(string(mappoint->FindAttribute("v3Normal_NC")->Value()));
  v3PixelDown_W = helper.loadVector<3>(string(mappoint->FindAttribute("v3PixelDown_W")->Value()));
  v3PixelRight_W = helper.loadVector<3>(string(mappoint->FindAttribute("v3PixelRight_W")->Value()));

  mappoint->QueryAttribute("nMEstimatorOutlierCount",&nMEstimatorOutlierCount);
  mappoint->QueryAttribute("nMEstimatorInlierCount",&nMEstimatorInlierCount);

  const XMLElement* mmdata = mappoint->FirstChildElement("MapMakerData");
  if(mmdata!=NULL)
  {
	  pMMData = new MapMakerData();
	  pMMData->load(mmdata,helper);
  }
  else
  {
	  cout<<"map point loading: "<<"no pmmdata found!!!"<<endl;
  }

}
*/
