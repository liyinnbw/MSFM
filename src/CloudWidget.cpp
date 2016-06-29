/*
 *  Created on: Mar 11, 2016
 *      Author: yoyo
 */

#include <QtGui>

#include <vtkVersion.h>
#include <vtkSmartPointer.h>
#include <vtkPointData.h>
#include <vtkIdTypeArray.h>
#include <vtkDataSetSurfaceFilter.h>
#include <vtkRendererCollection.h>
#include <vtkProperty.h>
#include <vtkPlanes.h>
#include <vtkObjectFactory.h>
#include <vtkPoints.h>
#include <vtkLine.h>
#include <vtkTriangle.h>
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkCellData.h>
#include <vtkPointSource.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkAreaPicker.h>
#include <vtkExtractGeometry.h>
#include <vtkDataSetMapper.h>
#include <vtkUnstructuredGrid.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkIdFilter.h>



#include "core/PlyIO.h"
#include "core/Utils.h"
#include "CloudWidget.h"

using namespace std;
using namespace cv;

// Define interaction style
class InteractorStyle : public vtkInteractorStyleRubberBandPick
{
  public:
    static InteractorStyle* New();
    vtkTypeMacro(InteractorStyle,vtkInteractorStyleRubberBandPick);

    InteractorStyle()
    {
      this->SelectedMapper = vtkSmartPointer<vtkDataSetMapper>::New();
      this->SelectedActor = vtkSmartPointer<vtkActor>::New();
      this->SelectedActor->SetMapper(SelectedMapper);
	  isActive = false;
    }

	virtual void OnKeyPress() 
    {
      // Get the keypress
      vtkRenderWindowInteractor *rwi = this->Interactor;
      std::string key = rwi->GetKeySym();
 	  /*
      // Output the key that was pressed
      std::cout << "Pressed " << key << std::endl;
 
      // Handle an arrow key
      if(key == "Up")
        {
        std::cout << "The up arrow was pressed." << std::endl;
        }
 	  */

      if(key == "d"){
        deleteSelectedPoints();
      }else if (key == "c"){
    	  showCameras();
      }
 
      // Forward events
      vtkInteractorStyleRubberBandPick::OnKeyPress();	//default r toggles selection
    }
	
	virtual void OnMouseMove(){
		if(!isActive){
			return;
		}
		// Forward events
		vtkInteractorStyleRubberBandPick::OnMouseMove();
	}
	virtual void OnLeftButtonDown(){
		if(!isActive){
			return;
		}
		// Forward events
		vtkInteractorStyleRubberBandPick::OnLeftButtonDown();
	}

    virtual void OnLeftButtonUp()
    {
	  if(!isActive){
		return;
	  }
      // Forward events
      vtkInteractorStyleRubberBandPick::OnLeftButtonUp();

      vtkPlanes* frustum = static_cast<vtkAreaPicker*>(this->GetInteractor()->GetPicker())->GetFrustum();

      vtkSmartPointer<vtkExtractGeometry> extractGeometry =
        vtkSmartPointer<vtkExtractGeometry>::New();
      extractGeometry->SetImplicitFunction(frustum);
#if VTK_MAJOR_VERSION <= 5
      extractGeometry->SetInput(this->Points);
#else
      extractGeometry->SetInputData(this->Points);
#endif
      extractGeometry->Update();

      vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
        vtkSmartPointer<vtkVertexGlyphFilter>::New();
      glyphFilter->SetInputConnection(extractGeometry->GetOutputPort());
      glyphFilter->Update();

      vtkPolyData* selected = glyphFilter->GetOutput();
      //std::cout << "Selected " << selected->GetNumberOfPoints() << " points." << std::endl;
      //std::cout << "Selected " << selected->GetNumberOfCells() << " cells." << std::endl;
#if VTK_MAJOR_VERSION <= 5
      this->SelectedMapper->SetInput(selected);
#else
      this->SelectedMapper->SetInputData(selected);
#endif
      this->SelectedMapper->ScalarVisibilityOff();

      //TODO: comment out the following debug
      vtkIdTypeArray* ids = vtkIdTypeArray::SafeDownCast(selected->GetPointData()->GetArray("OriginalIds"));
      for(vtkIdType i = 0; i < ids->GetNumberOfTuples(); i++)
      {
    	int originalID = ids->GetValue(i);
		if(	originalID>=parentWidget->numCloudPoints){
			int camIdx = (originalID-parentWidget->numCloudPoints)/4;
			std::cout << "selected camera " << camIdx << std::endl;
		}else{
			//std::cout << "point " << i << " : " << originalID<<" is a camera point" << std::endl;
		}
      }
        
      this->SelectedActor->GetProperty()->SetColor(1.0, 0.0, 0.0); //(R,G,B)
      this->SelectedActor->GetProperty()->SetPointSize(CloudWidget::POINT_SIZE);

      this->CurrentRenderer->AddActor(SelectedActor);
      this->GetInteractor()->GetRenderWindow()->Render();
      this->HighlightProp(NULL);
    }

    void SetPoints(vtkSmartPointer<vtkPolyData> points) {
		clearSelection();	
		//TODO: turn off selection
		this->Points = points;
	}
	void setActive(bool val){isActive = val;}
	void showCameras(){
		vtkIdTypeArray* ids = vtkIdTypeArray::SafeDownCast(SelectedMapper->GetInput()->GetPointData()->GetArray("OriginalIds"));
		QList<int> deleteIdxs;
		deleteIdxs.reserve(ids->GetNumberOfTuples());

		for(vtkIdType i = 0; i < ids->GetNumberOfTuples(); i++)
		{
			int originalID = ids->GetValue(i);
			if(	originalID<parentWidget->numCloudPoints){
			//std::cout << "delete " << i << " : " << ids->GetValue(i) << std::endl;
				deleteIdxs.push_back(ids->GetValue(i));
			}else{
				//std::cout << "point " << i << " : " << originalID<<" is a camera point" << std::endl;
			}
		}
		parentWidget->showCameras(deleteIdxs);
	}
	void deleteSelectedPoints(){
		vtkIdTypeArray* ids = vtkIdTypeArray::SafeDownCast(SelectedMapper->GetInput()->GetPointData()->GetArray("OriginalIds"));
		QList<int> deleteIdxs;
		deleteIdxs.reserve(ids->GetNumberOfTuples());

		for(vtkIdType i = 0; i < ids->GetNumberOfTuples(); i++)
        {
			int originalID = ids->GetValue(i);
			if(	originalID<parentWidget->numCloudPoints){
        	//std::cout << "delete " << i << " : " << ids->GetValue(i) << std::endl;
				deleteIdxs.push_back(ids->GetValue(i));
        	}else{
        		//std::cout << "point " << i << " : " << originalID<<" is a camera point" << std::endl;
        	}
        }
		parentWidget->deletePoints(deleteIdxs);

	}
	void setWidget(CloudWidget *widget){
		parentWidget = widget;
	}
	void clearSelection(){
		vtkPolyData* selected = vtkPolyData::New();
#if VTK_MAJOR_VERSION <= 5
      this->SelectedMapper->SetInput(selected);
#else
      this->SelectedMapper->SetInputData(selected);
#endif
      this->SelectedMapper->ScalarVisibilityOff();
	}
  private:
	CloudWidget *parentWidget;
    vtkSmartPointer<vtkPolyData> Points;
    vtkSmartPointer<vtkActor> SelectedActor;
    vtkSmartPointer<vtkDataSetMapper> SelectedMapper;
	bool isActive;

};
vtkStandardNewMacro(InteractorStyle);


// Constructor
CloudWidget:: CloudWidget(QWidget *parent)
				 : QVTKWidget(parent)
{
  
  // Create a mapper and actor
  mapper = vtkPolyDataMapper::New();
  //mapper->ScalarVisibilityOff();
  mapper->ScalarVisibilityOn(); //default on, if off, scalar color will not be applied, all geometry will be white
  actor = vtkActor::New();
  actor->SetMapper(mapper);
  
  // Create renderer and render window
  renderer = vtkRenderer::New();
  renderer->AddActor(actor);
  renderer->SetBackground(0,0,0); //(R,G,B) in range (0.0,0.0,0.0) ~ (1.0,1.0,1.0), default black
  //QT VtkWidget already has a renderWindow need not to create a new one.
  //vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow = GetRenderWindow();
  renderWindow->AddRenderer(renderer);


  //QT VtkWidget already has a renderWindowInteractor need not to create a new one.
  //vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor = vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor = GetInteractor();
  areaPicker = vtkAreaPicker::New();
  renderWindowInteractor->SetPicker(areaPicker);
  renderWindowInteractor->SetRenderWindow(renderWindow);

  //this is not needed for vtkWidget
  //renderWindow->Render();

  style = InteractorStyle::New();
  renderWindowInteractor->SetInteractorStyle( style );
  style->setWidget(this);

  //this is not needed for vtkWidget
  //renderWindowInteractor->Start();

  disableInteraction(); //initially do not handle input event

  numCloudPoints = 0;
}
void CloudWidget::deletePoints(const QList<int> idxs){

	disableInteraction(); //prevent further ui inputs untill refresh
	emit deletePointIdx(idxs);
}
void CloudWidget::showCameras(const QList<int> idxs){
	emit showCamerasSeeingPoints(idxs);
}
void CloudWidget::highlightPointIdx(const QList<int> idxs, const int camIdx){
	if(pointsData == NULL) return;
	//cout<<idxs.size()<<" points seen by camera "<<camIdx<<endl;
	vtkSmartPointer<vtkCellArray> vertices = pointsData->GetVerts();
	vtkIdType numOfPoints;
	vtkIdType *pointIDs;
	vertices->GetCell(0, numOfPoints,pointIDs);
	//int numCells = vertices->GetNumberOfCells();
	//for(int i=0; i<numCells; i++){
	//	vtkIdType numOfPoints;
	//	vtkIdType *pointIDs;
	//	vertices->GetCell(i, numOfPoints,pointIDs);
	//	cout<<"cell "<<i<<" vertices = "<<numOfPoints<<endl;
	//}

	vtkIdType highlightPid[idxs.size()];
	for(int i=0; i<idxs.size(); i++){
		highlightPid[i] = idxs[i];
	}
	vtkIdType newVertsCellId = vertices->InsertNextCell(idxs.size(),highlightPid);

	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkUnsignedCharArray::SafeDownCast(pointsData->GetCellData()->GetScalars("Colors"));

	unsigned char highlightColor[3] = {Utils::getRandomInt(0,255), Utils::getRandomInt(0,255), Utils::getRandomInt(0,255)};	//random color

	//just to add a new tuple, value doesnt matter
	#if VTK_MAJOR_VERSION < 7
		colors->InsertNextTupleValue(highlightColor);
	#else
		colors->InsertNextTypedTuple(highlightColor);
	#endif

	//shift tuples with idx>= 1 to the right by 1
	for(int i=colors->GetNumberOfTuples()-1; i>newVertsCellId; i--){
		colors->InsertTuple(i,colors->GetTuple(i-1));
	}

	//add the new tuple to idx 1
	#if VTK_MAJOR_VERSION < 7
		colors->InsertTupleValue(newVertsCellId, highlightColor);
	#else
		colors->InsertTypedTuple(newVertsCellId, highlightColor);
	#endif


	vtkSmartPointer<vtkCellArray> edges = pointsData->GetLines();
	int edgeCntBefore 	= edges->GetNumberOfCells();
	int colorCntBefore 	= colors->GetNumberOfTuples();
	vtkIdType camCenterPtId = numOfPoints+camIdx*4;
	for(int i=0; i<idxs.size(); i++){
		vtkSmartPointer<vtkLine> edge = vtkSmartPointer<vtkLine>::New();
		edge->GetPointIds()->SetId(0, camCenterPtId);
		edge->GetPointIds()->SetId(1, idxs[i]);
		edges->InsertNextCell(edge);
	#if VTK_MAJOR_VERSION < 7
		colors->InsertNextTupleValue(highlightColor);
	#else
		colors->InsertNextTypedTuple(highlightColor);
	#endif
	}
	int edgeCntAfter 	= edges->GetNumberOfCells();
	int colorCntAfter 	= colors->GetNumberOfTuples();
	assert(edgeCntBefore+idxs.size() == edgeCntAfter);
	assert(colorCntBefore+idxs.size() == colorCntAfter);
	pointsData->Modified(); //to trigger an UI update
}
void CloudWidget::disableInteraction(){
	style->setActive(false);
}
void CloudWidget::enableInteraction(){
	style->setActive(true);
}
void CloudWidget::loadPolygonAndCamera(const vector<Point3f> &verts, const vector<Point3i> &faces, const vector<Matx34d> &cameras, bool resetView){
	disableInteraction();
	//constants
	unsigned char red[3] = {255, 0, 0};
	unsigned char green[3] = {0, 255, 0};
	unsigned char blue[3] = {0, 0, 255};
	unsigned char white[3] = {255, 255, 255};
	unsigned char yellow[3] = {255, 255, 0};

	//insert points to vtk data structure
	// Create the geometry of a point (the coordinate)
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	// Create the topology of the point (a vertex)
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
	// Create the topology of the line (an edge)
	vtkSmartPointer<vtkCellArray> edges = vtkSmartPointer<vtkCellArray>::New();
	// Create the topology of the triangle
	vtkSmartPointer<vtkCellArray> triangles = vtkSmartPointer<vtkCellArray>::New();
	// Create colors
	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName ("Colors");

	//add vertex points
	numCloudPoints = verts.size();
	cout<<"numCloudPoints = "<<numCloudPoints<<endl;
	vtkIdType pid[numCloudPoints];//temp array to store point ids for create vertices
	for(int i=0; i<numCloudPoints; i++){
		pid[i] = points->InsertNextPoint(verts[i].x, verts[i].y, verts[i].z);
	}

	//create vertices from point ids
	vertices->InsertNextCell(numCloudPoints,pid);
	int numCells = vertices->GetNumberOfCells();
	for(int i=0; i<numCells; i++){
		vtkIdType numOfPoints;
		vtkIdType *pointIDs;
		vertices->GetCell(i, numOfPoints,pointIDs);
		assert(numOfPoints == numCloudPoints);
	}

	//inert cell color for the vertices
	#if VTK_MAJOR_VERSION < 7
		colors->InsertNextTupleValue(white);
	#else
		colors->InsertNextTypedTuple(white);
	#endif


	//add camera points & edges
	for(int n=0; n<cameras.size(); n++){
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

		//create & insert points
		vtkIdType idT    = points->InsertNextPoint(Tx, Ty, Tz);
		vtkIdType idI    = points->InsertNextPoint(Ix, Iy, Iz);
		vtkIdType idJ    = points->InsertNextPoint(Jx, Jy, Jz);
		vtkIdType idK    = points->InsertNextPoint(Kx, Ky, Kz);

		//create edges
		vtkSmartPointer<vtkLine> edgeI = vtkSmartPointer<vtkLine>::New();
		edgeI->GetPointIds()->SetId(0, idT);
		edgeI->GetPointIds()->SetId(1, idI);
		vtkSmartPointer<vtkLine> edgeJ = vtkSmartPointer<vtkLine>::New();
		edgeJ->GetPointIds()->SetId(0, idT);
		edgeJ->GetPointIds()->SetId(1, idJ);
		vtkSmartPointer<vtkLine> edgeK = vtkSmartPointer<vtkLine>::New();
		edgeK->GetPointIds()->SetId(0, idT);
		edgeK->GetPointIds()->SetId(1, idK);

		//insert edges
		edges->InsertNextCell(edgeI);
		edges->InsertNextCell(edgeJ);
		edges->InsertNextCell(edgeK);

		//insert cell colors for edges
	#if VTK_MAJOR_VERSION < 7
		colors->InsertNextTupleValue(red);
		colors->InsertNextTupleValue(green);
		colors->InsertNextTupleValue(blue);
	#else
		colors->InsertNextTypedTuple(red);
		colors->InsertNextTypedTuple(green);
		colors->InsertNextTypedTuple(blue);
	#endif

	}


	//add face triangles
	for(int i=0; i<faces.size(); i++){
		vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
		triangle->GetPointIds()->SetId ( 0, faces[i].x );
		triangle->GetPointIds()->SetId ( 1, faces[i].y );
		triangle->GetPointIds()->SetId ( 2, faces[i].z );
		triangles->InsertNextCell(triangle);
		//insert cell colors for edges
	#if VTK_MAJOR_VERSION < 7
		colors->InsertNextTupleValue(yellow);
	#else
		colors->InsertNextTypedTuple(yellow);
	#endif
	}


	  pointsData = vtkPolyData::New();
	  pointsData->SetPoints(points);
	  pointsData->SetVerts(vertices);
	  pointsData->SetLines(edges);
	  pointsData->SetPolys(triangles);
	  pointsData->GetCellData()->SetScalars(colors);
	  //pointsData->GetPointData()->SetScalars(colors);


	  idFilter = vtkIdFilter::New();
	  idFilter->SetInput(pointsData);
	  idFilter->SetIdsArrayName("OriginalIds");
	  idFilter->Update();

	  surfaceFilter = vtkDataSetSurfaceFilter::New();
	  surfaceFilter->SetInputConnection(idFilter->GetOutputPort());
	  surfaceFilter->Update();

	  vtkPolyData* input = surfaceFilter->GetOutput();


	#if VTK_MAJOR_VERSION <= 5
	  mapper->SetInputConnection(pointsData->GetProducerPort());
	  //mapper->SetInput(pointsData); //alternative way to set mapper data
	#else
	  mapper->SetInputData(input);
	#endif

	  actor->GetProperty()->SetPointSize(POINT_SIZE);
	  actor->GetProperty()->SetLineWidth(LINE_WIDTH);
	  style->SetPoints(input); //note input != pointsdata
	  //style->SetPoints(pointsData);

	  if(resetView){
		  renderer->ResetCamera();	//move camera to cloud center
	  }
	  enableInteraction();

}

void CloudWidget::loadCloudAndCamera(const vector<Point3f> &xyzs, const vector<Point3f> &norms, const vector<Matx34d> &cameras, bool resetView){
	disableInteraction();

	//constants
	unsigned char red[3] = {255, 0, 0};
	unsigned char green[3] = {0, 255, 0};
	unsigned char blue[3] = {0, 0, 255};
	unsigned char white[3] = {255, 255, 255};
	unsigned char yellow[3] = {255, 255, 0};

	//insert points to vtk data structure
	// Create the geometry of a point (the coordinate)
	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	// Create the topology of the point (a vertex)
	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();
	// Create the topology of the line (an edge)
	vtkSmartPointer<vtkCellArray> edges = vtkSmartPointer<vtkCellArray>::New();
	// Create colors
	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New();
	colors->SetNumberOfComponents(3);
	colors->SetName ("Colors");

	//add cloud points
	numCloudPoints = xyzs.size();
	cout<<"numCloudPoints = "<<numCloudPoints<<endl;
	vtkIdType pid[numCloudPoints];//temp array to store point ids for create vertices
	for(int i=0; i<numCloudPoints; i++){
		pid[i] = points->InsertNextPoint(xyzs[i].x, xyzs[i].y, xyzs[i].z);
	}
	//create vertices from point ids
	vertices->InsertNextCell(numCloudPoints,pid);
	int numCells = vertices->GetNumberOfCells();
	for(int i=0; i<numCells; i++){
		vtkIdType numOfPoints;
		vtkIdType *pointIDs;
		vertices->GetCell(i, numOfPoints,pointIDs);
		assert(numOfPoints == numCloudPoints);
	}

	//inert cell color for the vertices
	#if VTK_MAJOR_VERSION < 7
		colors->InsertNextTupleValue(white);
	#else
		colors->InsertNextTypedTuple(white);
	#endif


	//add camera points & edges
	for(int n=0; n<cameras.size(); n++){
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

		//create & insert points
		vtkIdType idT    = points->InsertNextPoint(Tx, Ty, Tz);
		vtkIdType idI    = points->InsertNextPoint(Ix, Iy, Iz);
		vtkIdType idJ    = points->InsertNextPoint(Jx, Jy, Jz);
		vtkIdType idK    = points->InsertNextPoint(Kx, Ky, Kz);

		//create edges
		vtkSmartPointer<vtkLine> edgeI = vtkSmartPointer<vtkLine>::New();
		edgeI->GetPointIds()->SetId(0, idT);
		edgeI->GetPointIds()->SetId(1, idI);
		vtkSmartPointer<vtkLine> edgeJ = vtkSmartPointer<vtkLine>::New();
		edgeJ->GetPointIds()->SetId(0, idT);
		edgeJ->GetPointIds()->SetId(1, idJ);
		vtkSmartPointer<vtkLine> edgeK = vtkSmartPointer<vtkLine>::New();
		edgeK->GetPointIds()->SetId(0, idT);
		edgeK->GetPointIds()->SetId(1, idK);

		//insert edges
		edges->InsertNextCell(edgeI);
		edges->InsertNextCell(edgeJ);
		edges->InsertNextCell(edgeK);

		//insert cell colors for edges
	#if VTK_MAJOR_VERSION < 7
		colors->InsertNextTupleValue(red);
		colors->InsertNextTupleValue(green);
		colors->InsertNextTupleValue(blue);
	#else
		colors->InsertNextTypedTuple(red);
		colors->InsertNextTypedTuple(green);
		colors->InsertNextTypedTuple(blue);
	#endif

	}

	//add normals
	if(!norms.empty()){
		for(int i=0; i<numCloudPoints; i++){
			//create & insert points
			vtkIdType idfrm    	= points->InsertNextPoint(xyzs[i].x, xyzs[i].y, xyzs[i].z);
			vtkIdType idto    	= points->InsertNextPoint(xyzs[i].x+norms[i].x*0.1, xyzs[i].y+norms[i].y*0.1, xyzs[i].z+norms[i].z*0.1);
			//create edges
			vtkSmartPointer<vtkLine> normal = vtkSmartPointer<vtkLine>::New();
			normal->GetPointIds()->SetId(0, idfrm);
			normal->GetPointIds()->SetId(1, idto);
			//insert edges
			edges->InsertNextCell(normal);
			//insert cell colors for edges
		#if VTK_MAJOR_VERSION < 7
			colors->InsertNextTupleValue(yellow);
		#else
			colors->InsertNextTypedTuple(yellow);
		#endif
		}
	}

  pointsData = vtkPolyData::New();
  pointsData->SetPoints(points);
  pointsData->SetVerts(vertices);
  pointsData->SetLines(edges);
  pointsData->GetCellData()->SetScalars(colors);
  //pointsData->GetPointData()->SetScalars(colors);


  idFilter = vtkIdFilter::New();
  idFilter->SetInput(pointsData);
  idFilter->SetIdsArrayName("OriginalIds");
  idFilter->Update();

  surfaceFilter = vtkDataSetSurfaceFilter::New();
  surfaceFilter->SetInputConnection(idFilter->GetOutputPort());
  surfaceFilter->Update();

  vtkPolyData* input = surfaceFilter->GetOutput();


#if VTK_MAJOR_VERSION <= 5
  mapper->SetInputConnection(pointsData->GetProducerPort());
  //mapper->SetInput(pointsData); //alternative way to set mapper data
#else
  mapper->SetInputData(input);
#endif

  actor->GetProperty()->SetPointSize(POINT_SIZE);
  actor->GetProperty()->SetLineWidth(LINE_WIDTH);
  style->SetPoints(input); //note input != pointsdata
  //style->SetPoints(pointsData);

  if(resetView){
	  renderer->ResetCamera();	//move camera to cloud center
  }
  enableInteraction();
}
/*
void CloudWidget::loadCloud(const vector<Point3f> &xyzs){
	disableInteraction();
	//insert points to vtk data structure
	// Create the geometry of a point (the coordinate)
  	vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
	// Create the topology of the point (a vertex)
  	vtkSmartPointer<vtkCellArray> vertices = vtkSmartPointer<vtkCellArray>::New();

	vtkIdType pid[xyzs.size()];
	for(int i=0; i<xyzs.size(); i++){
		pid[i] = points->InsertNextPoint(xyzs[i].x, xyzs[i].y, xyzs[i].z);
	}
	
	vertices->InsertNextCell(xyzs.size(),pid);
  
  pointsData = vtkPolyData::New();
  pointsData->SetPoints(points);
  pointsData->SetVerts(vertices);

  idFilter = vtkIdFilter::New();
  idFilter->SetInput(pointsData);
  idFilter->SetIdsArrayName("OriginalIds");
  idFilter->Update();

  surfaceFilter = vtkDataSetSurfaceFilter::New();
  surfaceFilter->SetInputConnection(idFilter->GetOutputPort());
  surfaceFilter->Update();

  vtkPolyData* input = surfaceFilter->GetOutput();
  
#if VTK_MAJOR_VERSION <= 5
  mapper->SetInputConnection(input->GetProducerPort());
#else
  mapper->SetInputData(input);
#endif

  actor->GetProperty()->SetPointSize(POINT_SIZE);
  style->SetPoints(input);
  renderer->ResetCamera();	//move camera to cloud center
  enableInteraction();
}*/
