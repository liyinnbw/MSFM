/*
 *  Interactive widget to display reconstructed scene
 */


#include <QtWidgets>

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

#include <Eigen/Eigen>

#include "core/Utils.h"
#include "CloudWidget.h"

using namespace std;
using namespace cv;
using namespace Eigen;

//constants
const static unsigned char red[3] = {255, 0, 0};
const static unsigned char green[3] = {0, 255, 0};
const static unsigned char blue[3] = {0, 0, 255};
const static unsigned char white[3] = {255, 255, 255};
const static unsigned char yellow[3] = {255, 255, 0};

// Define interaction style
class InteractorStyle : public vtkInteractorStyleRubberBandPick
{
  public:
    static InteractorStyle* New();
    vtkTypeMacro(InteractorStyle,vtkInteractorStyleRubberBandPick);

    InteractorStyle()
    :parentWidget(NULL)
    ,isActive(false)
    {
      this->SelectedMapper = vtkSmartPointer<vtkDataSetMapper>::New();
      this->SelectedActor = vtkSmartPointer<vtkActor>::New();
      this->SelectedActor->SetMapper(SelectedMapper);
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
    	  showSelectedPointsMeasurements();
      }else if (key == "p"){
		  parentWidget->showAllProjections();
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
// #if VTK_MAJOR_VERSION <= 5
//       extractGeometry->SetInput(this->Points);
// #else
      extractGeometry->SetInputData(this->Points);
// #endif
      extractGeometry->Update();

      vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
        vtkSmartPointer<vtkVertexGlyphFilter>::New();
      glyphFilter->SetInputConnection(extractGeometry->GetOutputPort());
      glyphFilter->Update();

      vtkPolyData* selected = glyphFilter->GetOutput();
      //std::cout << "Selected " << selected->GetNumberOfPoints() << " points." << std::endl;
      //std::cout << "Selected " << selected->GetNumberOfCells() << " cells." << std::endl;
// #if VTK_MAJOR_VERSION <= 5
//       this->SelectedMapper->SetInput(selected);
// #else
      this->SelectedMapper->SetInputData(selected);
// #endif
      this->SelectedMapper->ScalarVisibilityOff();

      //TODO: comment out the following debug
      vtkIdTypeArray* ids = vtkIdTypeArray::SafeDownCast(selected->GetPointData()->GetArray("OriginalIds"));
      for(vtkIdType i = 0; i < ids->GetNumberOfTuples(); i++)
      {
    	int originalID = ids->GetValue(i);
		if(	originalID>=parentWidget->numCloudPoints){
			int camIdx = (originalID-parentWidget->numCloudPoints)/4;
			std::cout << "selected camera " << camIdx << std::endl;
			//std::cout << "point " << i << " : " << originalID<< std::endl;
		}else{
			//std::cout << "point " << i << " : " << originalID<< std::endl;
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
	void showSelectedPointsMeasurements(){
		vtkIdTypeArray* ids = vtkIdTypeArray::SafeDownCast(SelectedMapper->GetInput()->GetPointData()->GetArray("OriginalIds"));
		QList<int> selectedIdxs;
		selectedIdxs.reserve(ids->GetNumberOfTuples());

		for(vtkIdType i = 0; i < ids->GetNumberOfTuples(); i++)
		{
			int originalID = ids->GetValue(i);
			if(	originalID<parentWidget->numCloudPoints){
			//std::cout << "delete " << i << " : " << ids->GetValue(i) << std::endl;
				selectedIdxs.push_back(ids->GetValue(i));
			}else{
				//std::cout << "point " << i << " : " << originalID<<" is a camera point" << std::endl;
			}
		}
		parentWidget->showPointsMeasurements(selectedIdxs);
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
// #if VTK_MAJOR_VERSION <= 5
//       this->SelectedMapper->SetInput(selected);
// #else
      this->SelectedMapper->SetInputData(selected);
// #endif
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
  mapper->ScalarVisibilityOff();
//   mapper->ScalarVisibilityOn(); //default on, if off, scalar color will not be applied, all geometry will be white
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
	emit requestDeletePoints(idxs);
}
void CloudWidget::showPointsMeasurements(const QList<int> idxs){
	disableInteraction();
	emit requestShowPointsMeasures(idxs);
}

void CloudWidget::showAllProjections(){
	//disableInteraction();
	emit requestShowAllProjections();
}

void CloudWidget::disableInteraction(){
	style->setActive(false);
}
void CloudWidget::enableInteraction(){
	style->setActive(true);
}

//visualize cloud and camera
void CloudWidget::visualize(bool resetView){

	disableInteraction();
	ptIdx2lm.clear();
	camIdx2f.clear();
	lm2ptIdx.clear();
	f2camIdx.clear();

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


	//===================================================
	//add points
	//===================================================
	Data &data = Data::GetInstance();
	const vector<LandMark::Ptr> &lms = data.getLandMarks();
	numCloudPoints = lms.size();

	vtkIdType pid[numCloudPoints];//temp array to store point ids for create vertices
	for(int i=0; i<numCloudPoints; i++){
		Vector3d &pt = lms[i]->pt;
		pid[i] = points->InsertNextPoint((double)pt[0], (double)pt[1], (double)pt[2]);

		//add to look up tables
		ptIdx2lm[pid[i]] = lms[i];
		lm2ptIdx[lms[i]] = pid[i];
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


	//===================================================
	//add cameras
	//===================================================
	const vector<Frame::Ptr> &kfs 	= data.getFrames();
	const vector<Frame::Ptr> &nkfs	= data.nonKeyFrames;
	int numFrames = kfs.size()+nkfs.size();

	for(int n=0; n<numFrames; n++){

		Frame::Ptr f;
		if(n<kfs.size()){
			f = kfs[n];
		}else{
			f = nkfs[n-kfs.size()];
		}

		/*
		//opencv
		Matx34d transform = fs[n]->getCVTransform();

		Matrix3d R;
		Vector3d T;
		for(int r=0; r<3; r++){
			for(int c=0; c<3; c++){
				R(r,c) = transform(r,c);
			}
			T(r) = transform(r,3);
		}

		Vector3d origin = R.transpose()*(-T);
		Vector3d axisX	= R.transpose()*(Vector3d(1,0,0)-T);
		Vector3d axisY	= R.transpose()*(Vector3d(0,1,0)-T);
		Vector3d axisZ	= R.transpose()*(Vector3d(0,0,1)-T);*/

		//opengv
		Vector3d origin = f->position;
		Quaterniond qc	= f->rotation.conjugate();
		Vector3d axisX	= qc*Vector3d(1,0,0)+origin;
		Vector3d axisY	= qc*Vector3d(0,1,0)+origin;
		Vector3d axisZ	= qc*Vector3d(0,0,1)+origin;

		vtkIdType idT   = points->InsertNextPoint((double)origin(0),(double)origin(1), (double)origin(2));
		vtkIdType idI   = points->InsertNextPoint((double)axisX(0),(double)axisX(1), (double)axisX(2));
		vtkIdType idJ   = points->InsertNextPoint((double)axisY(0),(double)axisY(1), (double)axisY(2));
		vtkIdType idK   = points->InsertNextPoint((double)axisZ(0),(double)axisZ(1), (double)axisZ(2));


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

		//add to look up tables
		camIdx2f[idT] = f;
		f2camIdx[f] = idT;
	}



  pointsData = vtkPolyData::New();
  pointsData->SetPoints(points);
  pointsData->SetVerts(vertices);
  pointsData->SetLines(edges);
  pointsData->GetCellData()->SetScalars(colors);
  //pointsData->GetPointData()->SetScalars(colors);


  idFilter = vtkIdFilter::New();
  idFilter->SetInputData(pointsData);
  idFilter->SetIdsArrayName("OriginalIds");
  idFilter->Update();

  surfaceFilter = vtkDataSetSurfaceFilter::New();
  surfaceFilter->SetInputConnection(idFilter->GetOutputPort());
  surfaceFilter->Update();

  vtkPolyData* input = surfaceFilter->GetOutput();


// #if VTK_MAJOR_VERSION <= 5
//   mapper->SetInputConnection(pointsData->GetProducerPort());
//   //mapper->SetInput(pointsData); //alternative way to set mapper data
// #else
  mapper->SetInputData(input);
// #endif

  actor->GetProperty()->SetPointSize(POINT_SIZE);
  actor->GetProperty()->SetLineWidth(LINE_WIDTH);
  style->SetPoints(input); //note input != pointsdata
  //style->SetPoints(pointsData);

  if(resetView){
	  renderer->ResetCamera();	//move camera to cloud center
  }


  if(points->GetNumberOfPoints()>0 ){
	  //cout<<"interaction enabled"<<endl;
	  enableInteraction();
  }
  update();

  cout<<"[visualize] numFrames = "<<numFrames<<" numLandmarks = "<<numCloudPoints<<" numMeasures = "<<data.countMeasurements()<<endl;
}

//visualize measurements
void CloudWidget::visualizeMeasurements(vector<Measurement::Ptr> &ms){

	//get edges
	vtkSmartPointer<vtkCellArray> edges = pointsData->GetLines();

	//get colors
	vtkSmartPointer<vtkUnsignedCharArray> colors = vtkUnsignedCharArray::SafeDownCast(pointsData->GetCellData()->GetScalars("Colors"));


	unsigned char highlightColor[3] = {(unsigned char)Utils::getRandomInt(0,255), (unsigned char)Utils::getRandomInt(0,255), (unsigned char)Utils::getRandomInt(0,255)};	//random color

	//add measures
	for(vector<Measurement::Ptr>::iterator it = ms.begin(); it!=ms.end(); ++it){
		Frame::Ptr 		&f 	= (*it)->frame;
		LandMark::Ptr	&p 	= (*it)->landmark;
		int	camIdx			= f2camIdx[f];
		int ptIdx			= lm2ptIdx[p];

		vtkSmartPointer<vtkLine> edge = vtkSmartPointer<vtkLine>::New();
		edge->GetPointIds()->SetId(0, camIdx);
		edge->GetPointIds()->SetId(1, ptIdx);
			edges->InsertNextCell(edge);
		#if VTK_MAJOR_VERSION < 7
			colors->InsertNextTupleValue(highlightColor);
		#else
			colors->InsertNextTypedTuple(highlightColor);
		#endif
	}

	pointsData->Modified(); //to trigger an UI update
	update();
}

