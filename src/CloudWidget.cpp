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
#include <vtkCellArray.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkPolyData.h>
#include <vtkPointSource.h>
#include <vtkInteractorStyleRubberBandPick.h>
#include <vtkAreaPicker.h>
#include <vtkExtractGeometry.h>
#include <vtkDataSetMapper.h>
#include <vtkUnstructuredGrid.h>
#include <vtkVertexGlyphFilter.h>
#include <vtkIdFilter.h>



#include "core/PlyIO.h"
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

      if(key == "d")
      {
        deleteSelectedPoints();
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

      //vtkIdTypeArray* ids = vtkIdTypeArray::SafeDownCast(selected->GetPointData()->GetArray("OriginalIds"));
      //for(vtkIdType i = 0; i < ids->GetNumberOfTuples(); i++)
        //{
        //std::cout << "Id " << i << " : " << ids->GetValue(i) << std::endl;
        //}
        
      this->SelectedActor->GetProperty()->SetColor(1.0, 0.0, 0.0); //(R,G,B)
      this->SelectedActor->GetProperty()->SetPointSize(5);

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
	void deleteSelectedPoints(){
		vtkIdTypeArray* ids = vtkIdTypeArray::SafeDownCast(SelectedMapper->GetInput()->GetPointData()->GetArray("OriginalIds"));
		QList<int> deleteIdxs;
		deleteIdxs.reserve(ids->GetNumberOfTuples());
		for(vtkIdType i = 0; i < ids->GetNumberOfTuples(); i++)
        {
        	//std::cout << "delete " << i << " : " << ids->GetValue(i) << std::endl;
			deleteIdxs.push_back(ids->GetValue(i));
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
  mapper->ScalarVisibilityOff();
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
}
void CloudWidget::deletePoints(const QList<int> idxs){

	disableInteraction(); //prevent further ui inputs untill refresh
	emit deletePointIdx(idxs);
}
void CloudWidget::disableInteraction(){
	style->setActive(false);
}
void CloudWidget::enableInteraction(){
	style->setActive(true);
}
void CloudWidget::loadCloud( QString fileName ){
	disableInteraction();
	vector<Point3f> xyzs;
	string fname = fileName.toStdString();
	PlyIO::readPLY(fname, xyzs);
	loadCloud(xyzs);
	enableInteraction();

}

void CloudWidget::loadCloud(std::vector<cv::Point3f> &xyzs){
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

  actor->GetProperty()->SetPointSize(5);
  style->SetPoints(input);
  renderer->ResetCamera();	//move camera to cloud center
  enableInteraction();
}
