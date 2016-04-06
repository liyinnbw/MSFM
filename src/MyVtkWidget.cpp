#include <QtGui>
#include "MyVtkWidget.h"
#include "vtkPolyDataAlgorithm.h"
#include "vtkPolyDataMapper.h"
#include "vtkActor.h"
#include "vtkRenderer.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkInteractorStyleTrackballCamera.h"



// Constructor
MyVtkWidget:: MyVtkWidget(QWidget *parent)
				 : QVTKWidget(parent)
{
	vtkPolyDataMapper *mapper = vtkPolyDataMapper::New();
    vtkActor *actor = vtkActor::New();
    actor->SetMapper(mapper);

    // Create renderer
    vtkRenderer *renderer = vtkRenderer::New();
    renderer->AddActor(actor);
    renderer->SetBackground(0.0, 0.0, 0.0);

    // QVTKWidget has already created a render window and interactor
    vtkRenderWindow *renderWindow = GetRenderWindow();
    renderWindow->AddRenderer(renderer);
    vtkRenderWindowInteractor *interactor = renderWindow->GetInteractor();
    
    vtkInteractorStyleTrackballCamera *style = vtkInteractorStyleTrackballCamera::New();
    interactor->SetInteractorStyle(style);
}

void MyVtkWidget::loadCloud(){
	
}
