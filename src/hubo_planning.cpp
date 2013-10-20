#include "hubo_planning.h"
#include "FlowLayout.h"

namespace hubo_planning_common
{



HuboPlanningPanel::HuboPlanningPanel(QWidget *parent)
    : rviz::Panel(parent)
{
    content = new HuboPlanningWidget;
    QHBoxLayout* panelLayout = new QHBoxLayout;
    panelLayout->addWidget(content);
    setLayout(panelLayout);
}

HuboPlanningWidget::~HuboPlanningWidget(){

}

HuboPlanningWidget::HuboPlanningWidget(QWidget *parent)
    : QTabWidget(parent)
{

    initializePlanerTab();
    std::cerr << "Planer Control Tab loaded" << std::endl;

    initializePlanerPoseTab();
    std::cerr << "Planer, Saved Poses Tab loaded" << std::endl;

    addTab(planerCmdTab, "Planer");
    addTab(PlanerPoseTab, "Saved Poses");
}    

//_______________________________________________________________________________________________________
void HuboPlanningWidget::initializePlanerTab()
{
    QVBoxLayout* planerLayout = new QVBoxLayout;

	QGroupBox* planerBox = new QGroupBox;
    planerBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    planerBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    planerBox->setTitle("Planer Interface");
    planerBox->setStyleSheet(groupStyleSheet);

    	QGridLayout* planerOptionsLayout = new QGridLayout;
    planerOptionsLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    
	StartButton = new QPushButton;
    StartButton->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    StartButton->setText("  Start Planer  ");
    StartButton->setToolTip("Start OpenRave/OpenHubo Planer");
    planerOptionsLayout->addWidget(StartButton, 0, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
    connect( StartButton, SIGNAL(clicked()), this, SLOT(handleStartPlaner()) );
    
	PlanButton = new QPushButton;
    PlanButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    PlanButton->setText("Goal Position");
    PlanButton->setToolTip("Plan Trajectory to goal position");
    planerOptionsLayout->addWidget(PlanButton, 1, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
    connect( PlanButton, SIGNAL(clicked()), this, SLOT(handlePlanToGoal()) );

    	CloseButton = new QPushButton;
    CloseButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    CloseButton->setText("Grasp");
    CloseButton->setToolTip("Close Hand");
    planerOptionsLayout->addWidget(CloseButton, 1, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
    connect( CloseButton, SIGNAL(clicked()), this, SLOT(handleCloseHand()) );

    	TurnButton = new QPushButton;
    TurnButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    TurnButton->setText("Turn Hand");
    TurnButton->setToolTip("Turns Hand to Screw in Hose");
    planerOptionsLayout->addWidget(TurnButton, 1, 2, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
    connect( TurnButton, SIGNAL(clicked()), this, SLOT(handleTurnHand()) );

    	GoBackButton = new QPushButton;
    GoBackButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    GoBackButton->setText("     Go Back     ");
    GoBackButton->setToolTip("Execute Trajectory backwars");
    planerOptionsLayout->addWidget(GoBackButton, 2, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
    connect( GoBackButton, SIGNAL(clicked()), this, SLOT(handleGoBack()) );

    planerBox->setLayout(planerOptionsLayout);
    planerLayout->addWidget(planerBox, Qt::AlignHCenter | Qt::AlignTop); 

    planerCmdTab = new QWidget;
    planerCmdTab->setLayout(planerLayout);
}
//______________________________________________________________________________________________________

void HuboPlanningWidget::initializePlanerPoseTab()
{  
    QVBoxLayout* PlanerPoseLayout = new QVBoxLayout;

    PlanerPoseTab = new QWidget;
    PlanerPoseTab->setLayout(PlanerPoseLayout);  

}
//______________________________________________________________________________________________________


void HuboPlanningPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    config.mapSetValue("Class", getClassId());

    rviz::Config ip_config = config.mapMakeChild("HuboIP");
}

void HuboPlanningPanel::load(const rviz::Config &config)
{
    rviz::Panel::load(config);
}


} // end namespace




#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( hubo_planning_common::HuboPlanningPanel,rviz::Panel )
PLUGINLIB_EXPORT_CLASS( hubo_planning_common::HuboPlanningWidget, QTabWidget )
