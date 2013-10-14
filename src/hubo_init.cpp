#include "hubo_init.h"
#include "FlowLayout.h"

namespace hubo_init_space
{



HuboInitPanel::HuboInitPanel(QWidget *parent)
    : rviz::Panel(parent)
{
    content = new HuboInitWidget;
    QHBoxLayout* panelLayout = new QHBoxLayout;
    panelLayout->addWidget(content);
    setLayout(panelLayout);
}

HuboInitWidget::~HuboInitWidget(){

}

HuboInitWidget::HuboInitWidget(QWidget *parent)
    : QTabWidget(parent)
{

    QProcess copyTable;
    copyTable.start("sudo cp -f /etc/hubo-ach/drc-hubo.joint.table /etc/hubo-ach/joint.table");
    copyTable.waitForFinished();

    groupStyleSheet = "QGroupBox {"
                      "border: 1px solid gray;"
                      "border-radius: 9px;"
                      "margin-top: 0.5em;"
                      "}"
                      "QGroupBox::title {"
                      "subcontrol-origin: margin;"
                      "left: 10px;"
                      "padding: 0 3px 0 3px;"
                      "}";


    initializeAchStructs();

    initializeSensorCmdTab();
    std::cerr << "Sensor Command Tab loaded" << std::endl;

    initializeSensorStateTab();
    std::cerr << "Sensor State Tab loaded" << std::endl;

    addTab(sensorCmdTab, "Sensor Command");
    addTab(sensorStateTab, "Sensor State");


    initializeAchConnections();
}    

void HuboInitWidget::achCreateCatch(QProcess::ProcessError err)
{
    ROS_INFO("Creating Ach Channel Failed: Error Code %d", (int)err);
}




void HuboInitWidget::initializeSensorCmdTab()
{
    radioSensorButtons = new QButtonGroup;
    radioSensorButtons->setExclusive(true);

    QHBoxLayout* radioSensorLayout = new QHBoxLayout;
    nullSensor = new QRadioButton;
    nullSensor->setText("Null Sensor");
    nullSensor->setToolTip("Set sensor values to zero. Must run this before data can be received.");
    radioSensorButtons->addButton(nullSensor);
    radioSensorLayout->addWidget(nullSensor);
    nullSensor->setChecked(true);
    initSensor = new QRadioButton;
    initSensor->setText("Initialize");
    initSensor->setToolTip("Reset sensor board values to default settings.\n"
                           "WARNING: This overwrites the current board settings.\n"
                           "(Currently Disabled)");
    initSensor->setCheckable(false);
    initSensor->setDisabled(true);
    radioSensorButtons->addButton(initSensor);
    radioSensorLayout->addWidget(initSensor);
    // Note: Leaving out initSensor because I think it's a dangerous feature

    QHBoxLayout* sensorLayout1 = new QHBoxLayout;
    lhFTButton = new QPushButton;
    lhFTButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    lhFTButton->setText("Left Hand FT");
    lhFTButton->setToolTip("Left hand force torque sensor");
    sensorLayout1->addWidget(lhFTButton);
    connect( lhFTButton, SIGNAL(clicked()), this, SLOT(handleLHFT()) );
    rhFTButton = new QPushButton;
    rhFTButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    rhFTButton->setText("Right Hand FT");
    rhFTButton->setToolTip("Right hand force torque sensor");
    sensorLayout1->addWidget(rhFTButton);
    connect( rhFTButton, SIGNAL(clicked()), this, SLOT(handleRHFT()) );

    imuButton = new QPushButton;
    imuButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    imuButton->setText("IMU");
    imuButton->setToolTip("Inertial Measurement Unit (waist)");
    connect( imuButton, SIGNAL(clicked()), this, SLOT(handleIMU()) );

    QHBoxLayout* sensorLayout3 = new QHBoxLayout;
    lfFTButton = new QPushButton;
    lfFTButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    lfFTButton->setText("Left Foot FT");
    lfFTButton->setToolTip("Left foot force torque sensor");
    sensorLayout3->addWidget(lfFTButton);
    connect( lfFTButton, SIGNAL(clicked()), this, SLOT(handleLFFT()) );
    rfFTButton = new QPushButton;
    rfFTButton->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    rfFTButton->setText("Right Foot FT");
    rfFTButton->setToolTip("Right foot force torque sensor");
    sensorLayout3->addWidget(rfFTButton);
    connect( rfFTButton, SIGNAL(clicked()), this, SLOT(handleRFFT()) );


    QVBoxLayout* masterSCTLayout = new QVBoxLayout;
    masterSCTLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    masterSCTLayout->addLayout(radioSensorLayout);
    masterSCTLayout->addLayout(sensorLayout1);
    masterSCTLayout->addWidget(imuButton, 0, Qt::AlignCenter);
    masterSCTLayout->addLayout(sensorLayout3);

    sensorCmdTab = new QWidget;
    sensorCmdTab->setLayout(masterSCTLayout);
}


void HuboInitWidget::initializeSensorStateTab()
{
    QVBoxLayout* sensorStateLayout = new QVBoxLayout;

    QGroupBox* ftBox = new QGroupBox;
    ftBox->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Minimum);
    ftBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    ftBox->setTitle("Force-Torque Readings");
    ftBox->setStyleSheet(groupStyleSheet);//HuboRefreshManager

    QGridLayout* ftStateLayout = new QGridLayout;
    ftStateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);

    QLabel* rhftLab = new QLabel;
    rhftLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    rhftLab->setText("Right Hand");
    ftStateLayout->addWidget(rhftLab, 0, 1, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* lhftLab = new QLabel;
    lhftLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    lhftLab->setText("Left Hand");
    ftStateLayout->addWidget(lhftLab, 0, 2, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* rfftLab = new QLabel;
    rfftLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    rfftLab->setText("Right Foot");
    ftStateLayout->addWidget(rfftLab, 0, 3, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* lfftLab = new QLabel;
    lfftLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    lfftLab->setText("Left Foot");
    ftStateLayout->addWidget(lfftLab, 0, 4, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* mxLab = new QLabel;
    mxLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    mxLab->setText("X Moment");
    mxLab->setToolTip("Moment about the X-Axis (N-m)");
    ftStateLayout->addWidget(mxLab, 1, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* myLab = new QLabel;
    myLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    myLab->setText("Y Moment");
    myLab->setToolTip("Moment about the Y-Axis (N-m)");
    ftStateLayout->addWidget(myLab, 2, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    QLabel* fzLab = new QLabel;
    myLab->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    fzLab->setText("Z Force");
    fzLab->setToolTip("Force along the Z-Axis (N)");
    ftStateLayout->addWidget(fzLab, 3, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);

    ft_mx.resize(4);
    ft_my.resize(4);
    ft_fz.resize(4);
    for(int i=0; i<4; i++)
    {
        QSizePolicy mxPolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
        
        QLineEdit* tempMxEdit = new QLineEdit;
        tempMxEdit->setSizePolicy(mxPolicy);
        tempMxEdit->setReadOnly(true);
        //tempMxEdit->setMinimumWidth(1);
        ftStateLayout->addWidget(tempMxEdit, 1, i+1, 1, 1);
        ft_mx[i] = tempMxEdit;

        QLineEdit* tempMyEdit = new QLineEdit;
        tempMyEdit->setSizePolicy(mxPolicy);
        tempMyEdit->setReadOnly(true);
        //tempMyEdit->setMinimumWidth(1);
        ftStateLayout->addWidget(tempMyEdit, 2, i+1, 1, 1);
        ft_my[i] = tempMyEdit;

        QLineEdit* tempFzEdit = new QLineEdit;
        tempFzEdit->setSizePolicy(mxPolicy);
        tempFzEdit->setReadOnly(true);
        //tempFzEdit->setMinimumWidth(1);
        ftStateLayout->addWidget(tempFzEdit, 3, i+1, 1, 1);
        ft_fz[i] = tempFzEdit;
    }

    copyFT = new QPushButton;
    copyFT->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    copyFT->setText("Copy Data");
    copyFT->setToolTip("Copy the Force-Torque data to clipboard (tab and newline separated)");
    ftStateLayout->addWidget(copyFT, 4, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
    connect(copyFT, SIGNAL(clicked()), this, SLOT(handleFTCopy()));
    
    ftBox->setLayout(ftStateLayout);
    sensorStateLayout->addWidget(ftBox, 0, Qt::AlignHCenter | Qt::AlignTop);


    imuBox = new QGroupBox;
    imuBox->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    imuBox->setAlignment(Qt::AlignHCenter | Qt::AlignTop);
    imuBox->setStyleSheet(groupStyleSheet);
    imuBox->setTitle("IMU Sensor Readings");

    QGridLayout* imuStateLayout = new QGridLayout;
    imuStateLayout->setAlignment(Qt::AlignHCenter | Qt::AlignTop);

    QLabel* tiltx = new QLabel;
    tiltx->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    tiltx->setText("Tilt X");
    tiltx->setToolTip("Angle of tilt about the X-Axis (deg)");
    imuStateLayout->addWidget(tiltx, 0, 0, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    a_x = new QLineEdit;
    a_x->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    a_x->setReadOnly(true);
    imuStateLayout->addWidget(a_x, 1, 0, 1, 1, Qt::AlignCenter);

    QLabel* tilty = new QLabel;
    tilty->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    tilty->setText("Tilt Y");
    tilty->setToolTip("Angle of tilt about the Y-Axis (deg)");
    imuStateLayout->addWidget(tilty, 0, 1, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    a_y = new QLineEdit;
    a_y->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    a_y->setReadOnly(true);
    imuStateLayout->addWidget(a_y, 1, 1, 1, 1, Qt::AlignCenter);

    QLabel* tiltz = new QLabel;
    tiltz->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    tiltz->setText("Tilt Z");
    tiltz->setToolTip("Angle of tilt about the Z-Axis (deg)");
    imuStateLayout->addWidget(tiltz, 0, 2, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    a_z = new QLineEdit;
    a_z->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    a_z->setReadOnly(true);
    imuStateLayout->addWidget(a_z, 1, 2, 1, 1, Qt::AlignCenter);


    QLabel* velx = new QLabel;
    velx->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    velx->setText("Angular Vel X");
    velx->setToolTip("Angular Velocity about the X-Axis");
    imuStateLayout->addWidget(velx, 3, 0, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    w_x = new QLineEdit;
    w_x->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    w_x->setReadOnly(true);
    imuStateLayout->addWidget(w_x, 4, 0, 1, 1, Qt::AlignCenter);

    QLabel* vely = new QLabel;
    vely->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    vely->setText("Angular Vel Y");
    vely->setToolTip("Angular Velocity about the Y-Axis");
    imuStateLayout->addWidget(vely, 3, 1, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    w_y = new QLineEdit;
    w_y->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    w_y->setReadOnly(true);
    imuStateLayout->addWidget(w_y, 4, 1, 1, 1, Qt::AlignCenter);

    QLabel* velz = new QLabel;
    velz->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    velz->setText("Angular Vel Z");
    velz->setToolTip("Angular Velocity about the Z-Axis");
    imuStateLayout->addWidget(velz, 3, 2, 1, 1, Qt::AlignLeft | Qt::AlignBottom );

    w_z = new QLineEdit;
    w_z->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    w_z->setReadOnly(true);
    imuStateLayout->addWidget(w_z, 4, 2, 1, 1);
    
    copyIMU = new QPushButton;
    copyIMU->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
    copyIMU->setText("Copy Data");
    copyIMU->setToolTip("Copy the IMU data to clipboard (tab and newline separated)");
    imuStateLayout->addWidget(copyIMU, 5, 0, 1, 1, Qt::AlignHCenter | Qt::AlignVCenter);
    connect(copyIMU, SIGNAL(clicked()), this, SLOT(handleIMUCopy()));

    imuBox->setLayout(imuStateLayout);
    sensorStateLayout->addWidget(imuBox, Qt::AlignHCenter | Qt::AlignTop);

    sensorStateTab = new QWidget;
    sensorStateTab->setLayout(sensorStateLayout);
}

void HuboInitPanel::save(rviz::Config config) const
{
    rviz::Panel::save(config);
    config.mapSetValue("Class", getClassId());

    rviz::Config ip_config = config.mapMakeChild("HuboIP");

    QVariant a = QVariant(content->getIPAddress(0));
    QVariant b = QVariant(content->getIPAddress(1));
    QVariant c = QVariant(content->getIPAddress(2));
    QVariant d = QVariant(content->getIPAddress(3));

    ip_config.mapSetValue("ipAddrA", a);
    ip_config.mapSetValue("ipAddrB", b);
    ip_config.mapSetValue("ipAddrC", c);
    ip_config.mapSetValue("ipAddrD", d);

}

void HuboInitPanel::load(const rviz::Config &config)
{
    rviz::Panel::load(config);
    rviz::Config ip_config = config.mapGetChild("HuboIP");
    QVariant a, b, c, d;
    if( !ip_config.mapGetValue("ipAddrA", &a) || !ip_config.mapGetValue("ipAddrB", &b)
     || !ip_config.mapGetValue("ipAddrC", &c) || !ip_config.mapGetValue("ipAddrD", &d))
        ROS_INFO("Loading the IP Address Failed");
    else
        content->setIPAddress(a.toInt(), b.toInt(), c.toInt(), d.toInt());
}


} // End hubo_init_space




#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( hubo_init_space::HuboInitPanel,rviz::Panel )
PLUGINLIB_EXPORT_CLASS( hubo_init_space::HuboInitWidget, QTabWidget )
