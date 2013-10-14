#include "hubo_init.h"




namespace hubo_init_space
{


void HuboInitWidget::handleFTCopy()
{
    QString copyText;
    
    for(int i=0; i<4; i++)
    {
        if(i>0)
            QTextStream(&copyText) << "\t";
        QTextStream(&copyText) << QString::number(h_state.ft[i].m_x);
    }
    QTextStream(&copyText) << "\n";
    for(int i=0; i<4; i++)
    {
        if(i>0)
            QTextStream(&copyText) << "\t";
        QTextStream(&copyText) << QString::number(h_state.ft[i].m_y);
    }
    QTextStream(&copyText) << "\n";
    for(int i=0; i<4; i++)
    {
        if(i>0)
            QTextStream(&copyText) << "\t";
        QTextStream(&copyText) << QString::number(h_state.ft[i].f_z);
    }
    
    QClipboard* clipboard = QApplication::clipboard();
    clipboard->setText(copyText);
}

void HuboInitWidget::handleIMUCopy()
{
    QString copyText;

    QTextStream(&copyText) << QString::number(h_state.imu[2].a_x)
                   << "\t" << QString::number(h_state.imu[2].a_y)
                   << "\t" << QString::number(h_state.imu[2].a_z)
                   << "\n" << QString::number(h_state.imu[2].w_x)
                   << "\t" << QString::number(h_state.imu[2].w_y)
                   << "\t" << QString::number(h_state.imu[2].w_z);
    
    QClipboard* clipboard = QApplication::clipboard();
    clipboard->setText(copyText);
}


void HuboInitWidget::commandSensor()
{
    if( nullSensor->isChecked() )
        h_cmd.type = D_NULL_SENSOR;
    if( initSensor->isChecked() )
        h_cmd.type = D_INIT_FT_ACC_SENSOR;
    
    sendCommand();
}

void HuboInitWidget::handleRHFT()
{
    h_cmd.param[0] = D_R_HAND_FT;
    commandSensor();
}
void HuboInitWidget::handleLHFT()
{
    h_cmd.param[0] = D_L_HAND_FT;
    commandSensor();
}
void HuboInitWidget::handleRFFT()
{
    h_cmd.param[0] = D_R_FOOT_FT;
    commandSensor();
}
void HuboInitWidget::handleLFFT()
{
    h_cmd.param[0] = D_L_FOOT_FT;
    commandSensor();
}
void HuboInitWidget::handleIMU()
{
    h_cmd.param[0] = D_IMU_SENSOR_0;
    commandSensor();
    h_cmd.param[0] = D_IMU_SENSOR_1;
    commandSensor();
    h_cmd.param[0] = D_IMU_SENSOR_2;
    commandSensor();
}

void HuboInitWidget::initializeAchStructs()
{
    memset(&h_state, 0, sizeof(h_state));
    memset(&h_param, 0, sizeof(h_param));
    memset(&h_cmd, 0, sizeof(h_cmd));
    hubo_pwm_gains_t temp;
    setJointParams(&h_param, &h_state, &temp);
    setSensorDefaults(&h_param);

    for(int i=0; i<HUBO_JOINT_COUNT; i++)
        if( strcmp(h_param.joint[i].name, "") == 0 )
            sprintf( h_param.joint[i].name, "N/A" );
}

void HuboInitWidget::initializeAchConnections()
{
    stateConnected = false;
    cmdConnected = false;
    stateOpen = false;
    cmdOpen = false;


    achChannelState.start("ach mk " + QString::fromLocal8Bit(HUBO_CHAN_STATE_NAME)
                          + " -1 -m 10 -n 8000 -o 666", QIODevice::ReadWrite);
    connect(&achChannelState, SIGNAL(error(QProcess::ProcessError)),
            this, SLOT(achCreateCatch(QProcess::ProcessError)));
    connect(&achChannelState, SIGNAL(started()), this, SLOT(achCreateSHandle()));
    
    achChannelCmd.start("ach -C " + QString::fromLocal8Bit(HUBO_CHAN_BOARD_CMD_NAME)
                        + " -1 -m 10 -n 3000 -o 666", QIODevice::ReadWrite);
    connect(&achChannelCmd, SIGNAL(error(QProcess::ProcessError)),
            this, SLOT(achCreateCatch(QProcess::ProcessError)));
    connect(&achChannelCmd, SIGNAL(started()), this, SLOT(achCreateCHandle()));
    
    connect(&achdState, SIGNAL(started()), this, SLOT(achdSStartedSlot()));
    connect(&achdCmd, SIGNAL(started()), this, SLOT(achdCStartedSlot()));

    
    connect(&achdState, SIGNAL(error(QProcess::ProcessError)), this, SLOT(achdSExitError(QProcess::ProcessError)));
    connect(&achdCmd, SIGNAL(error(QProcess::ProcessError)), this, SLOT(achdCExitError(QProcess::ProcessError)));
    
    connect(&achdState, SIGNAL(finished(int,QProcess::ExitStatus)),
            this, SLOT(achdSExitFinished(int,QProcess::ExitStatus)));
    connect(&achdCmd, SIGNAL(finished(int,QProcess::ExitStatus)),
            this, SLOT(achdCExitFinished(int,QProcess::ExitStatus)));

}

void HuboInitWidget::achCreateSHandle()
{
    ach_status_t r = ach_open(&stateChan, HUBO_CHAN_STATE_NAME, NULL);
    if( r == ACH_OK )
        stateOpen = true;
    else
    {
        ROS_INFO("State Channel failed to open");
        fprintf(stderr, "State Channel failed to open: %s", ach_result_to_string(r));
    }
}

void HuboInitWidget::achCreateCHandle()
{
    ach_status_t r = ach_open(&cmdChan, HUBO_CHAN_BOARD_CMD_NAME, NULL);
    if( r == ACH_OK )
        cmdOpen = true;
    else
    {
        ROS_INFO("Command Channel failed to open");
        fprintf(stderr, "Command Channel failed to open: %s", ach_result_to_string(r));
    }
}

void HuboInitWidget::achdConnectSlot()
{
    achdState.start("achd pull " + QString::number(ipAddrA)
                                 + "." + QString::number(ipAddrB)
                                 + "." + QString::number(ipAddrC)
                                 + "." + QString::number(ipAddrD)
                    + " " + QString::fromLocal8Bit(HUBO_CHAN_STATE_NAME));
    achdCmd.start("achd push " + QString::number(ipAddrA)
                               + "." + QString::number(ipAddrB)
                               + "." + QString::number(ipAddrC)
                               + "." + QString::number(ipAddrD)
                     + " " + QString::fromLocal8Bit(HUBO_CHAN_BOARD_CMD_NAME));
}

void HuboInitWidget::achdDisconnectSlot()
{
    achdState.kill();
    achdCmd.kill();
}


} // End: namespace hubo_init_space
