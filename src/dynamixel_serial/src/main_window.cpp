/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/dynamixel_serial/main_window.hpp"
#include "../include/dynamixel_serial/qnode.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dynamixel_serial {


unsigned char txdata[128] = {1,2,3,4,5,6,7,8, };

QByteArray Tx_data;

bool Tx_motion_flag = false;
unsigned char TxBuffer[128] = { 0, };
unsigned char RxBuffer[10] = { 0, };
unsigned char Parameter[128] = { 0, };

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode(argc,argv)
{
    ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.

    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));
    QObject::connect(&qnode, SIGNAL(Rx_MotorData()), this, SLOT(DynamixelCallback()));
    QObject::connect(&qnode, SIGNAL(Rx_MotionData()), this, SLOT(MotionCallback()));
    QObject::connect(&qnode, SIGNAL(Rx_Dxl_id()), this, SLOT(Return_persent_position()));
    QObject::connect(&qnode, SIGNAL(wheel_mode()), this, SLOT(wheel_Mode_callback()));

    qnode.init();
    DynamixelInit();
}

MainWindow::~MainWindow() {}

void MainWindow::DynamixelInit()
{

    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux
    QString device_nameS =  "/dev/" + ui.comboBox_deviceName->currentText();
    char *device_nameCh = device_nameS.toLocal8Bit().data();
    cout<<"device_name: "<<device_nameCh<<endl;
    portHandler_   = dynamixel::PortHandler::getPortHandler(device_nameCh);

    // Initialize packetHandler_ instance
    // Set the protocol version
    // Get methods and members of Protocol1packetHandler_ or Protocol2packetHandler_
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    //    groupSyncWrite(portHandler_, packetHandler_, ADDR_MX_GOAL_POSITION, LEN_MX_GOAL_POSITION);

    // Open port
    if( portHandler_->openPort() )
    {
        ROS_INFO("Succeeded to open the port!");
    }
    else
    {
        ROS_ERROR("Failed to open the port!");
    }

    // Set port baudrate
    if( portHandler_->setBaudRate(ui.comboBox_baudrate->currentText().toInt()) ) // BAUDRATE
    {
        ROS_INFO("Succeeded to change the baudrate!");
    }
    else
    {
        ROS_ERROR("Failed to change the baudrate!");
    }
}


void MainWindow::DynamixelCallback()
{
    cout<<"#### DynamixelCallback ####"<<endl<<endl;

    if(isMotion == false)
    {
        switch(qnode.Dx_msg.mode)
        {
        case 1: //position
        {

            uint8_t param_goal_position[2];
            bool dxl_addparam_result = false;                // addParam result

            dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, ADDR_MX_GOAL_POSITION, 4);

            // Add Dynamixel#1 ~22 goal position value to the Syncwrite storage
            for(int i = 0; i < qnode.Dx_msg.id.size(); i++)
            {
                param_goal_position[0] = DXL_LOBYTE(qnode.Dx_msg.position[i]);
                param_goal_position[1] = DXL_HIBYTE(qnode.Dx_msg.position[i]);
                dxl_addparam_result = groupSyncWrite.addParam(qnode.Dx_msg.id[i], param_goal_position);
                if (dxl_addparam_result != true)
                {
                    ROS_ERROR("[ID] %u, roupSyncWrite addparam failed",i);
                    break;
                }
            }
            groupSyncWrite.txPacket();
            groupSyncWrite.clearParam();
        }

            break;

        case 2: //speed (not used)
        {

            uint8_t param_goal_position[4];
            bool dxl_addparam_result = false;                // addParam result
            dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, ADDR_MX_GOAL_POSITION, 4);

            // Add Dynamixel#1 ~22 goal position value to the Syncwrite storage
            for(int i = 0; i < qnode.Dx_msg.id.size(); i++)
            {
                param_goal_position[0] = DXL_LOBYTE(qnode.Dx_msg.position[i]);
                param_goal_position[1] = DXL_HIBYTE(qnode.Dx_msg.position[i]);
                param_goal_position[2] = DXL_LOBYTE(qnode.Dx_msg.speed[i]);
                param_goal_position[3] = DXL_HIBYTE(qnode.Dx_msg.speed[i]);
                dxl_addparam_result = groupSyncWrite.addParam(qnode.Dx_msg.id[i], param_goal_position);
                if (dxl_addparam_result != true)
                {
                    ROS_ERROR("[ID] %u, roupSyncWrite addparam failed",i);
                    break;
                }
            }
            groupSyncWrite.txPacket();
            groupSyncWrite.clearParam();

            break;
        }
        case 3: //speed and position
        {
            uint8_t param_goal_position[4];
            bool dxl_addparam_result = false;                // addParam result
            dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, ADDR_MX_GOAL_POSITION, 4);

            // Add Dynamixel#1 ~22 goal position value to the Syncwrite storage
            for(int i = 0; i < qnode.Dx_msg.id.size(); i++)
            {
                param_goal_position[0] = DXL_LOBYTE(qnode.Dx_msg.position[i]);
                param_goal_position[1] = DXL_HIBYTE(qnode.Dx_msg.position[i]);
                param_goal_position[2] = DXL_LOBYTE(qnode.Dx_msg.speed[i]);
                param_goal_position[3] = DXL_HIBYTE(qnode.Dx_msg.speed[i]);
                dxl_addparam_result = groupSyncWrite.addParam(qnode.Dx_msg.id[i], param_goal_position);
                if (dxl_addparam_result != true)
                {
                    ROS_ERROR("[ID] %u, roupSyncWrite addparam failed",i);
                    break;
                }
            }
            groupSyncWrite.txPacket();
            groupSyncWrite.clearParam();

            break;
        }
        case 4: //torque off
        {
            uint8_t param_goal_position[2];
            bool dxl_addparam_result = false;                // addParam result
            dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, ADDR_MX_TORQUE_ENABLE, 2);

            // Add Dynamixel#1 ~22 goal position value to the Syncwrite storage
            for(int i = 0; i < qnode.Dx_msg.id.size(); i++)
            {
                param_goal_position[0] = DXL_LOBYTE(TORQUE_DISABLE);
                param_goal_position[1] = DXL_HIBYTE(TORQUE_DISABLE);
                dxl_addparam_result = groupSyncWrite.addParam(qnode.Dx_msg.id[i], param_goal_position);
                if (dxl_addparam_result != true)
                {
                    ROS_ERROR("[ID] %u, roupSyncWrite addparam failed",i);
                    break;
                }
            }
            groupSyncWrite.txPacket();
            groupSyncWrite.clearParam();

            break;
        }

            //    case 5: // motion
            //    {

            //        break;
            //    }
        default:
            break;
        }
    }
}

void MainWindow::MotionCallback()
{
    cout<<"MotionCallback"<<endl;
    if(isMotion == false)
    {
        dxlNowPosition();
        m_nowTime = 0;
        mTimer = new QTimer(this);
        connect(mTimer, SIGNAL(timeout()),this, SLOT(motionPlay()));
        mTimer->start(7.8); //7.8ms

        isMotion = true;
    }
}

void MainWindow::dxlNowPosition() //
{
    msg_generate::Step_msg step_Info;

    motion_Info = qnode.Motion_msg;
    step_Info = motion_Info.motion_data[0];


    dynamixel::GroupBulkRead groupBulkRead(portHandler_, packetHandler_);
    uint32_t dxl_present_position = 0;                        // Dynamixel moving status
    bool dxl_addparam_result = false;                // addParam result
    int dxl_comm_result = COMM_TX_FAIL;

    m_motionData.clear();

    for(int i = 0 ; i<23; i++)
    {

        dxl_addparam_result = groupBulkRead.addParam(i, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
        dxl_comm_result = groupBulkRead.txRxPacket();
        dxl_present_position = step_Info.position[i];
        cout<<"id: "<<i<<"  dxl_present_position: "<<(int)dxl_present_position<<endl;

        if(dxl_present_position <= 0)
        {
            motionData motion;
            motion.id = i;
            motion.nowPosition = 2048;
            motion.proportion = (step_Info.position[i] - motion.nowPosition) / step_Info.time;
            m_motionData.push_back(motion);
        }
        else
        {
            motionData motion;
            motion.id = i;
            motion.nowPosition = dxl_present_position;
            if( abs(step_Info.position[i] - motion.nowPosition) < 10 ) // 오차범위
            {
                motion.proportion = 0;

            }
            else
            {
                motion.proportion = (step_Info.position[i] - motion.nowPosition) / step_Info.time;

            }
            m_motionData.push_back(motion);
        }
        groupBulkRead.clearParam();
    }
}

void MainWindow::Return_persent_position()
{
    dxl_id_Info = qnode.dxl_id_msg;

    if(dxl_id_Info.r_arm == 1 && dxl_id_Info.l_arm == 1)
    {
        cout << "START" << endl;

        //BOTH
        for(int i = 0; i < 5; i ++)
        {
            dxl_pos_Info.r_dxl_position.push_back(ReadPosition(R_arm_id[i]));
            dxl_pos_Info.l_dxl_position.push_back(ReadPosition(L_arm_id[i]));
        }

        qnode.dxl_position_pub.publish(dxl_pos_Info);

        dxl_pos_Info.r_dxl_position.clear();
        dxl_pos_Info.l_dxl_position.clear();
    }
    else if(dxl_id_Info.r_arm == 0 && dxl_id_Info.l_arm == 0)
    {
        cout << "STOP" << endl;
        dxl_pos_Info.r_dxl_position.clear();
        dxl_pos_Info.l_dxl_position.clear();
    }
}

int MainWindow::ReadPosition(int id) //dxl_persent_position //njh
{
    dynamixel::GroupBulkRead groupBulkRead(portHandler_, packetHandler_);
    uint32_t dxl_present_position = 0;                        // Dynamixel moving status
    bool dxl_addparam_result = false;                // addParam result
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_addparam_result = groupBulkRead.addParam(id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);
    dxl_comm_result = groupBulkRead.txRxPacket();
        cout<<"dxl_comm_result"<<dxl_comm_result<<endl;
    dxl_present_position = groupBulkRead.getData(id, ADDR_MX_PRESENT_POSITION, LEN_MX_PRESENT_POSITION);

    return dxl_present_position;
}

int MainWindow::ReadLoad(int id)
{
    dynamixel::GroupBulkRead groupBulkRead(portHandler_, packetHandler_);
    uint32_t dxl_present_position = 0;                        // Dynamixel moving status
    bool dxl_addparam_result = false;                // addParam result
    int dxl_comm_result = COMM_TX_FAIL;

    dxl_addparam_result = groupBulkRead.addParam(id, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD);
    dxl_comm_result = groupBulkRead.txRxPacket();

    dxl_present_position = groupBulkRead.getData(id, ADDR_MX_PRESENT_LOAD, LEN_MX_PRESENT_LOAD);

    return dxl_present_position;
}

void MainWindow::PIDSet(int id, int P, int I, int D)
{
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, ADDR_MX_D_GAIN, 3);
    uint8_t param_PID_gain[3];
    bool dxl_addparam_result = false;                // addParam result
    //    int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position
    int dxl_comm_result = COMM_TX_FAIL;

    param_PID_gain[0] = DXL_LOBYTE(D);
    param_PID_gain[1] = DXL_HIBYTE(I);
    param_PID_gain[2] = DXL_LOBYTE(P);
    dxl_addparam_result = groupSyncWrite.addParam(id, param_PID_gain);

    dxl_comm_result = groupSyncWrite.txPacket();
    cout<<"dxl_comm_result: "<<dxl_comm_result<<endl;

}

void MainWindow::wheel_Mode_callback()
{
    cout << "cpp_wheel_mode" << endl;

    wheelMode(qnode.wheel_msg.id, qnode.wheel_msg.speed);

    cout << "id >> " << qnode.wheel_msg.id << endl;
    cout << "speed >> " << qnode.wheel_msg.speed << endl;
}

void MainWindow::wheelMode(int id, int value)
{
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, ADDR_MX_MOVING_SPEED, LEN_MX_MOVING_SPEED);
    uint8_t param_moving[2];
    bool dxl_addparam_result = false;                // addParam result
    int dxl_comm_result = COMM_TX_FAIL;

    param_moving[0] = DXL_LOBYTE(value);
    param_moving[1] = DXL_HIBYTE(value);
    dxl_addparam_result = groupSyncWrite.addParam(id, param_moving);

    dxl_comm_result = groupSyncWrite.txPacket();
}


void MainWindow::motionPlay()
{
    msg_generate::Step_msg step_Info = motion_Info.motion_data[motionStep];

    //    cout<<"qnode.Motion_msg.repeat: "<<qnode.Motion_msg.repeat<<endl;
    cout<<"motion_Info.repeat: "<<motion_Info.repeat<<endl;

    //        cout<<"111111111"<<endl;
    //    cout<<"isDelay: "<<isDelay<<endl;
    if(m_repeat >= motion_Info.repeat)
    {
        disconnect(mTimer, SIGNAL(timeout()),this, SLOT(motionPlay()));
        mTimer->stop();
        delete mTimer;
        isMotion = false;
        m_repeat = 0;

        motion_end_Info.motion_end = 1;
        qnode.motion_end_pub.publish(motion_end_Info);

        cout<<"motion finish"<<endl;
    }
    else
    {
        if(isDelay == true)
        {
            //        cout<<"m_delayTime: "<<m_delayTime<<endl;
            m_delayTime++;
            if(m_delayTime >= step_Info.delay)
            {
                isDelay = false;
                m_delayTime = 0;
                step_Info.delay = 0;
            }
        }
        if(isDelay == false)
        {
            m_nowTime++;

            if(m_nowTime >= step_Info.time)
            {
                if(step_Info.delay == 0)
                {
                    motionStep++;
                    m_nowTime = 0;
                    m_motionData.clear();
                    //                                    cout<<"222222222"<<endl;
                    if(motionStep < qnode.Motion_msg.max_step)
                    {
                        msg_generate::Step_msg temp_step_Info;
                        temp_step_Info = motion_Info.motion_data[motionStep-1];

                        step_Info = motion_Info.motion_data[motionStep];

                        //                                            cout<<"333333333"<<endl;

                        for(int i = 0; i < step_Info.id.size(); i++)
                        {
                            motionData motion;
                            motion.nowPosition = temp_step_Info.position[i];
                            motion.proportion = (step_Info.position[i] - motion.nowPosition) / step_Info.time;
                            motion.id = i;
                            m_motionData.push_back(motion);
                        }
                        //                                            cout<<"4444444444"<<endl;

                    }
                    else
                    {
                        msg_generate::Step_msg temp_step_Info;

                        temp_step_Info = motion_Info.motion_data[motionStep-1];
                        step_Info = motion_Info.motion_data[0];

                        for(int i = 0; i < step_Info.id.size(); i++)
                        {
                            motionData motion;
                            motion.nowPosition = temp_step_Info.position[i];
                            motion.proportion = (step_Info.position[i] - motion.nowPosition) / step_Info.time;
                            motion.id = i;
                            m_motionData.push_back(motion);
                        }
                        m_repeat++;
                        motionStep = 0;
                        cout<<"m_repeat: "<<m_repeat<<endl;
                        cout<<"5555555555"<<endl;
                    }
                }
                else
                {
                    isDelay = true;
                }
            }
            else
            {

                uint8_t param_goal_position[4];
                bool dxl_addparam_result = false;                // addParam result
                dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, ADDR_MX_GOAL_POSITION, 4);



                for(int i = 0; i < step_Info.id.size(); i++)    // Add Dynamixel#1 ~22 goal position value to the Syncwrite storage
                {
                    //            cout<<"m_motionData[i].nowPosition + m_motionData[i].proportion*m_nowTime: "<<m_motionData[i].nowPosition + m_motionData[i].proportion*m_nowTime<<endl;
                    param_goal_position[0] = DXL_LOBYTE((int)(m_motionData[i].nowPosition + m_motionData[i].proportion*m_nowTime));
                    param_goal_position[1] = DXL_HIBYTE((int)(m_motionData[i].nowPosition + m_motionData[i].proportion*m_nowTime));
                    param_goal_position[2] = DXL_LOBYTE((int)400);
                    param_goal_position[3] = DXL_HIBYTE((int)400);
                    dxl_addparam_result = groupSyncWrite.addParam((int)step_Info.id[i], param_goal_position);
                    if (dxl_addparam_result != true)
                    {
                        ROS_ERROR("[ID] %u, roupSyncWrite addparam failed",i);
                        break;
                    }
                    cout<<"m_motionData["<<i<<"].nowPosition + m_motionData["<<i<<"].proportion*m_nowTime; "<<m_motionData[i].nowPosition + m_motionData[i].proportion*m_nowTime<<endl;

                }
                //                            cout<<"6666666666"<<endl;

                int dxl_comm_result = COMM_TX_FAIL;
                dxl_comm_result = groupSyncWrite.txPacket();
                //        cout<<"dxl_comm_result: "<<dxl_comm_result<<endl;

                groupSyncWrite.clearParam();
                //                            cout<<"7777777777"<<endl;

            }
        }
    }


}

void MainWindow::on_pushButton_TxPosition_clicked()
{
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, ADDR_MX_GOAL_POSITION, 4);
    uint8_t param_goal_position[4];
    bool dxl_addparam_result = false;                // addParam result
    //    int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE};         // Goal position
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result

    int Position01 = ui.lineEdit_TxPosition01->text().toInt();
    int Id01 = ui.lineEdit_TxId01->text().toInt();
    int Position02 = ui.lineEdit_TxPosition02->text().toInt();
    int Id02 = ui.lineEdit_TxId02->text().toInt();
    int Position03 = ui.lineEdit_TxPosition03->text().toInt();
    int Id03 = ui.lineEdit_TxId03->text().toInt();
    int Position04 = ui.lineEdit_TxPosition04->text().toInt();
    int Id04 = ui.lineEdit_TxId04->text().toInt();

    param_goal_position[0] = DXL_LOBYTE(Position01);
    param_goal_position[1] = DXL_HIBYTE(Position01);
    param_goal_position[2] = DXL_LOBYTE(spd);
    param_goal_position[3] = DXL_HIBYTE(spd);
    dxl_addparam_result = groupSyncWrite.addParam(Id01, param_goal_position);

    param_goal_position[0] = DXL_LOBYTE(Position02);
    param_goal_position[1] = DXL_HIBYTE(Position02);
    param_goal_position[2] = DXL_LOBYTE(spd);
    param_goal_position[3] = DXL_HIBYTE(spd);
    dxl_addparam_result = groupSyncWrite.addParam(Id02, param_goal_position);

    param_goal_position[0] = DXL_LOBYTE(Position03);
    param_goal_position[1] = DXL_HIBYTE(Position03);
    param_goal_position[2] = DXL_LOBYTE(spd);
    param_goal_position[3] = DXL_HIBYTE(spd);
    dxl_addparam_result = groupSyncWrite.addParam(Id03, param_goal_position);

    param_goal_position[0] = DXL_LOBYTE(Position04);
    param_goal_position[1] = DXL_HIBYTE(Position04);
    param_goal_position[2] = DXL_LOBYTE(spd);
    param_goal_position[3] = DXL_HIBYTE(spd);
    dxl_addparam_result = groupSyncWrite.addParam(Id04, param_goal_position);


    // Syncwrite goal position
    dxl_comm_result = groupSyncWrite.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) packetHandler_->getTxRxResult(dxl_comm_result);

    // Clear syncwrite parameter storage
    groupSyncWrite.clearParam();


    // ----------------------------------- 현재모터가 Goal Position에 도착했는지
    dynamixel::GroupBulkRead groupBulkRead(portHandler_, packetHandler_);
    uint8_t dxl2_moving = 0;                        // Dynamixel moving status

    dxl_addparam_result = groupBulkRead.addParam(14, ADDR_MX_MOVING, LEN_MX_MOVING);
    dxl_comm_result = groupBulkRead.txRxPacket();
    dxl2_moving = groupBulkRead.getData(14, ADDR_MX_MOVING, LEN_MX_MOVING); // 결과값이 0이면 Goal Position 도착 1이면 아직 실행중
    cout<<"dxl2_moving: "<<dxl2_moving<<endl;
}

void MainWindow::on_pushButton_read_clicked()
{
    int nowPosition = ReadPosition(ui.lineEdit_RxId01->text().toInt());
    ui.lineEdit_Rxposition->setText(QString::number(nowPosition));
}
void MainWindow::on_pushButton_readLoad_clicked()
{
    int nowPosition = ReadLoad(ui.lineEdit_RxLoadId01->text().toInt());
    ui.lineEdit_RxLoad->setText(QString::number(nowPosition));
}
void MainWindow::on_pushButton_PIDSet_clicked()
{
    PIDSet(ui.lineEdit_TxPIDId01->text().toInt(), ui.lineEdit_TxPGain01->text().toInt(), ui.lineEdit_TxIGain01->text().toInt(), ui.lineEdit_TxDGain01->text().toInt());
}

void MainWindow::on_pushButton_torqueoff_clicked()
{
    uint8_t param_goal_position[2];
    bool dxl_addparam_result = false;                // addParam result
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler_, packetHandler_, ADDR_MX_TORQUE_ENABLE, 2);

    // Add Dynamixel#1 ~22 goal position value to the Syncwrite storage
    for(int i = 0; i < 23; i++)
    {
        param_goal_position[0] = DXL_LOBYTE(TORQUE_DISABLE);
        param_goal_position[1] = DXL_LOBYTE(TORQUE_DISABLE);

        dxl_addparam_result = groupSyncWrite.addParam(i, param_goal_position);
        if (dxl_addparam_result != true)
        {
            ROS_ERROR("[ID] %u, roupSyncWrite addparam failed",i);
            break;
        }
    }
    int dxl_comm_result = COMM_TX_FAIL;
    dxl_comm_result = groupSyncWrite.txPacket();
    cout<<"dxl_comm_result: "<<dxl_comm_result<<endl;
    groupSyncWrite.clearParam();
}
void MainWindow::on_pushButton_portSet_clicked()
{
    DynamixelInit();
}

void MainWindow::on_horizontalSlider_valueChanged(int value)
{
    spd = value;
    ui.label_5->setText(QString::number(spd));
}
}  // namespace dynamixel_serial



