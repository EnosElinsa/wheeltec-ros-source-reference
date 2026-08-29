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
#include "rclcpp/rclcpp.hpp"
#include <QMessageBox>
#include <iostream>
#include "../include/qt_ros_test/main_window.hpp"
#include <QObject>
#include <QVBoxLayout>
#include <opencv2/opencv.hpp>

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace qt_ros_test {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application
    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).

    //btn_disableall(true);
    //connect
    QObject::connect(ui.pushButton_u, SIGNAL(released()), this,SLOT(pushButton_stop()));
    QObject::connect(ui.pushButton_ru, SIGNAL(released()), this,SLOT(pushButton_stop()));
    QObject::connect(ui.pushButton_lu, SIGNAL(released()), this,SLOT(pushButton_stop()));
    QObject::connect(ui.pushButton_r, SIGNAL(released()), this,SLOT(pushButton_stop()));
    QObject::connect(ui.pushButton_l, SIGNAL(released()), this,SLOT(pushButton_stop()));
    QObject::connect(ui.pushButton_rd, SIGNAL(released()), this,SLOT(pushButton_stop()));
    QObject::connect(ui.pushButton_d, SIGNAL(released()), this,SLOT(pushButton_stop()));
    QObject::connect(ui.pushButton_ld, SIGNAL(released()), this,SLOT(pushButton_stop()));
    QObject::connect(ui.pushButton_stop, SIGNAL(pressed()), this,SLOT(pushButton_stop()));

    QObject::connect(ui.pushButton_u, SIGNAL(pressed()), this,SLOT(pushButton_go()));
    QObject::connect(ui.pushButton_ru, SIGNAL(pressed()), this,SLOT(pushButton_go()));
    QObject::connect(ui.pushButton_lu, SIGNAL(pressed()), this,SLOT(pushButton_go()));
    QObject::connect(ui.pushButton_r, SIGNAL(pressed()), this,SLOT(pushButton_go()));
    QObject::connect(ui.pushButton_l, SIGNAL(pressed()), this,SLOT(pushButton_go()));
    QObject::connect(ui.pushButton_rd, SIGNAL(pressed()), this,SLOT(pushButton_go()));
    QObject::connect(ui.pushButton_d, SIGNAL(pressed()), this,SLOT(pushButton_go()));
    QObject::connect(ui.pushButton_ld, SIGNAL(pressed()), this,SLOT(pushButton_go()));

    QObject::connect(&qnode, SIGNAL(batteryState(double)), this,SLOT(slotbatteryState(double)));
    QObject::connect(&qnode, SIGNAL(speedState(double,double,double,double,double,double)), this,SLOT(slotspeedState(double,double,double,double,double,double)));
    connect(&qnode, SIGNAL(image_val(QImage)), this,SLOT(updata_image(QImage)));

    killall = new QProcess;
    connect(killall,SIGNAL(finished(int, QProcess::ExitStatus)), SLOT(cmd_finished(int, QProcess::ExitStatus)));

    //打开速度仪表盘
    speedxboard = new CCtrlDashBoard(ui.widget_x);
    speedxboard->setGeometry(ui.widget_x->rect());
    speedyboard = new CCtrlDashBoard(ui.widget_y);
    speedyboard->setGeometry(ui.widget_y->rect());

    m_pTimer = new QTimer(this);
    connect(m_pTimer,SIGNAL(timeout()),this,SLOT(slottimeout()));

    init_cmd();
    qnode.init();
    slot_rosOpen();
}

MainWindow::~MainWindow() {
    QProcess::execute("killall -s INT -w ros2\n");
    if(_cmd)_cmd->close();
    delete _cmd;
    _cmd=NULL;
}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

//cmd:发送命令方式一
//特点：运行其他功能时可以显示远程界面：如运行寻线时，可以显示寻线窗口
//缺点：程序不能显示运行功能时打印出来的日志或者显示的太乱了
//用法：
//_cmd->write("ssh -Y wheeltec@192.168.0.100\n"                       //连接小车
//            "source /opt/ros/melodic/setup.bash\n"                  //初始化环境
//            "source wheeltec_robot/devel/setup.bash\n"
//            "roslaunch simple_follower line_follower.launch\n");     //运行相关功能（修改这句话即可）
void MainWindow::init_cmd()
{
    _cmd = new QProcess;
    _cmd->start("bash");
    QObject::connect(_cmd,SIGNAL(readyReadStandardOutput()),this,SLOT(slot_cmd_read()));
    QObject::connect(_cmd,SIGNAL(readyReadStandardError()),this,SLOT(slot_cmd_err()));

}

void MainWindow::btn_disableall(bool _enabled)
{
    ui.pushButton_base->setDisabled(_enabled);
    ui.pushButton_laser->setDisabled(_enabled);
    ui.pushButton_joy->setDisabled(_enabled);
    ui.pushButton_2dmap->setDisabled(_enabled);
    ui.pushButton_2dmap_2->setDisabled(_enabled);
    ui.pushButton_2dmap_3->setDisabled(_enabled);
    ui.pushButton_2dmapsave->setDisabled(_enabled);
    ui.pushButton_2dnav->setDisabled(_enabled);
    ui.pushButton_3dmap->setDisabled(_enabled);
    //ui.pushButton_3dnav->setDisabled(_enabled);
    ui.pushButton_laserfollower->setDisabled(_enabled);
    ui.pushButton_linefollower->setDisabled(_enabled);
    ui.pushButton_visfollower->setDisabled(_enabled);
    ui.pushButton_webusbcam->setDisabled(_enabled);
    ui.checkBox_key->setDisabled(_enabled);
    ui.checkBox__opencamera->setDisabled(_enabled);
    ui.pushButton_2->setDisabled(_enabled);
}
//关闭roslaunch进程下的所有功能
void MainWindow::closeroslaunch()
{
    killall->start("sh /home/wheeltec/wheeltec_ros2/src/qt_ros_test/script/killall.sh\n");
}

void MainWindow::cmd_finished(int exitCode, QProcess::ExitStatus exitStatus)
{
    ui.textEdit_cmd->append("-----------------成功关闭，请继续下一步操作--------------------");
    btn_disableall(false);
    ui.checkBox_key->setChecked(false);
    ui.checkBox__opencamera->setChecked(false);
}

//cmd命令回调函数：：打印日志
void MainWindow::slot_cmd_read()
{
    qint64 maxSize = 512;
    char buffer[maxSize];
    qint64 len;
    while(true)
    {
        len = _cmd->readLine(buffer, maxSize);
        if(len <= 0) {
            break;
        }
        QString str = QString::fromLocal8Bit(buffer);
        ui.textEdit_cmd->append("<font color=\"#000000\">"+str+"</font> ");
        ui.textEdit_cmd->moveCursor(QTextCursor::End);
        if(str.contains("Map saved successfully"))
        {
            qDebug()<<str;
            ui.pushButton_2dmapsave->setChecked(false);
        }
     }
}
//cmd命令回调函数：打印错误
void MainWindow::slot_cmd_err()
{
    QString str = QString::fromLocal8Bit(_cmd->readAllStandardError());
    ui.textEdit_cmd->append("<font color=\"#FF0000\">"+str+"</font>");
}


//ros打开时，处理图标、ros状态、读取参数
void MainWindow::slot_rosOpen()
{
    ui.label_7->setPixmap(QPixmap::fromImage(QImage("://images/robot.png")));
    ui.label_8->setText("在线");
    ui.label_9->setPixmap(QPixmap::fromImage(QImage("://images/ok.png")));
    qnode.nh->declare_parameter<std::string>("user_passward","dongguan");
    qnode.nh->get_parameter("user_passward", user_passward);
}
/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright wheeltec Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
** 读取设置
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "qt_ros_test");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.0.100:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.0.136")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    bool checked = settings.value("use_environment_variables", false).toBool();
}
/*****************************************************************************
**
** 保存设置
*****************************************************************************/
void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "qt_ros_test");
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace qt_ros_test

//使能键盘控制按钮，打开基本turn—on节点 
void qt_ros_test::MainWindow::on_checkBox_key_clicked(bool checked)
{
    if(ui.checkBox_key->isChecked())
    {
        _cmd->write("ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py\n");
        btn_disableall(true);
        ui.checkBox_key->setEnabled(true);
        ui.textEdit_cmd->append("-------------------------------------------正在打开键盘控制-------------------------------------------");
    }
    else
    {
        closeroslaunch();
        ui.textEdit_cmd->append("-------------------------------------------正在关闭键盘控制，请稍候-------------------------------------------");
    }
}
//控制小车运动的槽函数
void qt_ros_test::MainWindow::pushButton_go()
{
    double speed = ui.horizontalSlider_v->value();
    speed = speed/100;
    double cor = ui.horizontalSlider_c->value();
    cor = cor/100;
    if(ui.pushButton_lu->isDown())qnode.pub_cmd(ui.checkBox_mec->isChecked(),1,speed,cor);
    else if(ui.pushButton_u->isDown())qnode.pub_cmd(ui.checkBox_mec->isChecked(),2,speed,cor);
    else if(ui.pushButton_ru->isDown())qnode.pub_cmd(ui.checkBox_mec->isChecked(),3,speed,cor);
    else if(ui.pushButton_l->isDown())qnode.pub_cmd(ui.checkBox_mec->isChecked(),4,speed,cor);
    else if(ui.pushButton_r->isDown())qnode.pub_cmd(ui.checkBox_mec->isChecked(),6,speed,cor);
    else if(ui.pushButton_ld->isDown())qnode.pub_cmd(ui.checkBox_mec->isChecked(),9,speed,cor);
    else if(ui.pushButton_d->isDown())qnode.pub_cmd(ui.checkBox_mec->isChecked(),8,speed,cor);
    else if(ui.pushButton_rd->isDown())qnode.pub_cmd(ui.checkBox_mec->isChecked(),7,speed,cor);
}
//控制小车停止的槽函数
void qt_ros_test::MainWindow::pushButton_stop()
{
    qnode.pub_cmd(ui.checkBox_mec->isChecked(),5,0,0);
}
//图像显示
void qt_ros_test::MainWindow::updata_image(QImage im)
{
    ui.label_image->setPixmap(QPixmap::fromImage(im).scaled(ui.label_image->width()-10, ui.label_image->height()-10));
}
//电量显示
void qt_ros_test::MainWindow::slotbatteryState(double msg)
{
    ui.label_power->setText(QString::number(msg) + "V");
    if(msg>25)ui.progressBar->setValue(100);
    else if(msg<24 && msg>=20)ui.progressBar->setValue((1-(25-msg)/5)*100);
    else if(msg<20 && msg >18)ui.progressBar->setValue(0);
    else if(msg>12.5)ui.progressBar->setValue(100);
    else if(msg<12.5 && msg>=10)ui.progressBar->setValue((1-(12.5-msg)/2.5)*100);
    else if(msg<10)ui.progressBar->setValue(0);

    if(ui.progressBar->value()<=30)ui.label_4->setPixmap(QPixmap::fromImage(QImage("://images/power-l.png")));
    else if(ui.progressBar->value()>30)ui.label_4->setPixmap(QPixmap::fromImage(QImage("://images/power-v.png")));
}
//速度显示
void qt_ros_test::MainWindow::slotspeedState(double l_x, double l_y, double l_z, double a_x, double a_y, double a_z)
{
    ui.label_linerx->setText(QString::number(l_x));
    ui.label_linery->setText(QString::number(l_y));
    ui.label_linerz->setText(QString::number(l_z));
    ui.label_angerx->setText(QString::number(a_x));
    ui.label_angery->setText(QString::number(a_y));
    ui.label_angerz->setText(QString::number(a_z));

    speedxboard->setValue(abs(l_x * 100));
    speedyboard->setValue(abs(l_y * 100));
}

//速度调节
void qt_ros_test::MainWindow::on_horizontalSlider_v_sliderMoved(int position)
{
    double val = ui.horizontalSlider_v->value();
    val = val/100;
    ui.label_v->setText(QString("%1").arg(val));
}

void qt_ros_test::MainWindow::on_horizontalSlider_c_sliderMoved(int position)
{
    double val = ui.horizontalSlider_c->value();
    val = val/100;
    ui.label_c->setText(QString("%1").arg(val));
}

void qt_ros_test::MainWindow::on_checkBox__opencamera_clicked(bool checked)
{
    if(ui.checkBox__opencamera->isChecked())
    {
        _cmd->write("ros2 launch turn_on_wheeltec_robot wheeltec_camera.launch.py\n");
        btn_disableall(true);
        ui.checkBox__opencamera->setEnabled(true);
        ui.textEdit_cmd->append("-------------------------------------------正在打开摄像头-------------------------------------------");
    }
    else
    {
        closeroslaunch();
        ui.label_image->clear();
        ui.textEdit_cmd->append("-------------------------------------------正在关闭摄像头，请稍候-------------------------------------------\n");
    }
}

void qt_ros_test::MainWindow::on_pushButton_base_clicked(bool checked)
{
    if(ui.pushButton_base->isChecked())
    {
        _cmd->write("ros2 launch turn_on_wheeltec_robot turn_on_wheeltec_robot.launch.py\n");
        btn_disableall(true);
        ui.pushButton_base->setEnabled(true);  
        ui.textEdit_cmd->append("-------------------------------------------正在打开底盘控制-------------------------------------------");
    }
    else
    {
        closeroslaunch();
        ui.textEdit_cmd->append("-------------------------------------------正在关闭底盘控制，请稍候-------------------------------------------");
    }
}

void qt_ros_test::MainWindow::on_pushButton_laser_clicked(bool checked)
{
    if(ui.pushButton_laser->isChecked())
    {
        _cmd->write("ros2 launch turn_on_wheeltec_robot wheeltec_lidar.launch.py\n");
        btn_disableall(true);
        ui.pushButton_laser->setEnabled(true);  
        ui.textEdit_cmd->append("-------------------------------------------正在打开雷达-------------------------------------------");
    }
    else
    {
        closeroslaunch();
        ui.textEdit_cmd->append("-------------------------------------------正在关闭雷达，请稍候-------------------------------------------\n");
    }
}

void qt_ros_test::MainWindow::on_pushButton_joy_clicked(bool checked)
{
    if(ui.pushButton_joy->isChecked())
    {
        _cmd->write("ros2 launch wheeltec_joy wheeltec_joy.launch.py\n");
        btn_disableall(true);
        ui.pushButton_joy->setEnabled(true);  
        ui.textEdit_cmd->append("-------------------------------------------正在打开USB手柄控制-------------------------------------------");
    }
    else
    {
        closeroslaunch();
        ui.textEdit_cmd->append("-------------------------------------------正在关闭USB手柄控制，请稍候-------------------------------------------\n");
    }
}


void qt_ros_test::MainWindow::on_pushButton_linefollower_clicked(bool checked)
{
    if(ui.pushButton_linefollower->isChecked())
    {
        _cmd->write("ros2 launch simple_follower_ros2 line_follower.launch.py\n");
        ui.checkBox__opencamera->setChecked(true);
        ui.checkBox_key->setChecked(true);
        btn_disableall(true);
        ui.pushButton_linefollower->setEnabled(true);  
        ui.textEdit_cmd->append("-------------------------------------------正在打开寻线-------------------------------------------");
    }
    else
    {
        closeroslaunch();
        ui.textEdit_cmd->append("-------------------------------------------正在关闭关闭寻线，请稍候-------------------------------------------");
    }
}

void qt_ros_test::MainWindow::on_pushButton_laserfollower_clicked(bool checked)
{
    if(ui.pushButton_laserfollower->isChecked())
    {
        _cmd->write("ros2 launch simple_follower_ros2 laser_follower.launch.py\n");
        ui.checkBox_key->setChecked(true);
        btn_disableall(true);
        ui.pushButton_laserfollower->setEnabled(true);
        ui.textEdit_cmd->append("-------------------------------------------正在打开雷达跟随-------------------------------------------");
    }
    else
    {
        closeroslaunch();
        ui.textEdit_cmd->append("--------------------------------正在关闭雷达跟随，请稍候------------------------------------");
    }
}

void qt_ros_test::MainWindow::on_pushButton_clicked(bool checked)
{
   std::map<std::string, std::vector<std::string>> topicinfo;
   topicinfo = qnode.nh->get_topic_names_and_types();
   //RCLCPP_INFO(get_logger(),topicinfo);
   ui.comboBox_carmeratopic->clear();
   for (auto topic : topicinfo)
   {
       QString string = QString::fromStdString(topic.first);
       QString stringtype = QString::fromStdString(topic.second[0]);
       if(stringtype=="sensor_msgs/msg/Image" || stringtype=="sensor_msgs/msg/CompressedImage")
           ui.comboBox_carmeratopic->addItem(string);
   }
}

void qt_ros_test::MainWindow::on_pushButton_subimg_clicked(bool checked)
{
   qDebug()<<"subscribtion to:"<<ui.comboBox_carmeratopic->currentText();
   qnode.sub_camera(ui.comboBox_carmeratopic->currentText());
   ui.label_image->clear();
}

void qt_ros_test::MainWindow::on_pushButton_visfollower_clicked(bool checked)
{
    if(ui.pushButton_visfollower->isChecked())
    {
        _cmd->write("ros2 launch simple_follower_ros2 visual_follower.launch.py\n");
        ui.checkBox__opencamera->setChecked(true);
        ui.checkBox_key->setChecked(true);
        btn_disableall(true);
        ui.pushButton_visfollower->setEnabled(true);
        ui.textEdit_cmd->append("-------------------------------------------正在打开视觉跟随-------------------------------------------");
    }
    else
    {
        closeroslaunch();
        ui.textEdit_cmd->append("-------------------------------------------正在关闭视觉跟随，请稍候-------------------------------------------");
    }
}

void qt_ros_test::MainWindow::on_pushButton_webusbcam_clicked(bool checked)
{
    if(ui.pushButton_webusbcam->isChecked())
    {
        
        _cmd->write("sh /home/wheeltec/wheeltec_ros2/src/qt_ros_test/script/web.sh\n");
        ui.checkBox__opencamera->setChecked(true);
        btn_disableall(true);
        ui.pushButton_webusbcam->setEnabled(true);
        ui.textEdit_cmd->append("-------------------------------------------正在打开web浏览器显示摄像头-------------------------------------------");
    }
    else
    {
        closeroslaunch();
        ui.textEdit_cmd->append("-------------------------------------------正在关闭web浏览器显示摄像头，请稍候-------------------------------------------");
    }
}


void qt_ros_test::MainWindow::on_pushButton_2dmap_clicked(bool checked)
{
    if(ui.pushButton_2dmap->isChecked())
    {
        _cmd->write("sh /home/wheeltec/wheeltec_ros2/src/qt_ros_test/script/gmapping.sh\n");
        ui.checkBox_key->setChecked(true);
        btn_disableall(true);
        ui.pushButton_2dmap->setEnabled(true);
        ui.pushButton_2dmapsave->setEnabled(true);
        ui.textEdit_cmd->append("-------------------------------------------正在打开gmapping建图-------------------------------------------");
        QEventLoop loop;
        //创建单次定时器,槽函数为事件循环的退出函数
        QTimer::singleShot(2000, &loop, SLOT(quit()));
        //事件循环开始执行,程序会卡在这里,直到定时时间到,本循环被退出
        loop.exec();
        ui.textEdit_cmd->append("open mapping");
        QProcess* rviz_cmd =new QProcess(this);
        rviz_cmd->startDetached("rviz2");
    }
    else
    {
        closeroslaunch();
        QProcess* rviz_cmd =new QProcess(this);
        rviz_cmd->startDetached("killall -2 rviz2");
        ui.textEdit_cmd->append("-------------------------------------------正在关闭gmapping建图，请稍候-------------------------------------------");
    }
} 


void qt_ros_test::MainWindow::on_pushButton_2dmap_2_clicked()
{
    if(ui.pushButton_2dmap_2->isChecked())
    {
        _cmd->write("sh /home/wheeltec/wheeltec_ros2/src/qt_ros_test/script/slam_toolbox.sh\n");
        ui.checkBox_key->setChecked(true);
        btn_disableall(true);
        ui.pushButton_2dmap_2->setEnabled(true);
        ui.pushButton_2dmapsave->setEnabled(true);
        ui.textEdit_cmd->append("-------------------------------------------正在打开slam_toolbox建图-------------------------------------------");
        QEventLoop loop;
        //创建单次定时器,槽函数为事件循环的退出函数
        QTimer::singleShot(2000, &loop, SLOT(quit()));
        //事件循环开始执行,程序会卡在这里,直到定时时间到,本循环被退出
        loop.exec();
        ui.textEdit_cmd->append("open mapping");
        QProcess* rviz_cmd =new QProcess(this);
        rviz_cmd->startDetached("rviz2");
    }
    else
    {
        closeroslaunch();
        QProcess* rviz_cmd =new QProcess(this);
        rviz_cmd->startDetached("killall -2 rviz2");
        ui.textEdit_cmd->append("-------------------------------------------正在关闭slam_toolbox建图，请稍候-------------------------------------------");

    }
}

void qt_ros_test::MainWindow::on_pushButton_2dmap_3_clicked()
{
    if(ui.pushButton_2dmap_3->isChecked())
    {
        _cmd->write("sh /home/wheeltec/wheeltec_ros2/src/qt_ros_test/script/cartographer.sh\n");
        ui.checkBox_key->setChecked(true);
        btn_disableall(true);
        ui.pushButton_2dmap_3->setEnabled(true);
        ui.pushButton_2dmapsave->setEnabled(true);
        ui.textEdit_cmd->append("-------------------------------------------正在打开cartographer建图-------------------------------------------");
        QEventLoop loop;
        //创建单次定时器,槽函数为事件循环的退出函数
        QTimer::singleShot(2000, &loop, SLOT(quit()));
        //事件循环开始执行,程序会卡在这里,直到定时时间到,本循环被退出
        loop.exec();
        ui.textEdit_cmd->append("open mapping");
        QProcess* rviz_cmd =new QProcess(this);
        rviz_cmd->startDetached("rviz2");
    }
    else
    {
        closeroslaunch();
        QProcess* rviz_cmd =new QProcess(this);
        rviz_cmd->startDetached("killall -2 rviz2");
        ui.textEdit_cmd->append("-------------------------------------------正在关闭cartographer建图，请稍候-------------------------------------------");

    }
}

void qt_ros_test::MainWindow::slotConnectStateChanged(bool bState, QString strIp, int nPort)
{
    qDebug()<<bState<<strIp;
}


void qt_ros_test::MainWindow::on_pushButton_2dmapsave_clicked(bool checked)
{
    if(ui.pushButton_2dmapsave->isChecked())
    {
        _cmd->write("ros2 launch wheeltec_nav2 save_map.launch.py\n");
        ui.textEdit_cmd->append("-------------------------------------------正在保存2D地图-------------------------------------------");
    }
    else
    {
    }
}

void qt_ros_test::MainWindow::on_pushButton_2dnav_clicked(bool checked)
{
    if(ui.pushButton_2dnav->isChecked())
    {
        _cmd->write("sh /home/wheeltec/wheeltec_ros2/src/qt_ros_test/script/nav.sh\n");
        btn_disableall(true);
        ui.pushButton_2dnav->setEnabled(true);
        ui.textEdit_cmd->append("-------------------------------------------正在打开2D导航-------------------------------------------");
        QEventLoop loop;
        //创建单次定时器,槽函数为事件循环的退出函数
        QTimer::singleShot(5000, &loop, SLOT(quit()));
        //事件循环开始执行,程序会卡在这里,直到定时时间到,本循环被退出
        loop.exec();
        QProcess* rviz_cmd =new QProcess(this);
        rviz_cmd->startDetached("rviz2");
    }
    else
    {
        closeroslaunch();
        QProcess* rviz_cmd =new QProcess(this);
        rviz_cmd->startDetached("killall -2 rviz2");
        ui.textEdit_cmd->append("-------------------------------------------正在关闭2D导航，请稍候-------------------------------------------");
    }
}

void qt_ros_test::MainWindow::on_pushButton_3dmap_clicked(bool checked)
{
    if(ui.pushButton_3dmap->isChecked())
    {
        _cmd->write("ros2 launch wheeltec_robot_rtab wheeltec_slam_rtab.launch.py\n");
        ui.checkBox__opencamera->setChecked(true);
        ui.checkBox_key->setChecked(true);
        btn_disableall(true);
        ui.pushButton_3dmap->setEnabled(true);
        ui.textEdit_cmd->append("-------------------------------------------正在打开3D建图-------------------------------------------");
        QEventLoop loop;
        //创建单次定时器,槽函数为事件循环的退出函数
        QTimer::singleShot(2000, &loop, SLOT(quit()));
        //事件循环开始执行,程序会卡在这里,直到定时时间到,本循环被退出
        loop.exec();
        ui.textEdit_cmd->append("open 3d mapping");
        QProcess* rviz_cmd =new QProcess(this);
        rviz_cmd->startDetached("rviz2");
    }
    else
    {
        closeroslaunch();
        QProcess* rviz_cmd =new QProcess(this);
        rviz_cmd->startDetached("killall -2 rviz2");
        QEventLoop loop;
        //创建单次定时器,槽函数为事件循环的退出函数
        QTimer::singleShot(2000, &loop, SLOT(quit()));
        //事件循环开始执行,程序会卡在这里,直到定时时间到,本循环被退出
        loop.exec();

        ui.textEdit_cmd->append("-------------------------------------------正在关闭3D建图，请稍候-------------------------------------------");

    }
}
void qt_ros_test::MainWindow::on_pushButton_3dnav_clicked(bool checked)
{
    if(ui.pushButton_3dnav->isChecked())
    {
        _cmd->write("ros2 launch wheeltec_robot_rtab wheeltec_nav2_rtab.launch.py\n");
        ui.checkBox__opencamera->setChecked(true);
        ui.checkBox_key->setChecked(true);
        btn_disableall(true);
        ui.pushButton_3dnav->setEnabled(true);
        ui.textEdit_cmd->append("-------------------------------------------正在打开3D导航-------------------------------------------");
        QEventLoop loop;
        //创建单次定时器,槽函数为事件循环的退出函数
        QTimer::singleShot(7000, &loop, SLOT(quit()));
        //事件循环开始执行,程序会卡在这里,直到定时时间到,本循环被退出
        loop.exec();
        ui.textEdit_cmd->append("open 3d nav");
        QProcess* rviz_cmd =new QProcess(this);
        rviz_cmd->startDetached("rviz2");
    }
    else
    {
        closeroslaunch();
        QProcess* rviz_cmd =new QProcess(this);
        rviz_cmd->startDetached("killall -2 rviz2");
        _cmd->write("killall -3 python\n");
        ui.textEdit_cmd->append("-------------------------------正在关闭3D导航，请稍候------------------------------");
    }
}

void qt_ros_test::MainWindow::on_pushButton_2_clicked(bool checked)
{
    _cmd->write(ui.lineEdit_cmd->text().toLocal8Bit()+"\n");
    ui.textEdit_cmd->append("\r\n");
    ui.textEdit_cmd->append(ui.lineEdit_cmd->text());
    ui.lineEdit_cmd->clear();
}

void qt_ros_test::MainWindow::on_lineEdit_cmd_returnPressed()
{
    on_pushButton_2_clicked(true);
}

void qt_ros_test::MainWindow::keyPressEvent(QKeyEvent *event)
{
    if(event->isAutoRepeat()==false)
    {
        double speed = ui.horizontalSlider_v->value();
        speed = speed/100;
        double cor = ui.horizontalSlider_c->value();
        cor = cor/100;
        switch(event->key())
        {
        case Qt::Key_U:
                qnode.pub_cmd(ui.checkBox_mec->isChecked(),1,speed,cor);
            break;
        case Qt::Key_I:
            qnode.pub_cmd(ui.checkBox_mec->isChecked(),2,speed,cor);
            break;
        case Qt::Key_O:
            qnode.pub_cmd(ui.checkBox_mec->isChecked(),3,speed,cor);
            break;
        case Qt::Key_J:
            qnode.pub_cmd(ui.checkBox_mec->isChecked(),4,speed,cor);
            break;
        case Qt::Key_L:
            qnode.pub_cmd(ui.checkBox_mec->isChecked(),6,speed,cor);
            break;
        case Qt::Key_M:
            qnode.pub_cmd(ui.checkBox_mec->isChecked(),9,speed,cor);
            break;
        case Qt::Key_Comma:
            qnode.pub_cmd(ui.checkBox_mec->isChecked(),8,speed,cor);
            break;
        case Qt::Key_Period:
            qnode.pub_cmd(ui.checkBox_mec->isChecked(),7,speed,cor);
            break;
        default:
            break;
        }
        keypress = true;
    }
}

void qt_ros_test::MainWindow::keyReleaseEvent(QKeyEvent *event)
{
    if(event->isAutoRepeat()==false)
    {
        keypress = false;
        m_pTimer->start(20);
    }
}

void qt_ros_test::MainWindow::slottimeout()
{
    if(keypress==false && rclcpp::ok())
    {
        qnode.pub_cmd(ui.checkBox_mec->isChecked(),5,0,0);
        m_pTimer->stop();
    }
}


void qt_ros_test::MainWindow::on_quit_button_2_clicked(bool checked)
{
    closeroslaunch();
}
