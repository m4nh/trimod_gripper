#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QByteArray>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    this->socket = new QUdpSocket(this);
    //this->socket->bind(QHostAddress::LocalHost, 9999);
    this->time =0;


    this->joint_message.command=666;
    this->joint_message.time = this->time;
    for(int i = 0; i < 32; i++){
       this->joint_message.payload[i]=0;
    }

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updateJoint(int index , float value){

    this->joint_message.payload[index] = value;
qDebug()<<"Updating joint "<<index << " -> "<<this->joint_message.payload[index];
    char* arr = reinterpret_cast<char*>(&this->joint_message);
    //char* arr= new char[40];
    //memcpy ( arr, &m,sizeof(arr));
    this->socket->writeDatagram(arr,4*32+8,QHostAddress::LocalHost,9999);
    this->joint_message.time++;
}

void MainWindow::on_j0_slider_valueChanged(int value)
{
    updateJoint(0,value);
}

void MainWindow::on_j1_slider_valueChanged(int value)
{
    updateJoint(1,value);
}

void MainWindow::on_j2_slider_valueChanged(int value)
{
    updateJoint(2,value);
}

void MainWindow::on_j3_slider_valueChanged(int value)
{
    updateJoint(3,value);
}

void MainWindow::on_j4_slider_valueChanged(int value)
{
    updateJoint(4,value);
}

void MainWindow::on_j5_slider_valueChanged(int value)
{
    updateJoint(5,value);
}

void MainWindow::on_j6_slider_valueChanged(int value)
{
    updateJoint(6,value);
}
