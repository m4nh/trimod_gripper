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
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_j0_slider_valueChanged(int value)
{
    qDebug()<<value;
    UDPMessage m;
    m.command=666;
    m.time = this->time;
    m.payload[0]=value;


    char* arr = reinterpret_cast<char*>(&m.command);
    //char* arr= new char[40];
    //memcpy ( arr, &m,sizeof(arr));

    this->socket->writeDatagram(arr,40,QHostAddress::LocalHost,9999);
    this->time++;
}
