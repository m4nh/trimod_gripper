#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QUdpSocket>
#include <QDebug>
namespace Ui {



class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_j0_slider_valueChanged(int value);

private:
    Ui::MainWindow *ui;
    QUdpSocket* socket;
    int time;

    struct UDPMessage{
        int command;
        long time;
        float payload[16];
    };
};

#endif // MAINWINDOW_H
