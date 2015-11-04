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

    void on_j1_slider_valueChanged(int value);

    void on_j2_slider_valueChanged(int value);

    void on_j3_slider_valueChanged(int value);

    void on_j4_slider_valueChanged(int value);

    void on_j5_slider_valueChanged(int value);

    void on_j6_slider_valueChanged(int value);

private:
    Ui::MainWindow *ui;
    QUdpSocket* socket;
    int time;

    struct UDPMessage{
        int command;
        long time;
        float payload[32];
    };

    UDPMessage joint_message;
    void updateJoint(int index, float value);
};

#endif // MAINWINDOW_H
