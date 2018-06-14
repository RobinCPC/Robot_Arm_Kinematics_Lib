#ifndef KINDEMO_H
#define KINDEMO_H

#include <QDialog>
#include <QString>
#include <QStringList>
#include <QTableWidgetItem>

#include <memory>

#include <rakl.h>

namespace Ui {
class kinDemo;
}

class kinDemo : public QDialog
{
    Q_OBJECT

public:
    explicit kinDemo(QWidget *parent = 0);
    ~kinDemo();

private slots:
    void OnBtnClickInit();
    void OnBtnClickFK();
    void OnBtnClickIK();
    void OnBtnClickReset();

private:
    Ui::kinDemo *ui;
    //RA::rakl* robot;
    std::unique_ptr<RA::rakl> robot;
    RA::ARM_POS pose_tcp;
    RA::ARM_AXIS_VALUE joint_value;
};

#endif // KINDEMO_H
