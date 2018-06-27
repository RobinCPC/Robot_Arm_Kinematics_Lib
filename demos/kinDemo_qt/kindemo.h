#ifndef KINDEMO_H
#define KINDEMO_H

#include <QDialog>
#include <QString>
#include <QStringList>
#include <QTableWidgetItem>

#include <memory>

#include <kin/artic.h>

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
    std::unique_ptr<rb::kin::Artic> robot;
    rb::kin::ArmPose pose_tcp;
    rb::kin::ArmAxisValue joint_value;
};

#endif // KINDEMO_H
