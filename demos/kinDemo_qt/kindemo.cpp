#include "kindemo.h"
#include "ui_kindemo.h"

kinDemo::kinDemo(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::kinDemo)
{
    ui->setupUi(this);

    // initial a robot object first
    robot = std::unique_ptr<rb::Artic> {new rb::Artic};

    // Initailize DH-table table widget
    ui->dhTableWidget->setColumnCount(6);
    QStringList dhList = QStringList() << "A" << "Alpha" << "D" << "Theta"<< "Up" << "Low";
    ui->dhTableWidget->setHorizontalHeaderLabels(dhList);
    ui->dhTableWidget->setRowCount(6);
    QStringList jtList = QStringList() << "J1" << "J2" << "J3" << "J4" << "J5" << "J6";
    ui->dhTableWidget->setVerticalHeaderLabels(jtList);
    for(int i=0; i<6; ++i)
        ui->dhTableWidget->setColumnWidth(i, 77);

    // Initialize solution table widget
    ui->solTableWidget->setColumnCount(8);
    QStringList solList = QStringList() << "sol1" << "sol2" << "sol3" << "sol4"<<
                                          "sol5" << "sol6" << "sol7" << "sol8";
    ui->solTableWidget->setHorizontalHeaderLabels(solList);
    ui->solTableWidget->setRowCount(6);
    ui->solTableWidget->setVerticalHeaderLabels(jtList);
    for(int i=0; i<8; ++i)
        ui->solTableWidget->setColumnWidth(i, 92);
}

kinDemo::~kinDemo()
{
    delete ui;
}

void kinDemo::OnBtnClickInit()
{
    robot = std::unique_ptr<rb::Artic> {new rb::Artic};

    // get DH paramter
    Array6d a0 = robot->getA();
    Array6d alpha0 = robot->getAlpha();
    Array6d d0 = robot->getD();
    Array6d th0 = robot->getTheta();
    Array6d up0 = robot->getUpLimit();
    Array6d low0 = robot->getLowLimit();

    for(int i=0; i< 6; ++i)
    {
        QString get_str;
        ui->dhTableWidget->setItem(i, 0, new QTableWidgetItem(get_str.setNum(a0[i])));
        ui->dhTableWidget->setItem(i, 1, new QTableWidgetItem(get_str.setNum(alpha0[i])));
        ui->dhTableWidget->setItem(i, 2, new QTableWidgetItem(get_str.setNum(d0[i])));
        ui->dhTableWidget->setItem(i, 3, new QTableWidgetItem(get_str.setNum(th0[i])));
        ui->dhTableWidget->setItem(i, 4, new QTableWidgetItem(get_str.setNum(up0[i])));
        ui->dhTableWidget->setItem(i, 5, new QTableWidgetItem(get_str.setNum(low0[i])));
    }


    return;
}

void kinDemo::OnBtnClickFK()
{
    Array6d th = Array6d::Constant(0.0);
    QString get_str;

    for(int i=0; i < 6; ++i)
    {
        get_str = ui->dhTableWidget->item(i, 3)->text();
        th[i] = get_str.toDouble();
    }

    pose_tcp = robot->forwardKin(th);

    // output to ui
    get_str.setNum(pose_tcp.x);
    ui->pxEdit->setText(get_str);

    get_str.setNum(pose_tcp.y);
    ui->pyEdit->setText(get_str);

    get_str.setNum(pose_tcp.z);
    ui->pzEdit->setText(get_str);

    get_str.setNum(pose_tcp.a);
    ui->rowEdit->setText(get_str);

    get_str.setNum(pose_tcp.b);
    ui->pitchEdit->setText(get_str);

    get_str.setNum(pose_tcp.c);
    ui->yawEdit->setText(get_str);

    return;
}

void kinDemo::OnBtnClickIK()
{
    QString get_str;

    // get tcp position & orientation from ui
    get_str = ui->pxEdit->text();
    double px = get_str.toDouble();

    get_str = ui->pyEdit->text();
    double py = get_str.toDouble();

    get_str = ui->pzEdit->text();
    double pz = get_str.toDouble();

    get_str = ui->rowEdit->text();
    double row = get_str.toDouble();

    get_str = ui->pitchEdit->text();
    double pitch = get_str.toDouble();

    get_str = ui->yawEdit->text();
    double yaw = get_str.toDouble();

    Array6d q;
    rb::IK_RESULT check = robot->inverseKin(px, py, pz, row, pitch,
                                            yaw, q, joint_value);

    switch(check)
    {
    case rb::IK_RESULT::IK_COMPLETE:
        get_str = "Find Solutions.";
        ui->solCheckLabel->setText(get_str);
        break;
    case rb::IK_RESULT::IK_NO_SOLUTION:
        get_str = "No Solution.";
        ui->solCheckLabel->setText(get_str);
        break;
    case rb::IK_RESULT::IK_ANGLE_LIMIT:
        get_str = "Joint Limit.";
        ui->solCheckLabel->setText(get_str);
        break;
    case rb::IK_RESULT::IK_SINGULAR:
        get_str = "Singular Point Reach!.";
        ui->solCheckLabel->setText(get_str);
        break;
    case rb::IK_RESULT::IK_INPUT_INVALID:
        get_str = "Input Invalid!.";
        ui->solCheckLabel->setText(get_str);
        break;
    }

    // show most fit solution.
    ui->solComboBox->setCurrentIndex(joint_value.fit);

    // output all solutions to ui.
    for(int col=0; col < 8; ++col)
    {
        for(int row=0; row < 6; ++row)
        {
            ui->solTableWidget->setItem(row, col,
                new QTableWidgetItem(get_str.setNum(joint_value.axis_value(col, row))));
        }
    }

    return;
}

void kinDemo::OnBtnClickReset()
{
    // get data from dhTable
    QString get_str;

    Array6d a0 = Array6d::Constant(0.0);
    Array6d alpha0 = Array6d::Constant(0.0);
    Array6d d0 = Array6d::Constant(0.0);
    Array6d th0 = Array6d::Constant(0.0);
    Array6d up0 = Array6d::Constant(0.0);
    Array6d low0 = Array6d::Constant(0.0);

    for(int i=0; i < 6; ++i)
    {
        // get link length A from ui
        get_str = ui->dhTableWidget->item(i, 0)->text();
        a0[i] = get_str.toDouble();

        // get link twist(alpha) from ui
        get_str = ui->dhTableWidget->item(i, 1)->text();
        alpha0[i] = get_str.toDouble();

        // get link offset (d) from ui
        get_str = ui->dhTableWidget->item(i, 2)->text();
        d0[i] = get_str.toDouble();

        // get joint angle (theta) form ui
        get_str = ui->dhTableWidget->item(i, 3)->text();
        th0[i] = get_str.toDouble();

        // get limits from ui
        get_str = ui->dhTableWidget->item(i, 4)->text();
        up0[i] = get_str.toDouble();

        get_str = ui->dhTableWidget->item(i, 5)->text();
        low0[i] = get_str.toDouble();
    }

    robot = std::unique_ptr<rb::Artic> {new rb::Artic(a0, alpha0, d0,
                                                    th0, up0, low0)};

    return;
}
