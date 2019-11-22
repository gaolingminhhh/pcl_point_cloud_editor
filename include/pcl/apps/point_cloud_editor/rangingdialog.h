#pragma once
#include <QDialog>
#include <QPushButton>
#include <QLayout>
#include <QLabel>
#include <QWidget>
#include <pcl/apps/point_cloud_editor/cloud.h>

class RangingDialog{
//    Q_OBJECT
public:
    RangingDialog(QLabel *label1,QLabel *label2,QLabel *label3);
    ~RangingDialog();

    void SetDialog1(QString point1);
    void SetDialog2(QString point2);
    void SetResult(QString result);
private:
  //  QVBoxLayout *layout;
    QLabel *point1label;
    QLabel *point2label;
    QLabel *result;
//private slots:
//    void btnClicked();
};
