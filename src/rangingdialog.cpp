#include <pcl/apps/point_cloud_editor/rangingdialog.h>
#include <QObject>
#include <pcl/apps/point_cloud_editor/mainWindow.h>

RangingDialog::RangingDialog(QLabel *label1,QLabel *label2,QLabel *label3){
    point1label=label1;
   point2label=label2;
    result=label3;
//    button=new QPushButton(dialog);
//    point1label->setStyleSheet("color: white");
//    point2label->setStyleSheet("color: white");
//    result->setStyleSheet("color: white");

//    point1label->setAlignment(Qt::AlignLeft|Qt::AlignBottom);
 //   point1label->setAlignment(Qt::AlignCenter|Qt::AlignBottom);
  //  result->setAlignment(Qt::AlignRight|Qt::AlignBottom);

 //   layout->addWidget(button);
 //   dialog->setLayout(layout);
 //   connect(button,SIGNAL(clicked()),this,SLOT(btnClicked()));
//   QLayout *layout=new QLayout;
//    layout->addWidget(point1label);
//    layout->addWidget(point2label);
//    layout->addWidget(result);
    point1label->show();
//    point1label->setGeometry(0,0,200,35);
//    point2label->setGeometry(0,30,200,35);
//    result->setGeometry(0,60,200,35);
    point2label->show();
    result->show();
}

RangingDialog::~RangingDialog(){}


//void
//RangingDialog::btnClicked()
//{
//    //dialog->close();
//}

void
RangingDialog::SetDialog1(QString point1)
{
    point1label->clear();
    point2label->clear();
    result->clear();
 //   point1label->update();
  //  point2label->update();
   // result->update();
    point1label->setText(point1);
}

void
RangingDialog::SetDialog2(QString point2)
{
    point2label->setText(point2);
}

void
RangingDialog::SetResult(QString resultstr)
{
    result->setText(resultstr);
}
