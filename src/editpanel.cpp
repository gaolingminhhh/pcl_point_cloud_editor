#include <pcl/apps/point_cloud_editor/editpanel.h>

EditPanel::EditPanel(QString obj1,QString obj2, QWidget *parent)
{
    vbox = new QVBoxLayout;
    buttonGroup=new QButtonGroup(this);
    buttonGroup->setExclusive(false);
    addRadioButton(obj1);
    addRadioButton(obj2);
    ok=new QPushButton(QString("保存"),this);
    cancel=new QPushButton(QString("取消"),this);
    vbox->addWidget(ok);
    vbox->addWidget(cancel);
    setLayout(vbox);
    //connect(buttonGroup,SIGNAL(buttonClicked(int)),this,SLOT(buttonJudge(int)));
    connect(buttonGroup,SIGNAL(buttonToggled(int,bool)),this,SLOT(buttonJudge(int,bool)));
    connect(ok,SIGNAL(clicked()),this,SLOT(save()));
    connect(cancel,SIGNAL(clicked()),this,SLOT(cancelDialog()));
    connect(cancel,SIGNAL(clicked()),this,SLOT(close()));

}

EditPanel::~EditPanel()
{
    delete vbox;
}

void
EditPanel::save()
{
    cloudEditorWidget->SaveEditPointCloud();
}

void
EditPanel::cancelDialog()
{
    cloudEditorWidget->EditCancel();
}

void
EditPanel::addRadioButton(QString name)
{
    QCheckBox *btn=new QCheckBox(name);
    radiobuttons.push_back(btn);
    QString radioStyle=QString("QRadioButton { color: black }");
    btn->setStyleSheet(radioStyle);
    vbox->addWidget(btn);
    vbox->addStretch(1);
    buttonGroup->addButton(btn);
}

void
EditPanel::buttonJudge(int buttonid,bool ischecked)
{
    int id=-2-buttonid;
    this->buttonid=buttonid;
    cloudEditorWidget->changePtr(id,ischecked);
}

void
EditPanel::setCloudEditorWidget(CloudEditorWidget *cloudEditorWidget)
{
    this->cloudEditorWidget=cloudEditorWidget;
}
int
EditPanel::getButtonId()
{

    return buttonid;
}

void EditPanel::closeEvent(QCloseEvent *event)
{
    cloudEditorWidget->EditCancel();
}

