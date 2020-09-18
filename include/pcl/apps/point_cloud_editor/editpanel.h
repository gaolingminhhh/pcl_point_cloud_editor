#ifndef EDITPANEL_H
#define EDITPANEL_H

#include <QLineEdit>
#include <QDialog>
#include <QDialogButtonBox>
#include <QVBoxLayout>
#include <QDebug>
#include <pcl/apps/point_cloud_editor/cloud.h>
#include <QCheckBox>
#include <QRadioButton>
#include <QGroupBox>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <vector>
#include <QButtonGroup>
#include <QPushButton>
#include <pcl/apps/point_cloud_editor/cloudEditorWidget.h>
#include <QString>
class EditPanel:public QDialog
{
    Q_OBJECT
public:
    EditPanel(QString obj1,QString obj2,
            QWidget *parent = nullptr);
    ~EditPanel();
    void
    addRadioButton(QString name);
    int
    getButtonId();

    void
    setCloudEditorWidget(CloudEditorWidget *cloudEditorWidget);

public Q_SLOTS:

protected:
    void closeEvent(QCloseEvent*event);

private Q_SLOTS:
    void buttonJudge(int buttonid,bool ischecked);
    void save();
    void cancelDialog();

    //    void
    //    accept () override;



private:
    std::vector<QCheckBox*> radiobuttons;
    QVBoxLayout *vbox;
    QButtonGroup *buttonGroup;
    QPushButton *ok,*cancel;
    int buttonid=-1;
    CloudEditorWidget *cloudEditorWidget;

};

#endif // EDITPANEL_H
