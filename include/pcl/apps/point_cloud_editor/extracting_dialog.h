#ifndef EXTRACTING_DIALOG_H
#define EXTRACTING_DIALOG_H

#include <pcl/apps/point_cloud_editor/extracting.h>
#include <QLineEdit>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QLabel>
#include <QTimer>
#include <pcl/apps/point_cloud_editor/cloud.h>
#include <QCheckBox>
#include <pcl/apps/point_cloud_editor/localTypes.h>
#include <pcl/apps/point_cloud_editor/extracting.h>
#include <QPushButton>
class Extracting_Dialog:public QDialog
{
    Q_OBJECT
public:
    Extracting_Dialog(Extracting &extracting,
            QWidget *parent = nullptr);
    ~Extracting_Dialog();

    void
    closeEvent(QCloseEvent *event) override;

public Q_SLOTS:

private Q_SLOTS:
    void
    saveFile();

    void
    resetLines();


    void
    accept () override;
private:
    QPushButton *resetButton;
    QPushButton *saveButton;
    QLayout *layout;
    Extracting *extracting;
}

#endif // EXTRACTING_DIALOG_H
