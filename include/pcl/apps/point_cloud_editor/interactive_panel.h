#ifndef INTERACTIVE_PANEL_H
#define INTERACTIVE_PANEL_H
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
#include <QPushButton>
#include <pcl/registration/icp.h>
#include <pcl/apps/point_cloud_editor/highlightpoints.h>

class Interactive_Panel:public QDialog
{
    Q_OBJECT
public:
    Interactive_Panel(
            CloudPtr cloud_in,
            CloudPtr cloud_out,
            boost::shared_ptr<HightLightPoints> highlight,
            QWidget *parent = nullptr);
    ~Interactive_Panel();

public Q_SLOTS:

private Q_SLOTS:
    void
    ChangeColor(int i);

    void
    InverseColor(int i);

    void
    accept () override;



private:
    CloudPtr cloud_src_ptr;
    CloudPtr cloud_tgt_ptr;

    QCheckBox *checkbox;
    QCheckBox *checkbox2;
    QDialogButtonBox *closeBtn;
    boost::shared_ptr<HightLightPoints> highlight;
    pcl::IterativeClosestPoint<Point3D, Point3D> icp;
    IndexVector hightlightindicies;
    IndexVector hightlightindicies2;
    unsigned int cloudsize=0;
};


#endif // INTERACTIVE_PANEL_H
