#include <pcl/apps/point_cloud_editor/crackidentityform.h>


#include <pcl/apps/point_cloud_editor/denoiseParameterForm.h>

CrackIdentityForm::CrackIdentityForm () : ok_(false)
{
    knumbersneighborofcolor_ = new QLineEdit;
    knumbersneighborofedge_ = new QLineEdit;
    //  isBuilding_=new QCheckBox;
    //  isForest_=new QCheckBox;

    button_box_ = new QDialogButtonBox;
    button_box_->addButton(tr("取消"),
                           QDialogButtonBox::RejectRole);
    button_box_->addButton(tr("好"),
                           QDialogButtonBox::AcceptRole);
    connect(button_box_, SIGNAL(accepted()),
            this, SLOT(accept()));
    connect(button_box_, SIGNAL(rejected()),
            this, SLOT(reject()));
    layout_ = new QFormLayout;
    layout_->addRow(tr("&颜色邻近点数:"), knumbersneighborofcolor_);
    layout_->addRow(tr("&边缘邻近点数:"),
                    knumbersneighborofedge_);
    //  layout_->addRow(tr("&是否包含建筑物"),isBuilding_);
    //  layout_->addRow(tr("&是否包含森林"),isForest_);

    main_layout_ = new QVBoxLayout;
    main_layout_->addLayout(layout_);
    main_layout_->addWidget(button_box_);
    setLayout(main_layout_);
    setWindowTitle(tr("裂缝检测参数设置"));
}

CrackIdentityForm::~CrackIdentityForm()
{
    //    delete isBuilding_;
    //    delete isForest_;
    delete knumbersneighborofcolor_;
    delete knumbersneighborofedge_;
    delete button_box_;
    delete layout_;
    delete main_layout_;
}

void
CrackIdentityForm::accept ()
{
    QString knumbersneighborofcolor_str = knumbersneighborofcolor_->text();
    bool ok;
    knumbersneighborofcolor = knumbersneighborofcolor_str.toInt(&ok);
    // validates the input.
    if (!ok)
    {
        ok_ = false;
        return;
    }
    QString  knumbersneighborofedge_str= knumbersneighborofedge_->text();
    knumbersneighborofedge = knumbersneighborofedge_str.toInt(&ok);
    if (!ok)
    {
        ok_ = false;
        return;
    }
    //TODO :判断是否为建筑物和森林的状态
    this->done(0);
    ok_ = true;
}

void
CrackIdentityForm::reject ()
{
    ok_ = false;
    this->done(0);
}

