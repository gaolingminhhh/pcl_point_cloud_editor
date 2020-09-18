#include <pcl/apps/point_cloud_editor/groundfilterform.h>

GroundFilterForm::GroundFilterForm() : ok_(false)
{
  isBuilding_=new QCheckBox;
  isForest_=new QCheckBox;

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

  layout_->addRow(tr("&是否包含森林"),isForest_);

  layout_->addRow(tr("&是否包含建筑物"),isBuilding_);
  main_layout_ = new QVBoxLayout;
  main_layout_->addLayout(layout_);
  main_layout_->addWidget(button_box_);
  setLayout(main_layout_);
  setWindowTitle(tr("裂缝检测参数设置"));
}

GroundFilterForm::~GroundFilterForm()
{
    delete isBuilding_;
    delete isForest_;
  delete button_box_;
  delete layout_;
  delete main_layout_;
}

void
GroundFilterForm::accept ()
{
  isBuilding=isBuilding_->checkState()==Qt::Checked?true:false;
  isForest=isForest_->checkState()==Qt::Checked?true:false;
  //TODO :判断是否为建筑物和森林的状态
  this->done(0);
  ok_ = true;
}

void
GroundFilterForm::reject ()
{
  ok_ = false;
  this->done(0);
}

