#include <pcl/apps/point_cloud_editor/crackidentityform.h>


#include <pcl/apps/point_cloud_editor/denoiseParameterForm.h>

CrackIdentityForm::CrackIdentityForm () : ok_(false)
{
  k_ = new QLineEdit;
  thresh_ = new QLineEdit;
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
  layout_->addRow(tr("&邻近点数:"), k_);
  layout_->addRow(tr("&阀值:"),
                 thresh_);

  main_layout_ = new QVBoxLayout;
  main_layout_->addLayout(layout_);
  main_layout_->addWidget(button_box_);
  setLayout(main_layout_);
  setWindowTitle(tr("裂缝检测参数设置"));
}

CrackIdentityForm::~CrackIdentityForm()
{
  delete k_;
  delete thresh_;
  delete button_box_;
  delete layout_;
  delete main_layout_;
}

void
CrackIdentityForm::accept ()
{
  QString k__str = k_->text();
  bool ok;
  k = k__str.toInt(&ok);
  // validates the input.
  if (!ok)
  {
    ok_ = false;
    return;
  }
  QString  thresh__str= thresh_->text();
  thresh = thresh__str.toFloat(&ok);
  if (!ok)
  {
    ok_ = false;
    return;
  }
  this->done(0);
  ok_ = true;
}

void
CrackIdentityForm::reject ()
{
  ok_ = false;
  this->done(0);
}

