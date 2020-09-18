#ifndef CRACKIDENTITYFORM_H
#define CRACKIDENTITYFORM_H

#include <QLineEdit>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QLineEdit>
#include <QCheckBox>

class CrackIdentityForm : public QDialog
{
  Q_OBJECT

  public:
    /// @brief Default Constructor
    CrackIdentityForm();

    /// @brief Destructor
    ~CrackIdentityForm ();

    /// @brief Returns the mean
    inline
    int
    getknumbersneighborofcolor () const
    {
      return (knumbersneighborofcolor);
    }

    /// @brief Returns the standard deviation multiplier threshold
    inline
    float
    getknumbersneighborofedge () const
    {
      return (knumbersneighborofedge);
    }

//    inline
//    bool
//    getisbuilding() const
//    {
//      return (isBuilding);
//    }
//    inline
//    bool
//    getisforest() const
//    {
//      return (isForest);
//    }

    /// @brief Checks whether the OK button was pressed.
    inline
    bool
    ok () const
    {
      return (ok_);
    }

  private Q_SLOTS:
    /// @brief Accepts and stores the current user inputs, and turns off the
    /// dialog box.
    void
    accept () override;

    /// @brief Rejects the current inputs, and turns off the dialog box.
    void
    reject () override;

  private:
    /// The line for entering the mean
    QLineEdit *knumbersneighborofcolor_;
    /// The line for entering the standard deviation multiplier threshold
    QLineEdit *knumbersneighborofedge_;

//    QCheckBox *isBuilding_;
//    QCheckBox *isForest_;
    /// The button box.
    QDialogButtonBox *button_box_;


    /// The layout of the two input QLineEdit objects
    QFormLayout *layout_;
    /// The main layout for the dialog
    QVBoxLayout* main_layout_;
    /// The mean
     int knumbersneighborofcolor;
    /// The standard deviation multiplier threshold
    int knumbersneighborofedge;
    /// The flag indicating whether the OK button was pressed
    bool ok_;

    bool isBuilding;

    bool isForest;
};


#endif // CRACKIDENTITYFORM_H
