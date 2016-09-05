/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/dynamixel_workbench_single_manager_gui/main_window.hpp"

/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace dynamixel_workbench_single_manager_gui {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
    , qnode_(argc,argv)
{
  ui_.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
  QObject::connect(ui_.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

  setWindowIcon(QIcon(":/images/icon.png"));
  ui_.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
  QObject::connect(&qnode_, SIGNAL(rosShutdown()), this, SLOT(close()));

  /*********************
  ** Logging
  **********************/
  ui_.view_logging->setModel(qnode_.loggingModel());
  QObject::connect(&qnode_, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  /*********************
  ** Connect
  **********************/
  makeConnect();

  /*********************
  ** Auto Start
  **********************/
  qnode_.init();
  qnode_.getWorkbenchParam();

  makeUI();
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

void MainWindow::on_torque_enable_toggle_button_toggled(bool check)
{
  if(ui_.torque_enable_toggle_button->isChecked())
  {
    ui_.torque_enable_toggle_button->setText("Torque_Disable");
    qnode_.sendTorqueMsg(std::string("torque_enable"), true);

    ui_.id_spin_box->setEnabled(false);
    ui_.operating_mode_combo_box->setEnabled(false);
    ui_.set_baud_rate_line_edit->setEnabled(false);
  }
  else
  {
    ui_.torque_enable_toggle_button->setText("Torque_Enable");
    qnode_.sendTorqueMsg(std::string("torque_enable"), false);

    ui_.id_spin_box->setEnabled(true);
    ui_.operating_mode_combo_box->setEnabled(true);
    ui_.set_baud_rate_line_edit->setEnabled(true);
  }
}

void MainWindow::on_reboot_push_button_clicked(bool check)
{
  qnode_.sendRebootMsg();
}

void MainWindow::on_factory_reset_push_button_clicked(bool check)
{
  qnode_.sendResetMsg();
}

void MainWindow::changeDynamixelID()
{
  qnode_.sendSetIdMsg(ui_.id_spin_box->value());
}

void MainWindow::changeOperatingMode()
{
  qnode_.sendSetOperatingModeMsg(ui_.operating_mode_combo_box->currentIndex());
}

void MainWindow::changeBaudrate()
{
  qnode_.sendSetBaudrateMsg(ui_.set_baud_rate_line_edit->text().toFloat());
}

void MainWindow::changeControlTableValue()
{
  if (strncmp(ui_.address_name_line_edit->text().toStdString().c_str(), "Type", 4))
  {
    qnode_.sendControlTableValueMsg(ui_.address_name_line_edit->text().toStdString(), ui_.address_value_dial->value());
  }
}

void MainWindow::updateWorkbenchParamLineEdit(dynamixel_workbench_msgs::WorkbenchParam msg)
{
  ui_.device_name_line_edit->setText(QString::fromStdString(msg.device_name));
  ui_.get_baud_rate_line_edit->setText(QString::number(msg.baud_rate));
  ui_.protocol_version_line_edit->setText(QString::number(msg.protocol_version));
  ui_.model_name_line_edit->setText(QString::fromStdString(msg.model_name));
}

void MainWindow::makeConnect()
{
  qRegisterMetaType<dynamixel_workbench_msgs::WorkbenchParam>("dynamixel_workbench_msgs::WorkbenchParam");
  QObject::connect(&qnode_, SIGNAL(updateWorkbenchParam(dynamixel_workbench_msgs::WorkbenchParam)), this, SLOT(updateWorkbenchParamLineEdit(dynamixel_workbench_msgs::WorkbenchParam)));
  QObject::connect(ui_.id_spin_box, SIGNAL(valueChanged(int)), this, SLOT(changeDynamixelID()));
  QObject::connect(ui_.operating_mode_combo_box, SIGNAL(valueChanged(int)), this, SLOT(changeOperatingMode()));
  QObject::connect(ui_.set_baud_rate_line_edit, SIGNAL(returnPressed()), this, SLOT(changeBaudrate()));
  QObject::connect(ui_.operating_mode_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(changeOperatingMode()));
  QObject::connect(ui_.address_value_spin_box, SIGNAL(valueChanged(int)), ui_.address_value_dial, SLOT(setValue(int)));
  QObject::connect(ui_.address_value_dial, SIGNAL(valueChanged(int)), ui_.address_value_spin_box, SLOT(setValue(int)));
  QObject::connect(ui_.address_value_spin_box, SIGNAL(valueChanged(int)), this, SLOT(changeControlTableValue()));
  QObject::connect(ui_.address_value_dial, SIGNAL(valueChanged(int)), this, SLOT(changeControlTableValue()));
}

void MainWindow::makeUI()
{
  ui_.address_name_line_edit->setText("Type Address");
  ui_.id_spin_box->setValue(1);
  ui_.set_baud_rate_line_edit->setText(QString::number(1000000));

  if (reboot_button_)
  {
    ui_.reboot_push_button->setEnabled(true);
  }
  else
  {
    ui_.reboot_push_button->setEnabled(false);
  }

  if (ui_.protocol_version_line_edit->text().toFloat() == 2.0)
  {
    reboot_button_ = true;
  }
  else
  {
    reboot_button_ = false;
  }

  if (!strncmp(ui_.model_name_line_edit->text().toStdString().c_str(), "XM", 2))
  {
    ui_.address_value_dial->setRange(0, 4096);
    ui_.address_value_spin_box->setRange(0,4096);
  }
  else
  {
    ui_.address_value_dial->setRange(0, 1000);
    ui_.address_value_spin_box->setRange(0,1000);
  }
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
       // ui_.view_logging->scrollToTop();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<p>Copyright Robotis</p>"));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    //WriteSettings();
	QMainWindow::closeEvent(event);
}

}  // namespace dynamixel_workbench_single_manager_gui

