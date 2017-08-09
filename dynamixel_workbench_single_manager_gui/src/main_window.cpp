/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehoon Lim (Darby) */

#include <QtGui>
#include <QMessageBox>
#include <iostream>

#include "../include/dynamixel_workbench_single_manager_gui/main_window.hpp"

using namespace main_window;
using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode_(argc,argv)
{
  ui_.setupUi(this);
  QObject::connect(ui_.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
  setWindowIcon(QIcon(":/images/icon.png"));

  ui_.tab_manager->setCurrentIndex(0);
  QObject::connect(&qnode_, SIGNAL(rosShutdown()), this, SLOT(close()));  

  InitConnect();

  qnode_.init();

  InitUserInterface();
}

MainWindow::~MainWindow() {}

void MainWindow::showNoMasterMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

void MainWindow::on_torque_enable_toggle_button_toggled(bool check)
{
  if(ui_.torque_enable_toggle_button->isChecked())  // Torque ON!!!!
  {
    ui_.torque_enable_toggle_button->setText("Torque_Disable");
    qnode_.sendTorqueMsg(true);

    ui_.set_id_combo_box->setEnabled(false);
    ui_.set_operating_mode_combo_box->setEnabled(false);
    ui_.set_baud_rate_combo_box->setEnabled(false);

    ui_.set_position_zero_push_button->setVisible(false);
    ui_.set_address_value_dial->setEnabled(false);
  }
  else                                              // Torque OFF!!!
  {
    ui_.torque_enable_toggle_button->setText("Torque_Enable");
    qnode_.sendTorqueMsg(false);

    ui_.set_id_combo_box->setEnabled(true);
    ui_.set_operating_mode_combo_box->setEnabled(true);
    ui_.set_baud_rate_combo_box->setEnabled(true);

    ui_.set_position_zero_push_button->setVisible(false);
    ui_.set_address_value_dial->setEnabled(false);
  }

  setAddressComboBox(ui_.torque_enable_toggle_button->isChecked());
}

void MainWindow::on_reboot_push_button_clicked(bool check)
{
  if (!qnode_.sendRebootMsg())
    errorMsg();
  else
    rightMsg();
}

void MainWindow::on_factory_reset_push_button_clicked(bool check)
{
  if (!qnode_.sendResetMsg())
  {
    errorMsg();
  }
  else
  {
    ui_.set_id_combo_box->setCurrentIndex(0);
    ui_.set_baud_rate_combo_box->setCurrentIndex(0);
    ui_.set_operating_mode_combo_box->setCurrentIndex(0);

    if (ui_.torque_enable_toggle_button->isChecked())
        ui_.torque_enable_toggle_button->setChecked(false);

    rightMsg();
  }
}

void MainWindow::on_set_position_zero_push_button_clicked(bool check)
{
  qnode_.setPositionZeroMsg(dynamixel_->value_of_0_radian_position_);
  ui_.set_address_value_spin_box->setValue(dynamixel_->value_of_0_radian_position_);
  ui_.set_address_value_dial->setValue(dynamixel_->value_of_0_radian_position_);
}

void MainWindow::changeID()
{
  if (ui_.set_id_combo_box->currentText().toStdString() != "Select ID")
  {
    if (!qnode_.sendSetIdMsg(ui_.set_id_combo_box->currentText().toInt()))
      errorMsg();
    else
      rightMsg();

    ui_.get_id_line_edit->setText(QString::number(ui_.set_id_combo_box->currentText().toInt()));
  }
}

void MainWindow::changeBaudrate()
{
  if (ui_.set_baud_rate_combo_box->currentText().toStdString() != "Select Baudrate")
  {
    if (!qnode_.sendSetBaudrateMsg(ui_.set_baud_rate_combo_box->currentText().toLongLong()))
      errorMsg();
    else
      rightMsg();

    ui_.get_baud_rate_line_edit->setText(QString::number(ui_.set_baud_rate_combo_box->currentText().toLongLong()));
  }
}

void MainWindow::changeOperatingMode()
{
  if (ui_.set_operating_mode_combo_box->currentText().toStdString() != "Select Mode")
  {
    if (!qnode_.sendSetOperatingModeMsg(ui_.set_operating_mode_combo_box->currentText().toStdString(), dynamixel_info_->lode_info.protocol_version,
                                                                                                       dynamixel_info_->model_name,
                                                                                                       dynamixel_->value_of_max_radian_position_))
      errorMsg();
    else
      rightMsg();
  }
}

void MainWindow::changeControlTableValue()
{
  if (ui_.set_address_name_combo_box->currentText().toStdString() == "torque_enable")
  {
    if (ui_.set_address_value_spin_box->value() == true)
      ui_.torque_enable_toggle_button->setChecked(true);
    else if (ui_.set_address_value_spin_box->value() == false)
      ui_.torque_enable_toggle_button->setChecked(false);
  }

  if (!qnode_.sendAddressValueMsg(ui_.set_address_name_combo_box->currentText().toStdString(), ui_.set_address_value_dial->value()))
    errorMsg();
}

void MainWindow::setEachAddressFunction(QString index)
{
  if (index.toStdString() == "goal_position")
  {
    ui_.set_position_zero_push_button->setVisible(true);
    ui_.set_address_value_dial->setEnabled(true);
    ui_.set_address_value_dial->setRange(dynamixel_->value_of_min_radian_position_, dynamixel_->value_of_max_radian_position_);
    ui_.set_address_value_spin_box->setRange(dynamixel_->value_of_min_radian_position_, dynamixel_->value_of_max_radian_position_);
  }
  else
  {
    ui_.set_position_zero_push_button->setVisible(false);
    ui_.set_address_value_dial->setEnabled(false);
    ui_.set_address_value_dial->setRange(-2147483647, 2147483647);
    ui_.set_address_value_spin_box->setRange(-2147483647, 2147483647);
  }
}

void MainWindow::updateDynamixelInfoLineEdit(dynamixel_workbench_msgs::DynamixelInfo dynamixel_info)
{
  dynamixel_info_ = new DynamixelInfo;

  dynamixel_info_->lode_info.device_name      = dynamixel_info.load_info.device_name;
  dynamixel_info_->lode_info.baud_rate        = dynamixel_info.load_info.baud_rate;
  dynamixel_info_->lode_info.protocol_version = dynamixel_info.load_info.protocol_version;

  dynamixel_info_->model_id     = dynamixel_info.model_id;
  dynamixel_info_->model_number = dynamixel_info.model_number;
  dynamixel_info_->model_name   = dynamixel_info.model_name;

  dynamixel_ = new dynamixel_tool::DynamixelTool(dynamixel_info_->model_id,
                                                 dynamixel_info_->model_number);

  ui_.get_device_name_line_edit->setText(QString::fromStdString(dynamixel_info_->lode_info.device_name));
  ui_.get_id_line_edit->setText(QString::number(dynamixel_info_->model_id));
  ui_.get_baud_rate_line_edit->setText(QString::number(dynamixel_info_->lode_info.baud_rate));
  ui_.get_protocol_version_line_edit->setText(QString::number(dynamixel_info_->lode_info.protocol_version));
  ui_.get_model_name_line_edit->setText(QString::fromStdString(dynamixel_info_->model_name));
}

void MainWindow::InitConnect()
{
  // Init log
  ui_.view_logging->setModel(qnode_.loggingModel());

  // Get Dynamixel Info
  qRegisterMetaType<dynamixel_workbench_msgs::DynamixelInfo>("dynamixel_workbench_msgs::DynamixelInfo");
  QObject::connect(&qnode_, SIGNAL(updateDynamixelInfo(dynamixel_workbench_msgs::DynamixelInfo)), this, SLOT(updateDynamixelInfoLineEdit(dynamixel_workbench_msgs::DynamixelInfo)));

  QObject::connect(ui_.set_id_combo_box,             SIGNAL(currentIndexChanged(int)), this, SLOT(changeID()));
  QObject::connect(ui_.set_baud_rate_combo_box,      SIGNAL(currentIndexChanged(int)), this, SLOT(changeBaudrate()));
  QObject::connect(ui_.set_operating_mode_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(changeOperatingMode()));

  QObject::connect(ui_.set_address_value_spin_box,   SIGNAL(valueChanged(int)), ui_.set_address_value_dial, SLOT(setValue(int)));
  QObject::connect(ui_.set_address_value_dial,       SIGNAL(valueChanged(int)), ui_.set_address_value_spin_box, SLOT(setValue(int)));
  QObject::connect(ui_.set_address_value_spin_box,   SIGNAL(valueChanged(int)), this, SLOT(changeControlTableValue()));

  // Set Address function
  QObject::connect(ui_.set_address_name_combo_box,   SIGNAL(activated(QString)), this, SLOT(setEachAddressFunction(QString)));
}

void MainWindow::setIdComboBox()
{
  int id = 0;

  ui_.set_id_combo_box->addItem((QString("Select ID")));

  for (id = 1; id < 254; id++)
  {
    ui_.set_id_combo_box->addItem(QString::number(id));
  }

  // TODO  : add scroll bar

  //  QAbstractItemView* view = ui_.set_id_combo_box->view();
  //  view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
  //  view->setAutoScroll(true);
  //  view->setFixedHeight(70);
}

void MainWindow::setBaudRateComboBox()
{
  std::vector<uint32_t> baud_rate;

  baud_rate.push_back(19200);
  baud_rate.push_back(57600);
  baud_rate.push_back(115200);
  baud_rate.push_back(200000);
  baud_rate.push_back(250000);
  baud_rate.push_back(400000);
  baud_rate.push_back(500000);
  baud_rate.push_back(1000000);
  baud_rate.push_back(2000000);
  baud_rate.push_back(3000000);
  baud_rate.push_back(4000000);
  baud_rate.push_back(4500000);
  baud_rate.push_back(2500000);
  baud_rate.push_back(2250000);
  baud_rate.push_back(10500000);

  ui_.set_baud_rate_combo_box->addItem((QString("Select Baudrate")));

  for (std::vector<uint32_t>::size_type num = 0; num < baud_rate.size(); ++num)
  {
    if (dynamixel_->baud_rate_table_.find(baud_rate[num])->second != 0)
    {
      ui_.set_baud_rate_combo_box->addItem(QString::number(baud_rate[num]));
    }
  }
}

void MainWindow::setOperatingModeComboBox()
{
  ui_.set_operating_mode_combo_box->addItem((QString("Select Mode")));

  if(dynamixel_info_->model_name.find("AX") != std::string::npos ||
     dynamixel_info_->model_name.find("RX") != std::string::npos ||
     dynamixel_info_->model_name.find("EX") != std::string::npos)
  {
    ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
  }
  else if (dynamixel_info_->model_name.find("MX") != std::string::npos)
  {
    ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("extended_position_control"));
  }
  else if (dynamixel_info_->model_name.find("XL") != std::string::npos)
  {
    ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("extended_position_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("pwm_control"));
  }
  else if (dynamixel_info_->model_name.find("XM") != std::string::npos ||
           dynamixel_info_->model_name.find("XH") != std::string::npos)
  {
    ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("current_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("extended_position_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("position_control_based_on_current"));
    ui_.set_operating_mode_combo_box->addItem(QString("pwm_control"));
  }
  else if (dynamixel_info_->model_name.find("PRO") != std::string::npos)
  {
    if (dynamixel_info_->model_name.find("PRO_L42") != std::string::npos)
    {
      ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
    }
    else
    {
      ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("torque_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("extended_position_control"));
    }
  }
}

void MainWindow::setRebootButton()
{
  if (ui_.get_protocol_version_line_edit->text().toFloat() == 2.0)
  {
    ui_.reboot_push_button->setEnabled(true);
  }
  else
  {
    ui_.reboot_push_button->setEnabled(false);
  }
}

void MainWindow::setAddressComboBox(bool torque_enable)
{
  // Delete all Item on combo box
  uint8_t index_num = ui_.set_address_name_combo_box->count();
  for (uint8_t combo_box_index = 0; combo_box_index < index_num; combo_box_index++)
  {
    ui_.set_address_name_combo_box->removeItem(0);
  }

  // Add item on combo box
  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin();
       dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end();
       dynamixel_->it_ctrl_++)
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[dynamixel_->it_ctrl_->first.c_str()];
    if (torque_enable)
    {
      if ((dynamixel_->item_->access_type == dynamixel_tool::READ_WRITE) && (dynamixel_->item_->memory_type == dynamixel_tool::RAM))
      {
        ui_.set_address_name_combo_box->addItem(QString::fromStdString(dynamixel_->item_->item_name));
      }
    }
    else
    {
      if (dynamixel_->item_->access_type == dynamixel_tool::READ_WRITE)
      {
        ui_.set_address_name_combo_box->addItem(QString::fromStdString(dynamixel_->item_->item_name));
      }
    }
  }
}

void MainWindow::InitUserInterface()
{
  setIdComboBox();
  setBaudRateComboBox();
  setOperatingModeComboBox();
  setRebootButton();
  setAddressComboBox(false);

  ui_.set_address_value_spin_box->setRange(-2048, 2048);
  ui_.set_position_zero_push_button->setVisible(false);
  ui_.set_address_value_dial->setEnabled(false);
}

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(this, tr("About ..."),tr("<h2>Dynamixel workbench 1.0</h2><p>Copyright Robotis</p>"));
}

void MainWindow::errorMsg()
{
  QMessageBox::about(this, tr("Error ..."),tr("<h2>Error!!!!!</h2><p>It didn't works!!</p>"));
}

void MainWindow::rightMsg()
{
  QMessageBox::about(this, tr("Right ..."),tr("<h2>Complete!!</h2><p>It works!!</p>"));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}
