/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Taehoon Lim (Darby) */

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/dynamixel_workbench_single_manager_gui/main_window.hpp"

namespace dynamixel_workbench_single_manager_gui
{
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

  ui_.view_logging->setModel(qnode_.loggingModel());
  QObject::connect(&qnode_, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

  makeConnect();

  qnode_.init();

  makeUI();
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
  if(ui_.torque_enable_toggle_button->isChecked())
  {
    ui_.torque_enable_toggle_button->setText("Torque_Disable");
    qnode_.sendTorqueMsg(std::string("torque_enable"), true);

    ui_.set_id_combo_box->setEnabled(false);
    ui_.set_operating_mode_combo_box->setEnabled(false);
    ui_.set_baud_rate_combo_box->setEnabled(false);
    ui_.set_position_zero_push_button->setVisible(false);
    ui_.set_address_value_dial->setEnabled(false);
  }
  else
  {
    ui_.torque_enable_toggle_button->setText("Torque_Enable");
    qnode_.sendTorqueMsg(std::string("torque_enable"), false);

    ui_.set_id_combo_box->setEnabled(true);
    ui_.set_operating_mode_combo_box->setEnabled(true);
    ui_.set_baud_rate_combo_box->setEnabled(true);
    ui_.set_position_zero_push_button->setVisible(false);
    ui_.set_address_value_dial->setEnabled(false);
  }

  uint8_t index_num = ui_.set_address_name_combo_box->count();
  for (uint8_t combo_box_index = 0; combo_box_index < index_num; combo_box_index++)
  {
    ui_.set_address_name_combo_box->removeItem(0);
  }

  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin(); dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end(); dynamixel_->it_ctrl_++)
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[dynamixel_->it_ctrl_->first.c_str()];
    if (ui_.torque_enable_toggle_button->isChecked())
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

void MainWindow::on_reboot_push_button_clicked(bool check)
{
  qnode_.sendRebootMsg();
}

void MainWindow::on_factory_reset_push_button_clicked(bool check)
{
  qnode_.sendResetMsg();
  sleep(2);

  ui_.get_baud_rate_line_edit->setText(QString::number(57600));
  int index = ui_.set_baud_rate_combo_box->findText("Select Baudrate");

  if ( index != -1 )
  {
     ui_.set_baud_rate_combo_box->setCurrentIndex(index);
  }

  index = ui_.set_id_combo_box->findText("Select ID");

  if ( index != -1 )
  {
     ui_.set_id_combo_box->setCurrentIndex(index);
  }

  ui_.get_id_line_edit->setText(QString::number(1));
}

void MainWindow::on_set_position_zero_push_button_clicked(bool check)
{
  qnode_.setPositionZeroMsg(dynamixel_->value_of_0_radian_position_);
  ui_.set_address_value_spin_box->setValue(dynamixel_->value_of_0_radian_position_);
  ui_.set_address_value_dial->setValue(dynamixel_->value_of_0_radian_position_);
}

void MainWindow::changeDynamixelID()
{
  if (ui_.set_id_combo_box->currentText().toStdString() != "Select ID")
  {
    qnode_.sendSetIdMsg(ui_.set_id_combo_box->currentText().toInt());
    sleep(1);
    ui_.get_id_line_edit->setText(QString::number(ui_.set_id_combo_box->currentText().toInt()));
  }
}

void MainWindow::changeOperatingMode()
{
  if (ui_.set_operating_mode_combo_box->currentText().toStdString() != "Select Mode")
    qnode_.sendSetOperatingModeMsg(ui_.set_operating_mode_combo_box->currentText().toStdString());
}

void MainWindow::changeBaudrate()
{
  if (ui_.set_baud_rate_combo_box->currentText().toStdString() != "Select Baudrate")
  {
    qnode_.sendSetBaudrateMsg(ui_.set_baud_rate_combo_box->currentText().toFloat());
    sleep(2);
    ui_.get_baud_rate_line_edit->setText(QString::number(ui_.set_baud_rate_combo_box->currentText().toLongLong()));
  }
}

void MainWindow::changeControlTableValue()
{
  if (ui_.set_address_name_combo_box->currentText().toStdString() == "torque_enable")
  {
    if (ui_.set_address_value_spin_box->value() == 1)
    {
      ui_.torque_enable_toggle_button->setChecked(true);
    }
    else if ((ui_.set_address_value_spin_box->value() == 0))
    {
      ui_.torque_enable_toggle_button->setChecked(false);
    }
  }

  qnode_.sendControlTableValueMsg(ui_.set_address_name_combo_box->currentText(), ui_.set_address_value_dial->value());
}

void MainWindow::showHideButton(QString index)
{
  if (index.toStdString() == "goal_position")
  {
    ui_.set_position_zero_push_button->setVisible(true);
    ui_.set_address_value_dial->setEnabled(true);
    ui_.set_address_value_dial->setRange(dynamixel_->value_of_min_radian_position_, dynamixel_->value_of_max_radian_position_);
  }
  else
  {
    ui_.set_position_zero_push_button->setVisible(false);
    ui_.set_address_value_dial->setEnabled(false);
    ui_.set_address_value_dial->setRange(-1023, 1023);
  }
}

void MainWindow::updateWorkbenchParamLineEdit(dynamixel_workbench_msgs::WorkbenchParam msg)
{
  ui_.get_device_name_line_edit->setText(QString::fromStdString(msg.device_name));
  ui_.get_id_line_edit->setText(QString::number(msg.model_id));
  ui_.get_baud_rate_line_edit->setText(QString::number(msg.baud_rate));
  ui_.get_protocol_version_line_edit->setText(QString::number(msg.protocol_version));
  ui_.get_model_name_line_edit->setText(QString::fromStdString(msg.model_name));

  dynamixel_ = new dynamixel_tool::DynamixelTool(msg.model_id, msg.model_number, msg.protocol_version);
}

void MainWindow::makeConnect()
{
  qRegisterMetaType<dynamixel_workbench_msgs::WorkbenchParam>("dynamixel_workbench_msgs::WorkbenchParam");
  QObject::connect(&qnode_, SIGNAL(updateWorkbenchParam(dynamixel_workbench_msgs::WorkbenchParam)), this, SLOT(updateWorkbenchParamLineEdit(dynamixel_workbench_msgs::WorkbenchParam)));
  QObject::connect(ui_.set_id_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(changeDynamixelID()));
  QObject::connect(ui_.set_baud_rate_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(changeBaudrate()));
  QObject::connect(ui_.set_operating_mode_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(changeOperatingMode()));
  QObject::connect(ui_.set_address_name_combo_box, SIGNAL(activated(QString)), this, SLOT(showHideButton(QString)));
  QObject::connect(ui_.set_address_value_spin_box, SIGNAL(valueChanged(int)), ui_.set_address_value_dial, SLOT(setValue(int)));
  QObject::connect(ui_.set_address_value_dial, SIGNAL(valueChanged(int)), ui_.set_address_value_spin_box, SLOT(setValue(int)));
  QObject::connect(ui_.set_address_value_spin_box, SIGNAL(valueChanged(int)), this, SLOT(changeControlTableValue()));
  QObject::connect(ui_.set_address_value_dial, SIGNAL(valueChanged(int)), this, SLOT(changeControlTableValue()));
}

void MainWindow::makeUI()
{
  ui_.set_id_combo_box->addItem((QString("Select ID")));
  ui_.set_baud_rate_combo_box->addItem((QString("Select Baudrate")));
  ui_.set_operating_mode_combo_box->addItem((QString("Select Mode")));
  ui_.set_address_value_spin_box->setRange(-1023, 1023);
  ui_.set_position_zero_push_button->setVisible(false);
  ui_.set_address_value_dial->setEnabled(false);

  // Reboot Button Init
  if (ui_.get_protocol_version_line_edit->text().toFloat() == 2.0)
  {
    ui_.reboot_push_button->setEnabled(true);
  }
  else
  {
    ui_.reboot_push_button->setEnabled(false);
  }

  // Address Combobox Init
  for (dynamixel_->it_ctrl_ = dynamixel_->ctrl_table_.begin(); dynamixel_->it_ctrl_ != dynamixel_->ctrl_table_.end(); dynamixel_->it_ctrl_++)
  {
    dynamixel_->item_ = dynamixel_->ctrl_table_[dynamixel_->it_ctrl_->first.c_str()];
    if ((dynamixel_->item_->access_type == dynamixel_tool::READ_WRITE))
    {
      ui_.set_address_name_combo_box->addItem(QString::fromStdString(dynamixel_->item_->item_name));
    }
  }

  // ID Combobox Init
  for (int id = 1; id < 253; id++)
  {
    ui_.set_id_combo_box->addItem(QString::number(id));
  }

  // Operating mode Combobox Init
  if(!strncmp(dynamixel_->model_name_.c_str(), "AX", 2))
  {
    ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "MX", 2))
  {
    if (dynamixel_->model_number_ == 310) // MX-64
    {
      ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("extended_position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("torque_control"));
    }
    else if (dynamixel_->model_number_ == 320) // MX-106
    {
      ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("extended_position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("torque_control"));
    }
    else
    {
      ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("extended_position_control"));
    }
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "RX", 2))
  {
    ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "EX", 2))
  {
    ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "XL", 2))
  {
    ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "XM", 2))
  {
    ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("current_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("extended_position_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("position_control_based_on_current"));
    ui_.set_operating_mode_combo_box->addItem(QString("pwm_control"));
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "PRO", 3))
  {
    if (dynamixel_->model_number_ == 35072) // PRO_L42_10_S300_R
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

  // Baud rate Combobox Init
  if(!strncmp(dynamixel_->model_name_.c_str(), "AX", 2) || !strncmp(dynamixel_->model_name_.c_str(), "RX", 2) || !strncmp(dynamixel_->model_name_.c_str(), "EX", 2))
  {
    ui_.set_baud_rate_combo_box->addItem(QString("1000000"));
    ui_.set_baud_rate_combo_box->addItem(QString("500000"));
    ui_.set_baud_rate_combo_box->addItem(QString("400000"));
    ui_.set_baud_rate_combo_box->addItem(QString("250000"));
    ui_.set_baud_rate_combo_box->addItem(QString("200000"));
    ui_.set_baud_rate_combo_box->addItem(QString("115200"));
    ui_.set_baud_rate_combo_box->addItem(QString("57600"));
    ui_.set_baud_rate_combo_box->addItem(QString("19200"));
    ui_.set_baud_rate_combo_box->addItem(QString("9600"));
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "MX", 2))
  {
    ui_.set_baud_rate_combo_box->addItem(QString("1000000"));
    ui_.set_baud_rate_combo_box->addItem(QString("500000"));
    ui_.set_baud_rate_combo_box->addItem(QString("400000"));
    ui_.set_baud_rate_combo_box->addItem(QString("250000"));
    ui_.set_baud_rate_combo_box->addItem(QString("200000"));
    ui_.set_baud_rate_combo_box->addItem(QString("115200"));
    ui_.set_baud_rate_combo_box->addItem(QString("57600"));
    ui_.set_baud_rate_combo_box->addItem(QString("19200"));
    ui_.set_baud_rate_combo_box->addItem(QString("9600"));
//    ui_.set_baud_rate_combo_box->addItem(QString("2250000"));
//    ui_.set_baud_rate_combo_box->addItem(QString("2500000"));
//    ui_.set_baud_rate_combo_box->addItem(QString("3000000"));
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "XL", 2))
  {
    ui_.set_baud_rate_combo_box->addItem(QString("1000000"));
    ui_.set_baud_rate_combo_box->addItem(QString("115200"));
    ui_.set_baud_rate_combo_box->addItem(QString("57600"));
    ui_.set_baud_rate_combo_box->addItem(QString("9600"));
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "XM", 2))
  {
    ui_.set_baud_rate_combo_box->addItem(QString("9600"));
    ui_.set_baud_rate_combo_box->addItem(QString("57600"));
    ui_.set_baud_rate_combo_box->addItem(QString("115200"));
    ui_.set_baud_rate_combo_box->addItem(QString("1000000"));
    ui_.set_baud_rate_combo_box->addItem(QString("2000000"));
//    ui_.set_baud_rate_combo_box->addItem(QString("3000000"));
//    ui_.set_baud_rate_combo_box->addItem(QString("4000000"));
//    ui_.set_baud_rate_combo_box->addItem(QString("4500000"));
  }
  else if (!strncmp(dynamixel_->model_name_.c_str(), "PRO", 3))
  {
    ui_.set_baud_rate_combo_box->addItem(QString("9600"));
    ui_.set_baud_rate_combo_box->addItem(QString("57600"));
    ui_.set_baud_rate_combo_box->addItem(QString("115200"));
    ui_.set_baud_rate_combo_box->addItem(QString("1000000"));
    ui_.set_baud_rate_combo_box->addItem(QString("2000000"));
//    ui_.set_baud_rate_combo_box->addItem(QString("3000000"));
//    ui_.set_baud_rate_combo_box->addItem(QString("4000000"));
//    ui_.set_baud_rate_combo_box->addItem(QString("4500000"));
//    ui_.set_baud_rate_combo_box->addItem(QString("10500000"));
  }
}

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>Dynamixel workbench 1.0</h2><p>Copyright Robotis</p>"));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}

}
