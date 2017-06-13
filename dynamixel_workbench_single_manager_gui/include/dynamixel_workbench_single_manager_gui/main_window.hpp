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

#ifndef DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_GUI_MAIN_WINDOW_H
#define DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_GUI_MAIN_WINDOW_H

#ifndef Q_MOC_RUN

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#include "dynamixel_workbench_msgs/DynamixelCommand.h"

#endif

namespace main_window
{
class MainWindow : public QMainWindow
{
 Q_OBJECT

 public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

  void closeEvent(QCloseEvent *event);
	void showNoMasterMessage();

 public Q_SLOTS:
  void on_actionAbout_triggered();
//  void on_torque_enable_toggle_button_toggled(bool check);
//  void on_reboot_push_button_clicked(bool check);
//  void on_factory_reset_push_button_clicked(bool check);
//  void on_set_position_zero_push_button_clicked(bool check);

//  void changeDynamixelID();
//  void changeOperatingMode();
//  void changeBaudrate();
//  void changeControlTableValue();
//  void showHideButton(QString index);

  void updateDynamixelInfoLineEdit(dynamixel_workbench_msgs::DynamixelInfo dynamixel_info);

 private:
  Ui::MainWindowDesign ui_;
  qnode::QNode qnode_;

  bool reboot_button_;
  bool operating_mode_spinbox_;

  void InitUserInterface();
  void InitConnect();
};
}

#endif // DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_GUI_MAIN_WINDOW_H
