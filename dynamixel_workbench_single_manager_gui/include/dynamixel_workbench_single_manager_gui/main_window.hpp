/**
 * @file /include/dynamixel_workbench_single_manager_gui/main_window.hpp
 *
 * @brief Qt based gui for dynamixel_workbench_single_manager_gui.
 *
 * @date November 2010
 **/
#ifndef dynamixel_workbench_single_manager_gui_MAIN_WINDOW_H
#define dynamixel_workbench_single_manager_gui_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN

#include <QtGui/QMainWindow>
#include "ui_main_window.h"
#include "qnode.hpp"

#include "dynamixel_workbench_toolbox/dynamixel_tool.h"

#endif
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace dynamixel_workbench_single_manager_gui {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

        //void ReadSettings(); // Load up qt program settings at startup
        //void WriteSettings(); // Save qt program settings when closing

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();

public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
	*******************************************/
        void on_actionAbout_triggered();
        void on_torque_enable_toggle_button_toggled(bool check);
        void on_reboot_push_button_clicked(bool check);
        void on_factory_reset_push_button_clicked(bool check);
        void on_set_position_zero_push_button_clicked(bool check);

        void changeDynamixelID();
        void changeOperatingMode();
        void changeBaudrate();
        void changeControlTableValue();
        void showHideButton(QString index);

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView(); // no idea why this can't connect automatically
    void updateWorkbenchParamLineEdit(dynamixel_workbench_msgs::WorkbenchParam msg);

private:
        Ui::MainWindowDesign ui_;
        QNode qnode_;
        dynamixel_tool::DynamixelTool * dxl_;

        bool reboot_button_;
        bool operating_mode_spinbox_;
        void makeUI();
        void makeConnect();
};

}  // namespace dynamixel_workbench_single_manager_gui

#endif // dynamixel_workbench_single_manager_gui_MAIN_WINDOW_H
