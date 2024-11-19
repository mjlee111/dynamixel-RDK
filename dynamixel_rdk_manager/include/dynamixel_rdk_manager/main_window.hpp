/*
 * Copyright 2024 Myeong Jin Lee
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
 */

#ifndef dynamixel_rdk_manager_MAIN_WINDOW_H
#define dynamixel_rdk_manager_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"

#include <rclcpp/rclcpp.hpp>

#include <QAbstractTableModel>
#include <QLayoutItem>
#include <QMainWindow>
#include <QTableWidget>
#include <QVariant>
#include <string>
#include <thread>
#include <vector>

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:  // Public Functions
  MainWindow(QWidget * parent = nullptr);
  ~MainWindow();

  void setup_table();

public:  // Public Variables
  QNode * qnode;

public Q_SLOTS:
  void update_data();
  void update_id_list();

  void on_idList_currentIndexChanged(int index);
  void on_goal_pos_slider_valueChanged(int value);
  void on_goal_vel_slider_valueChanged(int value);
  void on_goal_acc_slider_valueChanged(int value);
  void on_pub_btn_clicked();

private:
  Ui::MainWindow * ui;
  void closeEvent(QCloseEvent * event);
  QTableWidget * table_widget_;
};

#endif  // dynamixel_rdk_manager_MAIN_WINDOW_H
