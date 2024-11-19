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

#include "../include/dynamixel_rdk_manager/main_window.hpp"

MainWindow::MainWindow(QWidget * parent) : QMainWindow(parent), ui(new Ui::MainWindow)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  table_widget_ = ui->tableWidget;
  setup_table();

  qnode = new QNode();

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
  QObject::connect(qnode, SIGNAL(update_data()), this, SLOT(update_data()));
  QObject::connect(qnode, SIGNAL(update_id_list()), this, SLOT(update_id_list()));
}

void MainWindow::setup_table()
{
  QStringList headers = {
    "ID",
    "Type",
    "Torque",
    "Error",
    "Pos [rad]",
    "Vel [rad/s]",
    "Acc [rad/s^2]",
    "Cur [mA]",
    "Volt [V]",
    "Temp [C]",
    "Goal-Pos [rad]",
    "Goal-Vel [rad/s]",
    "Goal-Acc [rad/s^2]"};

  table_widget_->setColumnCount(headers.size());
  table_widget_->setHorizontalHeaderLabels(headers);
  table_widget_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
  table_widget_->setWordWrap(true);
  for (int i = 0; i < table_widget_->columnCount(); ++i) {
    table_widget_->horizontalHeader()->setMinimumSectionSize(50);
    table_widget_->horizontalHeader()->setMaximumSectionSize(150);
  }
  table_widget_->horizontalHeader()->setStyleSheet(
    "QHeaderView::section { "
    "  background-color: #404040; "
    "  color: white; "
    "  padding: 2px; "
    "  border: 1px solid #303030; "
    "  font-weight: bold; "
    "}");
}

void MainWindow::update_data()
{
  const auto & ids = qnode->dynamixel_ids_;
  if (ids.empty()) return;

  if (table_widget_->rowCount() != ids.size()) {
    table_widget_->clearContents();
    table_widget_->setRowCount(ids.size());
  }

  try {
    for (int row = 0; row < ids.size(); ++row) {
      auto createOrUpdateItem = [this](int row, int col, const QString & text) {
        QTableWidgetItem * item = table_widget_->item(row, col);
        if (!item) {
          item = new QTableWidgetItem();
          item->setFlags(item->flags() & ~Qt::ItemIsEditable);
          table_widget_->setItem(row, col, item);
        }
        item->setText(text);
        return item;
      };

      // ID
      createOrUpdateItem(row, 0, QString::number(ids[row]));

      // Type
      createOrUpdateItem(row, 1, QString::fromStdString(qnode->dynamixel_types_[row]));

      // Torque
      auto * torqueItem = createOrUpdateItem(row, 2, qnode->torque_enabled_[row] ? "ON" : "OFF");
      torqueItem->setForeground(qnode->torque_enabled_[row] ? QBrush(Qt::green) : QBrush(Qt::red));

      // Error
      auto * errorItem = createOrUpdateItem(row, 3, QString::number(qnode->error_status_[row]));
      errorItem->setForeground(qnode->error_status_[row] > 0 ? QBrush(Qt::red) : QBrush(Qt::green));

      // Position
      createOrUpdateItem(row, 4, QString::number(qnode->present_positions_[row], 'f', 2));

      // Velocity
      createOrUpdateItem(row, 5, QString::number(qnode->present_velocities_[row], 'f', 2));

      // Acceleration
      createOrUpdateItem(row, 6, QString::number(qnode->present_accelerations_[row], 'f', 2));

      // Current
      createOrUpdateItem(row, 7, QString::number(qnode->present_currents_[row], 'f', 2));

      // Voltage
      createOrUpdateItem(row, 8, QString::number(qnode->present_voltages_[row], 'f', 2));

      // Temperature
      auto * tempItem =
        createOrUpdateItem(row, 9, QString::number(qnode->present_temperatures_[row], 'f', 1));
      if (qnode->present_temperatures_[row] > 55.0) {
        tempItem->setForeground(QBrush(Qt::red));
      } else {
        tempItem->setForeground(QBrush(Qt::white));
      }

      // Goal Position
      createOrUpdateItem(row, 10, QString::number(qnode->goal_positions_[row], 'f', 2));

      // Goal Velocity
      createOrUpdateItem(row, 11, QString::number(qnode->goal_velocities_[row], 'f', 2));

      // Goal Acceleration
      createOrUpdateItem(row, 12, QString::number(qnode->goal_accelerations_[row], 'f', 2));
    }

    table_widget_->resizeColumnsToContents();

    for (int i = 0; i < table_widget_->columnCount(); ++i) {
      int width = table_widget_->columnWidth(i);
      table_widget_->setColumnWidth(i, qBound(50, width, 150));
    }

  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("MainWindow"), "Error updating table: %s", e.what());
  }
}

void MainWindow::update_id_list()
{
  ui->goal_pos_slider->setRange(
    qnode->min_max_positions_[0].first * 100, qnode->min_max_positions_[0].second * 100);

  RCLCPP_INFO(
    rclcpp::get_logger("MainWindow"), "Min: %f, Max: %f", qnode->min_max_positions_[0].first,
    qnode->min_max_positions_[0].second);

  ui->idList->clear();
  for (const auto id : qnode->dynamixel_ids_) {
    ui->idList->addItem(QString::number(id));
  }
  ui->goal_pos->setText(QString::number(qnode->goal_positions_[0], 'f', 2));
  ui->goal_vel->setText(QString::number(qnode->goal_velocities_[0], 'f', 2));
  ui->goal_acc->setText(QString::number(qnode->goal_accelerations_[0], 'f', 2));
  ui->goal_pos_slider->setValue(qnode->goal_positions_[0] * 100);
  ui->goal_vel_slider->setValue(qnode->goal_velocities_[0] * 100);
  ui->goal_acc_slider->setValue(qnode->goal_accelerations_[0] * 100);
}

void MainWindow::on_idList_currentIndexChanged(int index)
{
  ui->goal_pos_slider->setValue(qnode->goal_positions_[index] * 100);
  ui->goal_vel_slider->setValue(qnode->goal_velocities_[index] * 100);
  ui->goal_acc_slider->setValue(qnode->goal_accelerations_[index] * 100);
  ui->goal_pos->setText(QString::number(qnode->goal_positions_[index], 'f', 2));
  ui->goal_vel->setText(QString::number(qnode->goal_velocities_[index], 'f', 2));
  ui->goal_acc->setText(QString::number(qnode->goal_accelerations_[index], 'f', 2));
}

void MainWindow::on_goal_pos_slider_valueChanged(int value)
{
  qnode->goal_positions_[ui->idList->currentIndex()] = value / 100.0;
  ui->goal_pos->setText(
    QString::number(qnode->goal_positions_[ui->idList->currentIndex()], 'f', 2));
}

void MainWindow::on_goal_vel_slider_valueChanged(int value)
{
  qnode->goal_velocities_[ui->idList->currentIndex()] = value / 100.0;
  ui->goal_vel->setText(
    QString::number(qnode->goal_velocities_[ui->idList->currentIndex()], 'f', 2));
}

void MainWindow::on_goal_acc_slider_valueChanged(int value)
{
  qnode->goal_accelerations_[ui->idList->currentIndex()] = value / 100.0;
  ui->goal_acc->setText(
    QString::number(qnode->goal_accelerations_[ui->idList->currentIndex()], 'f', 2));
}

void MainWindow::on_pub_btn_clicked()
{
  qnode->control_publish();
}

void MainWindow::closeEvent(QCloseEvent * event)
{
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete ui;
}
