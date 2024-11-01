/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date August 2024
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/dynamixel_rdk_ctrl/main_window.hpp"

MainWindow::MainWindow(QWidget * parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
}

void MainWindow::closeEvent(QCloseEvent * event)
{
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::on_slider1_valueChanged(int value)
{
  ui->pos1->setText(QString::number(value));
  ui->deg1->setText(QString::number(qnode->pos_to_degree(value)));
  ui->rad1->setText(QString::number(qnode->pos_to_radian(value)));
}

void MainWindow::on_slider2_valueChanged(int value)
{
  ui->pos2->setText(QString::number(value));
  ui->deg2->setText(QString::number(qnode->pos_to_degree(value)));
  ui->rad2->setText(QString::number(qnode->pos_to_radian(value)));
}

void MainWindow::on_slider3_valueChanged(int value)
{
  ui->pos3->setText(QString::number(value));
  ui->deg3->setText(QString::number(qnode->pos_to_degree(value)));
  ui->rad3->setText(QString::number(qnode->pos_to_radian(value)));
}

void MainWindow::on_set_clicked()
{
  int pos1 = qnode->pos_to_radian(ui->slider1->value());
  int pos2 = qnode->pos_to_radian(ui->slider2->value());
  int pos3 = qnode->pos_to_radian(ui->slider3->value());
  qnode->set_position(pos1, pos2, pos3);
}
