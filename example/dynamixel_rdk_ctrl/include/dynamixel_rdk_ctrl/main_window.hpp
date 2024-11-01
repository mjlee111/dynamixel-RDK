/**
 * @file /include/dynamixel_rdk_ctrl/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date August 2024
 **/

#ifndef dynamixel_rdk_ctrl_MAIN_WINDOW_H
#define dynamixel_rdk_ctrl_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"

#include <QMainWindow>

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget * parent = nullptr);
  ~MainWindow();
  QNode * qnode;

private slots:
  void on_slider1_valueChanged(int value);
  void on_slider2_valueChanged(int value);
  void on_slider3_valueChanged(int value);

  void on_set_clicked();

private:
  Ui::MainWindowDesign * ui;
  void closeEvent(QCloseEvent * event);
};

#endif  // dynamixel_rdk_ctrl_MAIN_WINDOW_H
