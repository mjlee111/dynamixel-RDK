#include "../include/dynamixel_rdk_ctrl/main_window.hpp"

#include <QApplication>
#include <iostream>

int main(int argc, char * argv[])
{
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}
