#include <QApplication>
#include "dialog.h"

int main(int argc, char **argv)
{
  QApplication app(argc, argv);
  Dialog dialog;
  dialog.show();
  return app.exec();
  return 0;
}
