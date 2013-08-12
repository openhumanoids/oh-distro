#ifndef DIALOG_H
#define DIALOG_H

#include <QDialog>
#include <QSignalMapper>
#include <string>

class QDialogButtonBox;
class QFileInfo;
class QTabWidget;

class FirmwareTab : public QWidget
{
  Q_OBJECT
public:
  FirmwareTab(QWidget *parent = 0);
  QSignalMapper *button_mapper;
};

class FunctionalTestTab : public QWidget
{
  Q_OBJECT
public:
  FunctionalTestTab(QWidget *parent = 0);
};

class Dialog : public QDialog
{
  Q_OBJECT
public:
  Dialog(QWidget *parent = 0);
public slots:
  void onFirmwareButton(const QString &board_name);
  void onFirmwareOperationChanged(const QString &operation);
private:
  QTabWidget       *tabWidget;
  QDialogButtonBox *buttonBox;
  std::string firmware_operation_;
};

#endif

