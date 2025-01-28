#pragma once

#include <QString>
#include <QDockWidget>

class Dock : public QDockWidget
{
  Q_OBJECT

public:
  Dock(QWidget *parent = nullptr);
  void setConfigKey(const QString& configKey);
  void disableSettingsUpdate();

public slots:
  void setVisible(bool visible) override;

private:
  QString configKey;
  bool updateSettings{true};
};
