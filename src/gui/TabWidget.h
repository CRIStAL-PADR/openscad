#pragma once

#include <QMouseEvent>
#include <QString>
#include <QWidget>
#include <QTabBar>
#include <QTabWidget>
#include <QStackedWidget>
#include <QList>

class TabWidget : public QTabWidget
{
  Q_OBJECT

public:
  TabWidget(QWidget *parent = nullptr);
};
