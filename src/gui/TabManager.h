#pragma once

#include <cstddef>
#include <functional>
#include <string>
#include <QObject>
#include <QSet>
#include "gui/Editor.h"
#include "gui/TabWidget.h"

class MainWindow; // for circular dependency

class TabManager : public QObject
{
  Q_OBJECT

public:
  TabManager(MainWindow *o, const QString& filename);

  QWidget *getWidget(); // The TabManager is associated with a widget that is controlling.

  bool refreshDocument(); // returns false if the file could not be opened
  bool shouldClose();
  bool save(EditorInterface *edt);
  bool saveAs(EditorInterface *edt);
  bool saveACopy(EditorInterface *edt);
  void openEditor(const QString& filename);
  size_t count();

public:
  static constexpr const int FIND_HIDDEN = 0;
  static constexpr const int FIND_VISIBLE = 1;
  static constexpr const int FIND_REPLACE_VISIBLE = 2;

signals:
  void tabCountChanged(int);

private:
  MainWindow *par;
  TabWidget *tabWidget;
  EditorInterface *editor;
  QSet<EditorInterface *> editorList;

  bool maybeSave(int);
  bool save(EditorInterface *edt, const QString& path);
  void saveError(const QIODevice& file, const std::string& msg, const QString& filepath);
  void applyAction(QObject *object, const std::function<void(int, EditorInterface *)>& func);

  void createTab(const QString& filename);
  void openTab(const QString& filename);
  void setTabName(const QString& filename, EditorInterface *edt = nullptr);


private slots:
  void tabSwitched(int);
  void closeTabRequested(int);
  void middleMouseClicked(int);

private slots:
  void highlightError(int);
  void unhighlightLastError();
  void undo();
  void redo();
  void cut();
  void paste();
  void indentSelection();
  void unindentSelection();
  void commentSelection();
  void uncommentSelection();
  void updateActionUndoState();
  void toggleBookmark();
  void nextBookmark();
  void prevBookmark();
  void jumpToNextError();
  void copyFileName();
  void copyFilePath();
  void openFolder();
  void closeTab();
  void showContextMenuEvent(const QPoint&);
  void showTabHeaderContextMenu(const QPoint &point);

  void stopAnimation();
  void updateFindState();

  void onHyperlinkIndicatorClicked(int pos);

public slots:
  void actionNew();
  void copy();
  void setContentRenderState(); // since last render

  /// this slot is called when the content of an editor is modified.
  /// so that visual feedback of the status can be updated.
  void setTabModified(EditorInterface *);

  bool saveAll();
  void closeCurrentTab();
  void nextTab();
  void prevTab();
  void setFocus();
};
