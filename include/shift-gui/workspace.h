#ifndef SHIFT_WORKSPACE_H_
#define SHIFT_WORKSPACE_H_

#include <QtWidgets/QMainWindow>

namespace Ui {
class MainWindow;
}

namespace Shift {

class Workspace : public QMainWindow {
  Q_OBJECT

public:
  explicit Workspace(QWidget *parent = 0);
  ~Workspace();

private:
  Ui::MainWindow *ui;

private slots:
};

} // namespace Shift
#endif
