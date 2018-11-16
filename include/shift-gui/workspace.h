#ifndef SHIFT_WORKSPACE_H_
#define SHIFT_WORKSPACE_H_

#include <QtWidgets/QMainWindow>

namespace Ui {
class MainWindow;
}

class VulkanWindow;

namespace Shift {

class Workspace : public QMainWindow {
  Q_OBJECT

public:
  explicit Workspace(VulkanWindow* vulkanWindow, QWidget* parent = 0);
  ~Workspace();

private:
  Ui::MainWindow* ui_;

private slots:
};

} // namespace Shift
#endif
