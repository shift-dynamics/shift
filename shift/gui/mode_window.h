#ifndef SHIFT_GUI_MODE_WINDOW_H_
#define SHIFT_GUI_MODE_WINDOW_H_

#include <QDialog>

#include <functional>
#include <vector>

class QCheckBox;
class QElapsedTimer;
class QLabel;
class QListWidget;
class QPushButton;
class QSlider;
class QTimer;

namespace shift {
namespace core {
namespace body {
class Flex;
}
} // namespace core
namespace gui {

class Simulation;

/*!
@brief the shapes of a flexible body built from a mesh: its static modes
(the six motions of each attachment but the reference frame, the three
translations of each retained point) and the fixed-interface modes
computed, with their frequencies, those kept checked

The selected shape is drawn on the body in the viewport (Simulation::
PreviewShape), its largest nodal displacement the amplitude set by the
slider as a fraction of the body's size, still or moving back and forth.
The vibration modes of a modal body can be checked: each change gives the
modes checked (indices into the modes computed, ascending in frequency) to
changed, and the body panel's Apply writes them as modes_kept with its
other changes. Closing the window draws the state shown again.
*/
class ModeWindow : public QDialog {
public:
  /*! \p checked: the modes shown checked at first (those kept, or a
   * choice not yet applied) */
  ModeWindow(Simulation& simulation, int body, const std::vector<int>& checked,
             std::function<void(const std::vector<int>&)> changed,
             std::function<void()> redraw, QWidget* parent = nullptr);
  ~ModeWindow() override;

  int get_body() const {
    return body_;
  }

protected:
  void done(int result) override;

private:
  /*! \brief draw the selected shape at the amplitude of the slider, times
   * the phase of the animation */
  void Preview();

  Simulation& simulation_;
  int body_;
  const shift::core::body::Flex* flex_ = nullptr;
  std::function<void(const std::vector<int>&)> changed_;
  std::function<void()> redraw_;
  double size_ = 1.0; /*!< the diagonal of the body's undeformed box */
  QListWidget* list_ = nullptr;
  QSlider* amplitude_ = nullptr;
  QLabel* amplitude_label_ = nullptr;
  QCheckBox* animate_ = nullptr;
  QTimer* timer_ = nullptr;
  QElapsedTimer* clock_ = nullptr;
};

} // namespace gui
} // namespace shift

#endif
