#ifndef _SHIFT_BASEPANEL_HPP_
#define _SHIFT_BASEPANEL_HPP_

#include "shift.h"

#include "openglwindow.h"
#include "renderwindow.h"

namespace Shift {

class BasePanel {

  Q_OBJECT

public:
  BasePanel(RenderWindow* renderWindow) {
    m_renderWindow = renderWindow;
  }

  ~BasePanel() {}

private:
  RenderWindow* m_renderWindow;
};
} // namespace Shift

#endif
