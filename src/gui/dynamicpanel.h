#ifndef _SHIFT_DYNAMICPANEL_HPP_
#define _SHIFT_DYNAMICPANEL_HPP_

#include "shift.h"

#include "openglwindow.h"
#include "renderwindow.h"

namespace Shift {

class DynamicPanel {

  Q_OBJECT

public:
  DynamicPanel(RenderWindow* renderWindow) {
    m_renderWindow = renderWindow;
  }

  ~DynamicPanel() {
    // s2_displacement_t* origin = m_frames.at(0);
    // free(origin);
  }

private:
  RenderWindow* m_renderWindow;

  /*
  BodyPanel;
  JointPanel;
  LightPanel;
  TerrainPanel;
  FramePanel;
  PointCloudPanel;
  */
};
} // namespace Shift

#endif
