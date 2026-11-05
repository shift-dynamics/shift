#ifndef SHIFT_LIGHT_IES_H_
#define SHIFT_LIGHT_IES_H_

#include <string>
#include <vector>

#include "shift/core/factory.h"
#include "shift/core/light/base.h"

namespace shift {
namespace core {
namespace light {

/*!
@brief A light with the distribution of a real luminaire, from an IES
photometric file (IES LM-63, type C photometry)

The file gives the luminous intensity (candela) of the luminaire over a
grid of vertical angles (0 straight down, the nadir, to 180 straight up) and
horizontal angles (0 to 360 around the nadir). The nadir is the -z axis of
the light's frame, horizontal angle 0 its x axis and 90 its y axis.

The distribution is normalized by its peak: the light is brightest, Kd times
intensity, in the direction of the peak. For a physical light give
attenuation [0, 0, 1] and intensity the peak candela (get_peak_candela())
divided by the brightness that should show as white.

\\code
"lights": [{"type": "ies", "name": "flood", "parent": ["fixed", "pole"],
            "filename": "flood.ies", "Kd": [1, 1, 1]}]
\\endcode
*/
class IES : public Base {
public:
  IES()
    : Base() {
    DeclareParameters();
  }
  IES(std::string name)
    : Base(name) {
    DeclareParameters();
  }
  virtual ~IES() override {}
  static std::unique_ptr<IES> Create();
  static bool is_registered() {
    return registered_;
  }

  void Load() override;

  /*! \brief reads an IES LM-63 file, replacing the distribution; throws
   * shift::core::Exception if it can't */
  void Read(const std::string& path);

  /*! \brief luminous intensity in the direction at vertical angle (from the
   * nadir, the -z axis of the light's frame) and horizontal angle (from its
   * x axis towards y), both in radians, divided by the peak intensity */
  double Distribution(double vertical, double horizontal) const;

  /*! \brief largest luminous intensity of the file (candela), after its
   * candela multiplier */
  double get_peak_candela() const {
    return peak_;
  }

private:
  virtual void DeclareParameters() override;
  /*! \brief path of a file next to the model file or in an include
   * directory */
  std::string FindFile(const std::string& name) const;

  std::string filename_;
  /* the distribution as in the file: angles in degrees, increasing; candela
  for each horizontal angle, over the vertical angles */
  std::vector<double> vertical_;
  std::vector<double> horizontal_;
  std::vector<double> candela_; // horizontal_.size() x vertical_.size()
  double peak_ = 0.0;
  static bool registered_;
};

} // namespace light
}
} // namespace shift

#endif
