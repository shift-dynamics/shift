#ifndef _SHIFT_CONTEXT_H_
#define _SHIFT_CONTEXT_H_

#include <json/json.h>
#include <string>

#include "shift-core/assembly.h"
#include "shift-core/base.h"
#include "shift-core/modelparameters.h"

namespace Shift {

/*!
@brief Shift Context datastructure
*/
class Context : Base {

public:
  Context(std::string name)
    : Base(name)
    , modelParameters_(name)
    , fixed_("fixed") {
    initialize();
  }
  Context()
    : Base("")
    , modelParameters_("")
    , fixed_("fixed") {
    initialize();
  }
  ~Context() {}
  void readJSON(const std::string filename);
  void writeJSON();
  void writeJSON(const std::string filename);
  void print();

private:
  ModelParameters modelParameters_;
  Assembly fixed_;
  std::string filename_;
  Json::Value root_;
  void initialize();
};

} // namespace Shift

#endif
