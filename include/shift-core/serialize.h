#ifndef _SHIFT_SERIALIZE_H_
#define _SHIFT_SERIALIZE_H_

#include <string>
#include <vector>

namespace Shift {

enum class SerialType {
  STRING,
  STRING_ARRAY,
  DOUBLE,
  DOUBLE_ARRAY,
  KSL_VEC3,
  KSL_VEC4,
  KSL_QUATERNION,
  KSL_SCREW,
  KSL_COSCREW,
  KSL_INERTIA,
  POSE,
  BOOLEAN
};

class SerialEntry {

public:
  SerialEntry(std::string keyword, SerialType serialType, void* data,
              int rowDim = 1, int colDim = 1) {
    keyword_ = keyword;
    serialType_ = serialType;
    data_ = data;
    rowDim_ = rowDim;
    colDim_ = colDim;
  };
  ~SerialEntry(){};

private:
  std::string keyword_;
  SerialType serialType_;
  void* data_ = NULL;
  int rowDim_ = 1;
  int colDim_ = 1;
};

/*!
@brief Shift Serialize class
*/
class Serialize {

public:
  Serialize() {}
  ~Serialize() {}

  void addSerialEntry(const std::string keyword, const SerialType serialType,
                      void* data, int rowDim = 1, int colDim = 1);

private:
  std::vector<SerialEntry> serialEntries_;
};

}; // namespace Shift

#endif
