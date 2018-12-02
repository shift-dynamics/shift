#ifndef _SHIFT_SERIALIZE_H_
#define _SHIFT_SERIALIZE_H_

#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "ksl/coscrew.h"
#include "ksl/inertia.h"
#include "ksl/quaternion.h"
#include "ksl/screw.h"
#include "ksl/vec3.h"

namespace Shift {

enum class SerialType {
  STRING,
  STRING_ARRAY,
  STRING_2DARRAY,
  DOUBLE,
  DOUBLE_ARRAY,
  KSL_VEC3,
  KSL_VEC4,
  KSL_QUATERNION,
  KSL_SCREW,
  KSL_COSCREW,
  KSL_INERTIA,
  KSL_MAT3X3,
  KSL_MAT4X4,
  EULERANGLES,
  BOOLEAN
};

class SerialEntry {

public:
  SerialEntry(const std::string keyword, const SerialType serialType,
              void* data, const int rowDim = 1, const int colDim = 1) {
    keyword_ = keyword;
    type_ = serialType;
    data_ = data;
    rowDim_ = rowDim;
    colDim_ = colDim;
  };
  ~SerialEntry() {
    switch(type_) {
      case SerialType::STRING: {
        std::string* data = static_cast<std::string*>(defaultData_);
        if(data) {
          delete data;
        }
        break;
      }
      case SerialType::STRING_ARRAY: {
        std::vector<std::string>* data =
          static_cast<std::vector<std::string>*>(defaultData_);
        if(data) {
          delete data;
        }
        break;
      }
      case SerialType::STRING_2DARRAY: {
        std::vector<std::vector<std::string>>* data =
          static_cast<std::vector<std::vector<std::string>>*>(defaultData_);
        if(data) {
          delete data;
        }
        break;
      }
      case SerialType::DOUBLE:
      case SerialType::DOUBLE_ARRAY: {
        double* data = static_cast<double*>(defaultData_);
        if(data) {
          free(data);
        }
        break;
      }
      case SerialType::KSL_VEC3: {
        ksl_vec3_t* data = static_cast<ksl_vec3_t*>(defaultData_);
        if(data) {
          free(data);
        }
        break;
      }
      case SerialType::KSL_VEC4: {
        ksl_vec4_t* data = static_cast<ksl_vec4_t*>(defaultData_);
        if(data) {
          free(data);
        }
        break;
      }
      case SerialType::KSL_QUATERNION: {
        ksl_quaternion_t* data = static_cast<ksl_quaternion_t*>(defaultData_);
        if(data) {
          free(data);
        }
        break;
      }
      case SerialType::KSL_SCREW: {
        ksl_screw_t* data = static_cast<ksl_screw_t*>(defaultData_);
        if(data) {
          free(data);
        }
        break;
      }
      case SerialType::KSL_COSCREW: {
        ksl_coscrew_t* data = static_cast<ksl_coscrew_t*>(defaultData_);
        if(data) {
          free(data);
        }
        break;
      }
      case SerialType::KSL_INERTIA: {
        ksl_inertia_t* data = static_cast<ksl_inertia_t*>(defaultData_);
        if(data) {
          free(data);
        }
        break;
      }
      case SerialType::KSL_MAT3X3: {
        ksl_mat3x3_t* data = static_cast<ksl_mat3x3_t*>(defaultData_);
        if(data) {
          free(data);
        }
        break;
      }
      case SerialType::KSL_MAT4X4: {
        ksl_mat4x4_t* data = static_cast<ksl_mat4x4_t*>(defaultData_);
        if(data) {
          free(data);
        }
        break;
      }
      case SerialType::BOOLEAN: {
        bool* data = static_cast<bool*>(defaultData_);
        if(data) {
          free(data);
        }
        break;
      }
      default: { break; }
    }
  };

  bool isRequired() {
    return required_;
  }

  void setRequired(bool required) {
    required_ = required;
  }

  void setDefaultData(void* defaultData) {
    defaultData_ = defaultData;
  }

  void setRowDim(const int rowDim) {
    rowDim_ = rowDim;
  }

  void setColDim(const int colDim) {
    colDim_ = colDim;
  }

  void setDescription(std::string description) {
    description_ = description;
  }

  void setQuaternionMapsToMat3x3(bool enabled) {
    quaternionMapsToMat3x3_ = enabled;
  }

  void setMat4x4MapsToMat3x3(bool enabled) {
    mat4x4MapsToMat3x3_ = enabled;
  }

  void setVec3MapsToMat3x3(bool enabled) {
    vec3MapsToMat3x3_ = enabled;
  }

  void setEulerAnglesMapToMat3x3(bool enabled) {
    eulerAnglesMapToMat3x3_ = enabled;
  }

  void addValidStringValue(std::string value) {
    validStringValues_.insert(validStringValues_.end(), value);
  }

  void addSerialEntry(SerialEntry& serialEntry);

private:
  std::string keyword_;
  std::string description_ = "N/A";
  bool required_ = true;
  SerialType type_;
  void* data_ = NULL;
  void* defaultData_ = NULL;
  int rowDim_ = 1;
  int colDim_ = 1;
  std::vector<std::string> validStringValues_;
  bool quaternionMapsToMat3x3_ = false;
  bool mat4x4MapsToMat3x3_ = false;
  bool vec3MapsToMat3x3_ = false;
  bool loadedFromFile_ = false;
  bool eulerAnglesMapToMat3x3_ = false;
  std::vector<SerialEntry> serialEntries_;
};

/*!
@brief Shift Serialize class
*/
class Serialize {

public:
  Serialize() {}
  ~Serialize() {}

  void addSerialEntry(SerialEntry& serialEntry);

private:
  std::vector<SerialEntry> serialEntries_;
};

}; // namespace Shift

#endif
