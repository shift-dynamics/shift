#ifndef SHIFT_SERIALIZE_H_
#define SHIFT_SERIALIZE_H_

#include <exception>
#include <json/json.h>
#include <string>
#include <vector>

#include "ksl/SE3.h"
#include "ksl/coscrew.h"
#include "ksl/inertia.h"
#include "ksl/quaternion.h"
#include "ksl/screw.h"
#include "ksl/vec3.h"
#include "ksl/vec4.h"

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

class SerialEntry;

class SerialEntry {

public:
  SerialEntry(const std::string key, const SerialType serialType, void* data,
              const int rowDim = 1, const int colDim = 1);
  ~SerialEntry();

  std::string getKey() {
    return key_;
  }

  std::string getDescription() {
    return description_;
  }

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

  void setMat4x4MapsToSE3(bool enabled) {
    mat4x4MapsToSE3_ = enabled;
  }

  void setVec3MapsToMat3x3(bool enabled) {
    vec3MapsToMat3x3_ = enabled;
  }

  void setEulerAnglesMapToMat3x3(bool enabled) {
    eulerAnglesMapToMat3x3_ = enabled;
  }

  void setNormalizeQuaternion(bool enabled) {
    normalizeQuaternion_ = enabled;
  }

  void setNormalizeMat3x3(bool enabled) {
    normalizeMat3x3_ = enabled;
  }

  void addValidStringValue(std::string value) {
    validStringValues_.insert(validStringValues_.end(), value);
  }

  bool isLoadedFromFile() {
    return loadedFromFile_;
  }

  void readJSON(Json::Value&);

private:
  std::string key_;
  std::string description_ = "N/A";
  bool required_ = true;
  SerialType type_;
  void* data_ = NULL;
  void* defaultData_ = NULL;
  int rowDim_ = 1; /*!< -1 indicates dynamic sizing requested */
  int colDim_ = 1; /*!< -1 indicates dynamic sizing requested */

  std::vector<std::string> validStringValues_;
  bool quaternionMapsToMat3x3_ = false;
  bool mat4x4MapsToSE3_ = false;
  bool vec3MapsToMat3x3_ = false;
  bool loadedFromFile_ = false;
  bool eulerAnglesMapToMat3x3_ = false;
  bool normalizeQuaternion_ = true;
  bool normalizeMat3x3_ = true;
  ksl_vec3i_t axes_ = {{0, 1, 2}};

  void readJSON_STRING(Json::Value&);
  void readJSON_STRING_ARRAY(Json::Value&);
  void readJSON_STRING_2DARRAY(Json::Value&);
  void readJSON_DOUBLE(Json::Value&);
  void readJSON_DOUBLE_ARRAY(Json::Value&);
  void readJSON_KSL_VEC3(Json::Value&);
  void readJSON_KSL_VEC4(Json::Value&);
  void readJSON_KSL_QUATERNION(Json::Value&);
  void readJSON_KSL_SCREW(Json::Value&);
  void readJSON_KSL_COSCREW(Json::Value&);
  void readJSON_KSL_INERTIA(Json::Value&);
  void readJSON_KSL_MAT3X3(Json::Value&);
  void readJSON_KSL_MAT4X4(Json::Value&);
  void readJSON_EULERANGLES(Json::Value&);
  void readJSON_BOOLEAN(Json::Value&);
};

/*!
@brief Shift Serialize class
*/
class Serialize {

public:
  Serialize() {}
  ~Serialize() {}

  void addSerialEntry(SerialEntry& serialEntry);
  void readJSON(Json::Value&);

protected:
  std::vector<SerialEntry> serialEntries_;
};

}; // namespace Shift

#endif
