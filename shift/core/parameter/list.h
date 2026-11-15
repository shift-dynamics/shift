#ifndef SHIFT_PARAMETER_LIST_H_
#define SHIFT_PARAMETER_LIST_H_

#include "shift/core/parameter/value.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace shift {
namespace core {

class Context;
class Assembly;
namespace body {
class Base;
}

namespace parameter {

/*!
@brief Shift parameter::List class
*/
class List {

public:
  List() {}
  List(Context* context, Assembly* assembly = nullptr,
       shift::core::body::Base* body = nullptr)
    : context_(context)
    , parent_assembly_ptr_(assembly)
    , parent_body_ptr_(body) {}

  virtual ~List() {}

  virtual Value& operator[](const char* key) {
    return parameters_[key];
  }

  virtual Value& operator[](const std::string key) {
    return parameters_[key];
  }

  /*! \brief the parameter of the key, or nullptr if there is none; unlike
   * operator[], it does not add one */
  Value* Find(const std::string& key) {
    auto it = parameters_.find(key);
    return it == parameters_.end() ? nullptr : &it->second;
  }

  bool is_active() {
    return active_;
  }

  Assembly* get_parent_assembly() const {
    return parent_assembly_ptr_;
  }

  /*! \brief name, an object of this list, with the assemblies around it:
   * <assembly>__<subassembly>__<name>. Names are unique within an assembly, so
   * this identifies the object in the model. An object of a body has the name
   * of the body before its own. */
  std::string FullyQualifiedName(const std::string& name) const;

  /*! \brief throw if two of the fully qualified names are the same; kind names
   * the objects in the message */
  static void CheckNames(const std::vector<std::string>& names,
                         const std::string& kind);

  /*! \brief as CheckNames, for the objects of an array; unnamed ones are
   * skipped, as a name is optional for some */
  template <class T>
  static void CheckObjectNames(const std::vector<std::unique_ptr<T>>& objects,
                               const std::string& kind) {
    std::vector<std::string> names;
    for(const auto& object : objects) {
      if(!object->get_name().empty()) {
        names.push_back(object->FullyQualifiedName(object->get_name()));
      }
    }
    CheckNames(names, kind);
  }

  shift::core::body::Base* get_parent_body() {
    return parent_body_ptr_;
  }

  void set_parent(Context* context, Assembly* parent_assembly = nullptr,
                  shift::core::body::Base* parent_body = nullptr);

  auto& get_parameters() const {
    return parameters_;
  }

  void set_context(Context* context) {
    context_ = context;
  }

  void set_parent_assembly(Assembly* assembly) {
    parent_assembly_ptr_ = assembly;
  }

  void set_parent_body(shift::core::body::Base* body) {
    parent_body_ptr_ = body;
  }

  void set_is_body(bool is_body) {
    is_body_ = is_body;
  }

  void set_is_assembly(bool is_assembly) {
    is_assembly_ = is_assembly;
  }

  bool has_key(const std::string key) const {
    return parameters_.count(key);
  }

  virtual void ActivateObjects(bool parent_is_active) {
    for(auto& parameter : parameters_) {
      parameter.second.ActivateObjects(active_ && parent_is_active);
    }
  }

  virtual void DeclareParameters() = 0;

  void RemoveParameter(std::string key);

  Value& DeclareParameter(const std::string key, Value parameter);

  Value& DeclareParameter(const std::string key, std::string* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);
    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, std::vector<std::string>* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key,
                          std::vector<std::vector<std::string>>* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, int* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, std::vector<int>* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, double* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, std::vector<double>* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_vec3_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_vec4_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_quaternion_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_screw_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_coscrew_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_inertia_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_mat3x3_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_mat4x4_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_eulerangles_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);

    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, bool* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);
    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ksl_SE3_t* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);
    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, List* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);
    return DeclareParameter(key, value);
  }

  Value& DeclareParameter(const std::string key, ArrayBase* data,
                          const std::string description, bool required = true) {
    Value value(data, description, required);
    return DeclareParameter(key, value);
  }

  /*! \brief declare an input of the component: a number that it reads each
   * time it is evaluated, and that another component, such as a controller,
   * sets as the model runs (SetInput). The model file may give the input's
   * value, which holds until something sets it; without one, the input
   * starts at the value of data. An input can be changed while the model
   * runs (Value::set_runtime), so a schedule can set it too. One component
   * at most sets an input (ConnectInput). */
  Value& DeclareInput(const std::string key, double* data,
                      const std::string description) {
    Value value(data, description, false);
    value.set_role(Value::INPUT).set_runtime();
    return DeclareParameter(key, value);
  }

  /*! \brief declare an output of the component: a number that it computes
   * as the model runs, for other components to read (GetOutput) and for the
   * loggers to record. An output is not read from the model file; giving it
   * there is an error. A component declares its outputs in
   * DeclareParameters, or, when they depend on the model (one per pulley of
   * a belt, say), in its Load or Connect, pointing into storage that does
   * not move after that; the loggers connect to them after every Load. The
   * outputs are logged in the order they are declared. Throws if key is a
   * parameter or an input of the component. */
  Value& DeclareOutput(const std::string key, double* data,
                       const std::string description);

  /*! \brief declare an output of each of names, kept in values, which is
   * sized to them and must not be resized after */
  void DeclareOutputs(const std::vector<std::string>& names,
                      std::vector<double>* values,
                      const std::string& description) {
    values->assign(names.size(), 0.0);
    for(size_t k = 0; k < names.size(); k++) {
      DeclareOutput(names[k], &(*values)[k], description);
    }
  }

  /*! \brief the keys of the component's outputs, in the order declared */
  const std::vector<std::string>& get_output_keys() const {
    return output_keys_;
  }

  /*! \brief whether the component declares an input or an output of key */
  bool has_input(const std::string& key) const;
  bool has_output(const std::string& key) const;

  /*! \brief record driver, the fully qualified name of a component, as the
   * one that sets the input key of this component. Called once, when the
   * driver connects (in its Connect); throws if there is no such input, or
   * another component sets it already. */
  void ConnectInput(const std::string& key, const std::string& driver);

  /*! \brief set the input key to value; throws if there is no such input */
  void SetInput(const std::string& key, double value);

  /*! \brief the value of the input key; throws if there is no such input */
  double GetInput(const std::string& key) const;

  /*! \brief the value of the output key; throws if there is no such output */
  double GetOutput(const std::string& key) const;

  /*! \brief where the output key is kept, for a logger to read it at each
   * output time; throws if there is no such output */
  const double* GetOutputAddress(const std::string& key) const;

  virtual void ReadJSON(Json::Value&);

  void Print() const;
  void Print(std::string, bool break_line = true) const;
  std::vector<std::string> DescribeHeaders() {
    std::vector<std::string> headers;
    return headers;
  }

protected:
  /*! \brief the name of the component, for messages: its "name" parameter,
   * or its "type" when it has no name */
  std::string NameForMessages() const;

  /*! \brief the value of key if it has the role, else throws, saying what
   * the component lacks */
  const Value& FindRole(const std::string& key, Value::Role role) const;

  std::map<std::string, parameter::Value> parameters_;
  std::vector<std::string> output_keys_; //!< in the order declared
  Context* context_ = nullptr;
  Assembly* parent_assembly_ptr_ = nullptr;
  shift::core::body::Base* parent_body_ptr_ = nullptr;
  bool active_ = true;
  bool is_body_ = false;
  bool is_assembly_ = false;
};

} // namespace parameter
} // namespace core
} // namespace shift
#endif
