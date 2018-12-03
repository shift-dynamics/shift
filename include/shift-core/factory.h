#ifndef _SHIFT_FACTORY_
#define _SHIFT_FACTORY_

#include <functional>
#include <map>

namespace Shift {

template <class T> class Factory {

public:
  Factory();
  Factory(const Factory&) {}
  Factory& operator=(const Factory&) {
    return *this;
  }

  ~Factory() {
    factoryMap_.clear();
  }

  typedef std::function<T*()> CreateFunction;

  static Factory* get() {
    static Factory instance;
    return &instance;
  }

  static void registerModule(const std::string& name,
                             CreateFunction pfnCreate) {
    factoryMap_.insert(std::make_pair(name, pfnCreate));
  }
  static T* create(const std::string& name) {
    return factoryMap_[name]();
  }

private:
  static std::map<std::string, CreateFunction> factoryMap_;
};

template <class T> class Registration {
public:
  Registration(std::string name, typename Factory<T>::CreateFunction fcn) {
    Factory<T>::registerModule(name, fcn);
  }
};

} // namespace Shift

#endif
