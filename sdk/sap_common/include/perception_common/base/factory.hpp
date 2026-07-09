/*
 * Copyright (C) 2020 by SenseTime Group Limited. All rights reserved.
 * Jiang Qinhong <jiangqinhong@sensetime.com>
 */
#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "perception_common/log/common_log.hpp"

// To regiester factory, and class for factory
// Factory should be registered in base class

namespace senseAD {
namespace perception {

// idea from boost any but make it more simple and don't use type_info.
class Abstract {
 public:
    Abstract() : content_(NULL) {}

    template <typename T>
    explicit Abstract(const T &value) : content_(new Holder<T>(value)) {}

    Abstract(const Abstract &other)
        : content_(other.content_ ? other.content_->Clone() : nullptr) {}

    ~Abstract() { delete content_; }

    template <typename T>
    T *AbstractCast() {
        return content_ ? &(static_cast<Holder<T> *>(content_)->held_)
                        : nullptr;
    }

 private:
    class PlaceHolder {
     public:
        virtual ~PlaceHolder() {}
        virtual PlaceHolder *Clone() const = 0;
    };

    template <typename T>
    class Holder : public PlaceHolder {
     public:
        explicit Holder(const T &value) : held_(value) {}
        virtual ~Holder() {}
        virtual PlaceHolder *Clone() const { return new Holder(held_); }

        T held_;
    };

    PlaceHolder *content_;
};

class AbstractFactory {
 public:
    AbstractFactory() {}
    virtual ~AbstractFactory() {}
    virtual Abstract NewInstance() { return Abstract(); }
    AbstractFactory(const AbstractFactory &) = delete;
    AbstractFactory &operator=(const AbstractFactory &) = delete;

 private:
};

typedef std::unordered_map<std::string, AbstractFactory *> FactoryMap;
typedef std::unordered_map<std::string, FactoryMap> BaseClassMap;

inline BaseClassMap &GlobalFactoryMap() {
    static BaseClassMap factory_map;
    return factory_map;
}

inline bool GetRegisteredClasses(
    const std::string &domain,
    const std::string &base_class_name,
    std::vector<std::string> *registered_derived_classes_names) {
    if (registered_derived_classes_names == nullptr) {
        AP_LERROR(Factory)
            << "registered_derived_classes_names is not available";
        return false;
    }
    BaseClassMap &map = GlobalFactoryMap();
    auto iter = map.find(domain + base_class_name);
    if (iter == map.end()) {
        AP_LERROR(Factory) << "class not registered:"
                           << domain + base_class_name;
        return false;
    }
    for (auto pair : iter->second) {
        registered_derived_classes_names->push_back(pair.first);
    }
    return true;
}

}  // namespace perception
}  // namespace senseAD

#define REGISTER_FACTORY(domain, base_class)                                 \
    class domain##base_class##Register {                                     \
        typedef ::senseAD::perception::Abstract Abstract;                    \
        typedef ::senseAD::perception::FactoryMap FactoryMap;                \
                                                                             \
     public:                                                                 \
        static base_class *GetInstanceByName(const ::std::string &name) {    \
            FactoryMap &map = ::senseAD::perception::GlobalFactoryMap()      \
                [std::string(#domain) + std::string(#base_class)];           \
            FactoryMap::iterator iter = map.find(name);                      \
            if (iter == map.end()) {                                         \
                for (auto c : map) {                                         \
                    AP_LERROR(Factory) << "Instance:" << c.first;            \
                }                                                            \
                AP_LERROR(Factory) << "Get instance " << name << " failed."; \
                return nullptr;                                              \
            }                                                                \
            Abstract object = iter->second->NewInstance();                   \
            return *(object.AbstractCast<base_class *>());                   \
        }                                                                    \
        static std::vector<base_class *> GetAllInstances() {                 \
            std::vector<base_class *> instances;                             \
            FactoryMap &map = ::senseAD::perception::GlobalFactoryMap()      \
                [std::string(#domain) + std::string(#base_class)];           \
            instances.reserve(map.size());                                   \
            for (auto item : map) {                                          \
                Abstract object = item.second->NewInstance();                \
                instances.push_back(*(object.AbstractCast<base_class *>())); \
            }                                                                \
            return instances;                                                \
        }                                                                    \
        static const ::std::string GetUniqInstanceName() {                   \
            FactoryMap &map = ::senseAD::perception::GlobalFactoryMap()      \
                [std::string(#domain) + std::string(#base_class)];           \
            return map.begin()->first;                                       \
        }                                                                    \
        static base_class *GetUniqInstance() {                               \
            FactoryMap &map = ::senseAD::perception::GlobalFactoryMap()      \
                [std::string(#domain) + std::string(#base_class)];           \
            Abstract object = map.begin()->second->NewInstance();            \
            return *(object.AbstractCast<base_class *>());                   \
        }                                                                    \
        static bool IsValid(const ::std::string &name) {                     \
            FactoryMap &map = ::senseAD::perception::GlobalFactoryMap()      \
                [std::string(#domain) + std::string(#base_class)];           \
            return map.find(name) != map.end();                              \
        }                                                                    \
    };

#define REGISTER_CLASS(domain, clazz, name)                                  \
    namespace {                                                              \
    class AbstractFactory##domain##name                                      \
        : public senseAD::perception::AbstractFactory {                      \
     public:                                                                 \
        virtual ~AbstractFactory##domain##name() {}                          \
        virtual ::senseAD::perception::Abstract NewInstance() {              \
            return ::senseAD::perception::Abstract(new name());              \
        }                                                                    \
    };                                                                       \
    __attribute__((constructor)) void RegisterFactory##domain##name() {      \
        ::senseAD::perception::FactoryMap &map =                             \
            ::senseAD::perception::GlobalFactoryMap()[std::string(#domain) + \
                                                      std::string(#clazz)];  \
        if (map.find(#name) == map.end())                                    \
            map[#name] = new AbstractFactory##domain##name();                \
    }                                                                        \
    }  // namespace

#define REGISTER_TEMPLATE_CLASS(domain, clazz, name, suffix, T)            \
    namespace {                                                            \
    class AbstractFactory##domain##name##suffix                            \
        : public senseAD::perception::AbstractFactory {                    \
     public:                                                               \
        virtual ~AbstractFactory##domain##name##suffix() {}                \
        virtual ::senseAD::perception::Abstract NewInstance() {            \
            return ::senseAD::perception::Abstract(new name<T>());         \
        }                                                                  \
    };                                                                     \
    __attribute__((constructor))                                           \
        void RegisterFactory##domain##name##suffix() {                     \
        ::senseAD::perception::FactoryMap &map =                           \
            ::senseAD::perception::GlobalFactoryMap()                      \
                [std::string(#domain) + std::string(#clazz)];              \
        std::string full_name = std::string(#name) + std::string(#suffix); \
        if (map.find(full_name) == map.end())                              \
            map[full_name] = new AbstractFactory##domain##name##suffix();  \
    }                                                                      \
    }  // namespace
