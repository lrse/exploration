/* 
 * File:   singleton.h
 * Author: v01d
 *
 * Created on September 6, 2009, 8:12 PM
 */

#ifndef _SINGLETON_H
#define	_SINGLETON_H

#include <iostream>
#include <stdexcept>
#include <string>
#include <typeinfo>

namespace HybNav {
  /**
   * This templatized class serves as a base class for other classes to get a singleton-like
   * functionality.
   * Child classes must use the public constructor to set the instance pointer, which will be automatically
   * unset when the instance is destroyed.
   * This means that this base class doesn't provide implicit creation/destruction, which must be handled by the user.
   */
  template<class T>
  class Singleton {
    public:
      // Child classes will call this constructor setting the singleton instance pointer
      Singleton(T* child_this) {
        if (instance_ptr()) throw std::runtime_error((std::string("Singleton<") + typeid(T).name()) + std::string("> already created"));
        instance_ptr() = child_this;
      }

      // Returns the instance pointer. Throws an exception if the pointer is not set.
      inline static T* instance(void) {
        if (!instance_ptr()) throw std::runtime_error((std::string("Singleton<") + typeid(T).name()) + std::string("> is not instatiated yet/anymore"));
        return instance_ptr();
      }

    private:
      Singleton(void) {} // forces the usage of the public constructor
      Singleton(const T&) {} // avoid copies

      // magic method that holds and also returns the internal singleton pointer for use/modification
      inline static T*& instance_ptr(void) {
        static T* _instance_ptr = NULL;
        return _instance_ptr;
      }

    protected:      
      // Automatically unsets the instance pointer. Protected to avoid instantiating on stack.
      virtual ~Singleton(void) { instance_ptr() = NULL; }
  };
}

#endif	/* _SINGLETON_H */


