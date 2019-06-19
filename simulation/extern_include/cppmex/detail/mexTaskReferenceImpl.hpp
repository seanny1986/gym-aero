/* Copyright 2018 The MathWorks, Inc. */

#ifndef __MEX_TASK_REFERENCE_IMPL_HPP__
#define __MEX_TASK_REFERENCE_IMPL_HPP__

#include "../mexTaskReference.hpp"

LIBMWMEX_API_EXTERN_C{
    void mexApiDestroyTaskHandle(uintptr_t);
}

namespace matlab {
    namespace engine {
        
        inline TaskReference::TaskReference() {};
        inline TaskReference::TaskReference(std::function<bool(uintptr_t, bool)>&& cancel)
            :handle(0),cancelImpl(std::move(cancel)){};
        inline TaskReference::TaskReference(uintptr_t aHandle, std::function<bool(uintptr_t, bool)>&& cancel)
            :handle(aHandle), cancelImpl(std::move(cancel)) {};
        inline  uintptr_t TaskReference::getHandle() const 
        { 
            return handle;  
        }
        inline  TaskReference::~TaskReference()
        {
            if(handle) mexApiDestroyTaskHandle(handle);
        };
        inline  bool TaskReference::cancel(bool allowInterrupt) 
        {
            return cancelImpl(handle, allowInterrupt);
        }
    }
}        
        
#endif //__MEX_TASK_REFERENCE_IMPL_HPP__