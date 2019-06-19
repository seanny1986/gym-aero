/* Copyright 2017 The MathWorks, Inc. */

#ifndef TASK_REFERENCE_IMPL_HPP
#define TASK_REFERENCE_IMPL_HPP

#include <MatlabExecutionInterface/task_reference.hpp>

namespace matlab {
    namespace execution {
        
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
            if(handle) cpp_engine_destroy_task_handle(handle);
        };
        inline  bool TaskReference::cancel(bool allowInterrupt) 
        {
            return cancelImpl(handle, allowInterrupt);
        }
    }
}        
        
#endif //TASK_REFERENCE_IMPL_HPP