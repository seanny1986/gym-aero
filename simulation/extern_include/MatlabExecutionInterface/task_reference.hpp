/* Copyright 2017 The MathWorks, Inc. */

#ifndef TASK_REFERENCE_HPP
#define TASK_REFERENCE_HPP

#include <string>
#include <functional>

namespace matlab {
    namespace execution{
        
        class TaskReference {
        public:
            TaskReference();
            TaskReference(std::function<bool(uintptr_t, bool)>&& cancel);
            TaskReference(uintptr_t aHandle, std::function<bool(uintptr_t, bool)>&& cancel);
            uintptr_t getHandle() const ;
            virtual ~TaskReference();
            virtual bool cancel(bool allowInterrupt);
            
        private:
            TaskReference(TaskReference& handle) = delete;
            TaskReference& operator=(TaskReference&) = delete;
            uintptr_t handle;
            std::function<bool(uintptr_t, bool)> cancelImpl;
            
        };
    }
}        
        
#endif //TASK_REFERENCE_HPP