/* Copyright 2017 The MathWorks, Inc. */

#ifndef CPPSHAREDLIB_API_HPP
#define CPPSHAREDLIB_API_HPP

#include "cppsharedlib_api_util.hpp"
#include <MatlabDataArray/TypedArray.hpp>

/**
* Exported API to be loaded by C++ shared library clients located outside of bin/<arch> folder.
*/
CPP_RUNTIME_C_API void runtime_create_session(char16_t** options, size_t size);
CPP_RUNTIME_C_API void runtime_terminate_session();
CPP_RUNTIME_C_API uint64_t create_mvm_instance_async(const char16_t* name);
CPP_RUNTIME_C_API uint64_t create_mvm_instance(const char16_t* name, bool* errFlag);
CPP_RUNTIME_C_API void terminate_mvm_instance(const uint64_t mvmHandle);
CPP_RUNTIME_C_API void wait_for_figures_to_close(const uint64_t mvmHandle);

CPP_RUNTIME_C_API void cppsharedlib_destroy_handles(uintptr_t* handles);

CPP_RUNTIME_C_API uintptr_t cppsharedlib_feval_with_completion(  const uint64_t matlabHandle, 
                                                            const char* function, 
                                                            size_t nlhs, 
                                                            bool scalar, 
                                                            matlab::data::impl::ArrayImpl** args, 
                                                            size_t nrhs, 
                                                            void(*success)(void*, size_t, bool, matlab::data::impl::ArrayImpl**), 
                                                            void(*exception)(void*, size_t, bool, size_t, const void*), 
                                                            void* p, 
                                                            void* output, 
                                                            void* error, 
                                                            void(*write)(void*, const char16_t*, size_t), 
                                                            void(*deleter)(void*));

CPP_RUNTIME_C_API bool cppsharedlib_cancel_feval_with_completion(uintptr_t taskHandle, bool allowInteruption);
CPP_RUNTIME_C_API void cppsharedlib_destroy_task_handle(uintptr_t taskHandle);

CPP_RUNTIME_C_API size_t cppsharedlib_get_stacktrace_number(const uintptr_t frameHandle);
CPP_RUNTIME_C_API const char* cppsharedlib_get_stacktrace_message(const uintptr_t frameHandle);
CPP_RUNTIME_C_API const char16_t* cppsharedlib_get_stackframe_file(const uintptr_t frameHandle, size_t frameNumber);
CPP_RUNTIME_C_API const char* cppsharedlib_get_stackframe_func(const uintptr_t frameHandle, size_t frameNumber);
CPP_RUNTIME_C_API uint64_t cppsharedlib_get_stackframe_line(const uintptr_t frameHandle, size_t frameNumber);

CPP_RUNTIME_C_API int cppsharedlib_run_main(int(*mainfcn)(int, const char**), int argc, const char** argv);

#endif /* CPPSHAREDLIB_API_HPP */
