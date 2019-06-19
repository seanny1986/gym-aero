/* Copyright 2017-2018 The MathWorks, Inc. */

#ifndef CPP_ENGINE_API_HPP
#define CPP_ENGINE_API_HPP

#include "cpp_engine_api_util.hpp"
#include <MatlabDataArray/TypedArray.hpp>

/**
* Exported API to be loaded by Engine clients located outside of bin/<arch> folder.
*/
CPP_ENGINE_C_API void cpp_engine_create_session();
CPP_ENGINE_C_API uint64_t cpp_engine_create_out_of_process_matlab(char16_t** options, size_t size, bool* errFlag);
CPP_ENGINE_C_API uint64_t cpp_engine_attach_shared_matlab(const char16_t* name, bool* errFlag);
CPP_ENGINE_C_API size_t cpp_engine_find_shared_matlab(char16_t*** names);
CPP_ENGINE_C_API void cpp_engine_destroy_names(char16_t** names, size_t size);
CPP_ENGINE_C_API void cpp_engine_destroy_handles(uintptr_t* handles);
CPP_ENGINE_C_API uintptr_t cpp_engine_feval_with_completion(const uint64_t matlabHandle, const char* function, size_t nlhs, bool scalar, matlab::data::impl::ArrayImpl** args, size_t nrhs, void(*success)(void*, size_t, bool, matlab::data::impl::ArrayImpl**), void(*exception)(void*, size_t, bool, size_t, const void*), void* p, void* output, void* error, void(*write)(void*, const char16_t*, size_t), void(*deleter)(void*));
CPP_ENGINE_C_API void cpp_engine_eval_with_completion(const uint64_t matlabHandle, const char16_t* statement, void(*success)(void*), void(*exception)(void*, size_t, const void*), void* p, void* output, void* error, void(*write)(void*, const char16_t*, size_t), void(*deleter)(void*), uintptr_t** handles);
CPP_ENGINE_C_API bool cpp_engine_cancel_feval_with_completion(uintptr_t taskHandle, bool allowInteruption);
CPP_ENGINE_C_API void cpp_engine_destroy_task_handle(uintptr_t taskHandle);
CPP_ENGINE_C_API void cpp_engine_terminate_out_of_process_matlab(const uint64_t matlabHandle);
CPP_ENGINE_C_API void cpp_engine_terminate_session();
CPP_ENGINE_C_API void* cpp_engine_get_function_ptr(int fcn);

#endif /* CPP_ENGINE_API_HPP */