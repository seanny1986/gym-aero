/* Copyright 2017-2018 The MathWorks, Inc. */

#ifndef __MEX_FUNCTION_ADAPTER_HPP__
#define __MEX_FUNCTION_ADAPTER_HPP__

#include "MatlabDataArray.hpp"
#include "mex.hpp"
#include "mexEngineUtilImpl.hpp"
#include <vector>
#include <iostream>
#include "assert.h"

LIBMWMEX_API_EXTERN_C {

    void* cppmex_getEngine(void*);

    void* cppmex_mexLock(void*);

    void* cppmex_mexUnlock(void*);

    void* cppmex_mexLock_with_error_check(void*, int*);

    void* cppmex_mexUnlock_with_error_check(void*, int*);

    const char16_t* cppmex_getFunctionName(void*);

    void mexReleaseMemory(char*, char16_t*);

    void* mexGetFunctionImpl();

    void mexDestroyFunctionImpl(void*);
}


template <typename T>
matlab::mex::Function * mexCreatorUtil() {
    static_assert(std::is_base_of<matlab::mex::Function, T>::value, "MexFunction class must be a subclass of matlab::mex::Function.");
    matlab::mex::Function* mexFunction = new T();
    return mexFunction;
}

void mexDestructorUtil(matlab::mex::Function * t) {
    delete t;
}

void mexHandleException(void (*callbackErrHandler)(const char*, const char*)) {
    try {
        throw;
    } catch(const matlab::engine::MATLABException& ex) {
        callbackErrHandler(ex.what(), ex.getMessageID().c_str());
    } catch(const matlab::engine::Exception& ex) {
        callbackErrHandler(ex.what(), "");
    } catch(const matlab::Exception& ex) {
        callbackErrHandler(ex.what(), "");
    } catch(const std::exception& ex) {
        callbackErrHandler(ex.what(), "");
    } catch(...) {
        callbackErrHandler("Unknown exception thrown.", "");
    }
}

class MexFunction;

EXTERN_C
void mexFunction() {}

MEXFUNCTION_LINKAGE
void* mexCreateMexFunction(void (*callbackErrHandler)(const char*, const char*)) {
    try {
        matlab::mex::Function *mexFunc = mexCreatorUtil<MexFunction>();
        return mexFunc;
    } catch(...) {
        mexHandleException(callbackErrHandler);
        return nullptr;
    }
}

MEXFUNCTION_LINKAGE
void mexDestroyMexFunction(void* mexFunc,
                           void (*callbackErrHandler)(const char*, const char*)) {
    matlab::mex::Function* mexFunction = reinterpret_cast<matlab::mex::Function*>(mexFunc);
    try {
        mexDestructorUtil(mexFunction);
    } catch(...) {
        mexHandleException(callbackErrHandler);
        return;
    }
}

MEXFUNCTION_LINKAGE
void mexFunctionAdapter(int nlhs_,
                        int nlhs,
                        int nrhs,
                        void* vrhs[],
                        void* mFun,
                        void (*callbackOutput)(int, void**),
                        void (*callbackErrHandler)(const char*, const char*)) {

    matlab::mex::Function* mexFunction = reinterpret_cast<matlab::mex::Function*>(mFun);

    std::vector<matlab::data::Array> edi_prhs;
    edi_prhs.reserve(nrhs);
    implToArray(nrhs, vrhs, edi_prhs);

    std::vector<matlab::data::Array> edi_plhs(nlhs);
    matlab::mex::ArgumentList outputs(edi_plhs.begin(), edi_plhs.end(), nlhs_);
    matlab::mex::ArgumentList inputs(edi_prhs.begin(), edi_prhs.end(), nrhs);

    try {
        (*mexFunction)(outputs, inputs);
    } catch(...) {
        mexHandleException(callbackErrHandler);
        return;
    }

    arrayToImplOutput(nlhs, edi_plhs, callbackOutput);
}

/*** matlab::mex::Function ***/

matlab::mex::Function::Function() {
    functionImpl = mexGetFunctionImpl();
}

matlab::mex::Function::~Function() NOEXCEPT_FALSE {
    mexDestroyFunctionImpl(functionImpl);
}

void matlab::mex::Function::operator()(matlab::mex::ArgumentList outs, matlab::mex::ArgumentList ins) {}

std::shared_ptr<matlab::engine::MATLABEngine> matlab::mex::Function::getEngine() {
    void* engine = cppmex_getEngine(functionImpl);
    return std::make_shared<matlab::engine::MATLABEngine>(engine);
}

void matlab::mex::Function::mexLock() {
    int errID = 0;
    cppmex_mexLock_with_error_check(functionImpl, &errID);
    throwIfError(errID, nullptr);
}

void matlab::mex::Function::mexUnlock() {
    int errID = 0;
    cppmex_mexUnlock_with_error_check(functionImpl, &errID);
    throwIfError(errID, nullptr);
}

std::u16string matlab::mex::Function::getFunctionName() const {
    const char16_t* fn = cppmex_getFunctionName(functionImpl);

    if(!fn) {
        std::string outOfMemoryError = "Not enough memory available to support the request.";
        throw matlab::engine::Exception(outOfMemoryError);
    }

    char16_t* fName = const_cast<char16_t*>(fn);
    std::u16string fNameStr = std::u16string(fName);
    mexReleaseMemory(nullptr, fName);
    return fNameStr;
}

#endif
