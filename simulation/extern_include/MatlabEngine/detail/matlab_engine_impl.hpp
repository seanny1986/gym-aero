/* Copyright 2017 The MathWorks, Inc. */

#ifndef MATLAB_ENGINE_IMPL_HPP
#define MATLAB_ENGINE_IMPL_HPP

#include "../matlab_engine.hpp"
#include "engine_execution_interface_impl.hpp"
#include "engine_exception_impl.hpp"

namespace matlab {
    namespace engine {
            
        using namespace matlab::execution; 
        
        inline MATLABEngine::MATLABEngine(uint64_t handle) : matlab::execution::ExecutionInterface(handle) {
        }

        
        inline void MATLABEngine::eval(const std::u16string &str,
        const std::shared_ptr<StreamBuffer> &output,
        const std::shared_ptr<StreamBuffer> &error
        ) {
            return evalAsync(str, output, error).get();
        }

        inline matlab::data::Array MATLABEngine::getVariable(const std::u16string &varName, WorkspaceType workspaceType) {
            return getVariableAsync(varName, workspaceType).get();
        }

        inline matlab::data::Array MATLABEngine::getVariable(const std::string &varName, WorkspaceType workspaceType) {
            return getVariable(std::u16string(varName.cbegin(), varName.cend()), workspaceType);
        }

        inline void MATLABEngine::setVariable(const std::u16string &varName, const matlab::data::Array& var, WorkspaceType workspaceType) {
            return setVariableAsync(varName, var, workspaceType).get();
        }

        inline void MATLABEngine::setVariable(const std::string &varName, const matlab::data::Array& var, WorkspaceType workspaceType) {
            return setVariable(std::u16string(varName.cbegin(), varName.cend()), var, workspaceType);
        }

        inline matlab::data::Array MATLABEngine::getProperty(const matlab::data::Array &object, const std::u16string &propertyName) {
            return getPropertyAsync(object, propertyName).get();
        }

        inline matlab::data::Array MATLABEngine::getProperty(const matlab::data::Array &object, const std::string &propertyName) {
            return getProperty(object, std::u16string(propertyName.cbegin(), propertyName.cend()));
        }

        inline matlab::data::Array MATLABEngine::getProperty(const matlab::data::Array &objectArray, size_t index, const std::u16string &propertyName) {
            return getPropertyAsync(objectArray, index, propertyName).get();
        }

        inline matlab::data::Array MATLABEngine::getProperty(const matlab::data::Array &objectArray, size_t index, const std::string &propertyName) {
            return getProperty(objectArray, index, std::u16string(propertyName.cbegin(), propertyName.cend()));
        }

        inline void MATLABEngine::setProperty(matlab::data::Array &object, const std::u16string &propertyName, const matlab::data::Array &property) {
            return setPropertyAsync(object, propertyName, property).get();
        }

        inline void MATLABEngine::setProperty(matlab::data::Array &object, const std::string &propertyName, const matlab::data::Array &property) {
            return setProperty(object, std::u16string(propertyName.cbegin(), propertyName.cend()), property);
        }

        inline void MATLABEngine::setProperty(matlab::data::Array &objectArray, size_t index, const std::u16string &propertyName, const matlab::data::Array &property) {
            return setPropertyAsync(objectArray, index, propertyName, property).get();
        }

        inline void MATLABEngine::setProperty(matlab::data::Array &objectArray, size_t index, const std::string &propertyName, const matlab::data::Array &property) {
            return setProperty(objectArray, index, std::u16string(propertyName.cbegin(), propertyName.cend()), property);
        }

        inline MATLABEngine::~MATLABEngine() {
            if (matlabHandle != 0) {
                cpp_engine_terminate_out_of_process_matlab(matlabHandle);
                matlabHandle = 0;
            }
        }

        
        inline FutureResult<void> MATLABEngine::evalAsync(const std::u16string &statement,
            const std::shared_ptr<StreamBuffer> &output,
            const std::shared_ptr<StreamBuffer> &error
            ) {
           
            std::promise<void>* p = new std::promise<void>();
            std::future<void> f = p->get_future();

            uintptr_t* handles;
            std::vector<uintptr_t> handlesV(3);
            
            void* output_ = output? new std::shared_ptr<StreamBuffer>(std::move(output)) : nullptr;
            void* error_ = error? new std::shared_ptr<StreamBuffer>(std::move(error)) : nullptr;

            cpp_engine_eval_with_completion(matlabHandle, statement.c_str(), &set_eval_promise_data, &set_eval_promise_exception, p, output_, error_, &writeToStreamBuffer, &deleteStreamBufferImpl, &handles);

            for (size_t i = 0; i < 3; i++) {
                handlesV[i] = handles[i];
            }
            cpp_engine_destroy_handles(handles);
            return FutureResult<void>(std::move(f), std::make_shared<TaskReference>(handlesV[0], &cpp_engine_cancel_feval_with_completion));
        }


        inline FutureResult<matlab::data::Array> MATLABEngine::getVariableAsync(const std::u16string &varName, WorkspaceType workspaceType) {
            matlab::data::ArrayFactory factory;
            const std::string base = (workspaceType == WorkspaceType::BASE) ? std::string("base") : std::string("global");
            auto arg1 = factory.createCharArray(base);
            auto arg2 = factory.createCharArray(varName);
            std::vector<matlab::data::Array> args;
            args.push_back(arg2);
            args.push_back(arg1);
            return fevalAsync(convertUTF8StringToUTF16String("matlab.internal.engine.getVariable"), args, nullptr, nullptr);
        }

        inline FutureResult<matlab::data::Array> MATLABEngine::getVariableAsync(const std::string &varName, WorkspaceType workspaceType) {
            return getVariableAsync(std::u16string(varName.cbegin(), varName.cend()), workspaceType);
        }

        inline FutureResult<void> MATLABEngine::setVariableAsync(const std::u16string &varName, const matlab::data::Array& var, WorkspaceType workspaceType) {
            matlab::data::ArrayFactory factory;
            std::string base = (workspaceType == WorkspaceType::BASE) ? std::string("base") : std::string("global");
            auto arg1 = factory.createCharArray(base);
            auto arg2 = factory.createCharArray(varName);
            std::vector<matlab::data::Array> args;
            args.push_back(arg2);
            args.push_back(var);
            args.push_back(arg1);
            size_t nrhs = args.size();
            std::unique_ptr<matlab::data::impl::ArrayImpl*, void(*)(matlab::data::impl::ArrayImpl**)> argsImplPtr(new matlab::data::impl::ArrayImpl*[nrhs], [](matlab::data::impl::ArrayImpl** ptr) {
                delete[] ptr;
            });
            matlab::data::impl::ArrayImpl** argsImpl = argsImplPtr.get();
            size_t i = 0;
            for (auto e : args) {
                argsImpl[i++] = matlab::data::detail::Access::getImpl<matlab::data::impl::ArrayImpl>(e);
            }
            std::promise<void>* p = new std::promise<void>();
            std::future<void> f = p->get_future();
            uintptr_t handle = cpp_engine_feval_with_completion(matlabHandle, "matlab.internal.engine.setVariable", 0, true, argsImpl, nrhs, &set_feval_promise_data, &set_feval_promise_exception, p, nullptr, nullptr, &writeToStreamBuffer, &deleteStreamBufferImpl);
            return FutureResult<void>(std::move(f), std::make_shared<TaskReference>(handle, &cpp_engine_cancel_feval_with_completion));
        }

        inline FutureResult<void> MATLABEngine::setVariableAsync(const std::string &varName, const matlab::data::Array& var, WorkspaceType workspaceType) {
            return setVariableAsync(std::u16string(varName.cbegin(), varName.cend()), var, workspaceType);
        }

        namespace {
            FutureResult<matlab::data::Array> getPropertyAsyncImpl(const matlab::data::Array &objectArray, size_t index, const std::u16string &propertyName, matlab::engine::MATLABEngine* eng, bool isScalar) {
                matlab::data::ArrayFactory factory;
                auto arg = factory.createCharArray(propertyName);
                std::vector<matlab::data::Array> args;
                args.push_back(objectArray);
                if (!isScalar) {
                    args.push_back(factory.createScalar(index + 1));
                }
                args.push_back(arg);
                return eng->fevalAsync(convertUTF8StringToUTF16String("matlab.internal.engine.getProperty"), args, nullptr, nullptr);
            }
        }

        inline FutureResult<matlab::data::Array> MATLABEngine::getPropertyAsync(const matlab::data::Array &object, const std::u16string &propertyName) {
            return getPropertyAsyncImpl(object, 0, propertyName, this, true);
        }

        inline FutureResult<matlab::data::Array> MATLABEngine::getPropertyAsync(const matlab::data::Array &objectArray, size_t index, const std::u16string &propertyName) {
            return getPropertyAsyncImpl(objectArray, index, propertyName, this, false);
        }

        inline FutureResult<matlab::data::Array> MATLABEngine::getPropertyAsync(const matlab::data::Array &object, const std::string &propertyName) {
            return getPropertyAsync(object, std::u16string(propertyName.cbegin(), propertyName.cend()));
        }

        namespace {

            FutureResult<void> setPropertyAsyncImpl(matlab::data::Array &objectArray, size_t index, const std::u16string &propertyName, const matlab::data::Array &property, matlab::engine::MATLABEngine* eng, uint64_t engHandle, bool isScalar) {
                matlab::data::ArrayFactory factory;
                auto arg = factory.createCharArray(propertyName);
                std::vector<matlab::data::Array> args;
                args.push_back(objectArray);
                if (!isScalar) {
                    args.push_back(factory.createScalar(index + 1));
                }
                args.push_back(arg);
                args.push_back(property);
                size_t nrhs = args.size();
                FutureResult<void> ret;
                if (objectArray.getType() == matlab::data::ArrayType::VALUE_OBJECT) {
                    FutureResult<matlab::data::Array> f = eng->fevalAsync(convertUTF8StringToUTF16String("matlab.internal.engine.setProperty"), args);
                    auto copyableF = std::make_shared<FutureResult<matlab::data::Array>>(std::move(f));
                    auto convertToResultType = [&objectArray, copyableF]() {
                        matlab::data::Array vec = copyableF->get();
                        objectArray = std::move(vec);
                    };
                    std::future<void> future = std::async(std::launch::deferred, std::move(convertToResultType));
                    ret = FutureResult<void>(std::move(future), copyableF->getTaskReference());
                }
                else if (objectArray.getType() == matlab::data::ArrayType::HANDLE_OBJECT_REF) {
                    std::unique_ptr<matlab::data::impl::ArrayImpl*, void(*)(matlab::data::impl::ArrayImpl**)> argsImplPtr(new matlab::data::impl::ArrayImpl*[nrhs], [](matlab::data::impl::ArrayImpl** ptr) {
                        delete[] ptr;
                    });
                    matlab::data::impl::ArrayImpl** argsImpl = argsImplPtr.get();;
                    size_t i = 0;
                    for (auto e : args) {
                        argsImpl[i++] = matlab::data::detail::Access::getImpl<matlab::data::impl::ArrayImpl>(e);
                    }
                    std::promise<void>* p = new std::promise<void>();
                    std::future<void> f = p->get_future();
                    uintptr_t handle = cpp_engine_feval_with_completion(engHandle, "matlab.internal.engine.setProperty", 0, true, argsImpl, nrhs, &set_feval_promise_data, &set_feval_promise_exception, p, nullptr, nullptr, &writeToStreamBuffer, &deleteStreamBufferImpl);

                    ret = FutureResult<void>(std::move(f), std::make_shared<TaskReference>(handle, &cpp_engine_cancel_feval_with_completion));
                }
                else {
                    throw matlab::engine::EngineException("The input variable is not a MATLAB object.");
                }
                return ret;
            }
        }

        inline FutureResult<void> MATLABEngine::setPropertyAsync(matlab::data::Array &object, const std::u16string &propertyName, const matlab::data::Array &property) {
            return setPropertyAsyncImpl(object, 0, propertyName, property, this, matlabHandle, true);
        }

        inline FutureResult<matlab::data::Array> MATLABEngine::getPropertyAsync(const matlab::data::Array &objectArray, size_t index, const std::string &propertyName) {
            return getPropertyAsync(objectArray, index, std::u16string(propertyName.cbegin(), propertyName.cend()));
        }

        inline FutureResult<void> MATLABEngine::setPropertyAsync(matlab::data::Array &objectArray, size_t index, const std::u16string &propertyName, const matlab::data::Array &property) {
            return setPropertyAsyncImpl(objectArray, index, propertyName, property, this, matlabHandle, false);
        }

        inline FutureResult<void> MATLABEngine::setPropertyAsync(matlab::data::Array &object, const std::string &propertyName, const matlab::data::Array &property) {
            return setPropertyAsync(object, std::u16string(propertyName.cbegin(), propertyName.cend()), property);
        }

        inline FutureResult<void> MATLABEngine::setPropertyAsync(matlab::data::Array &objectArray, size_t index, const std::string &propertyName, const matlab::data::Array &property) {
            return setPropertyAsync(objectArray, index, std::u16string(propertyName.cbegin(), propertyName.cend()), property);
        }
    }
}

#endif /* MATLAB_ENGINE_IMPL_HPP */
