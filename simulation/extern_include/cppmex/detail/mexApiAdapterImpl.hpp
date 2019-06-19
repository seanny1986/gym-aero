/* Copyright 2017-2018 The MathWorks, Inc. */

#ifndef __MEX_API_ADAPTER_HPP__
#define __MEX_API_ADAPTER_HPP__

#include "MatlabDataArray.hpp"
#include "MatlabDataArray/TypedArray.hpp"
#include "mex.hpp"
#include "mexEngineUtilImpl.hpp"
#include "mexExceptionType.hpp"
#include <vector>
#include <iostream>
#include "assert.h"
#include "../mexMatlabEngine.hpp"

LIBMWMEX_API_EXTERN_C{

    void mexApiFeval(void*, const int, void**, const int, void**, const char*, int*, void**, void*, void*, void(*)(void*, const char16_t*, size_t), void(*)(void*));

    void* mexApiGetVariable(void*, const char*, const char16_t*, int*, void**);

    void* mexApiSetVariable(void*, void*, const char*, const char16_t*, int*, void**);

    void* mexApiGetProperty(void*, void*, size_t, const char16_t*, int*, void**);

    void* mexApiSetProperty(void*, void*, size_t, const char16_t*, void*, int*, void**);

    uintptr_t mexApiFevalAsync(void*, const int, const int, void**, const char*, bool, void(*)(void*, int, bool, matlab::data::impl::ArrayImpl**), void(*)(void*, int, bool, int, void*), void*, void*, void*, void(*)(void*, const char16_t*, size_t), void(*)(void*));

    uintptr_t mexApiEvalAsync(void*, const char16_t*, void(*)(void*), void(*)(void*, int, void*), void*, void*, void*, void(*)(void*, const char16_t*, size_t), void(*)(void*));

    uintptr_t mexApiGetVariableAsync(void*, const char*, const char16_t*, void(*)(void*, int, bool, matlab::data::impl::ArrayImpl**), void(*)(void*, int, bool, int, void*), void*);

    uintptr_t mexApiSetVariableAsync(void*, void*, const char*, const char16_t*, void(*)(void*, int, bool, matlab::data::impl::ArrayImpl**), void(*)(void*, int, bool, int, void*), void*);

    uintptr_t mexApiGetPropertyAsync(void*, void*, size_t, const char16_t*, void(*)(void*, int, bool, matlab::data::impl::ArrayImpl**), void(*)(void*, int, bool, int, void*), void*);

    uintptr_t mexApiSetPropertyAsync(void*, void*, size_t, const char16_t*, void*, void(*)(void*, int, bool, matlab::data::impl::ArrayImpl**), void(*)(void*, int, bool, int, void*), void*);

    bool mexApiCancelFevalWithCompletion(uintptr_t, bool);
}

namespace matlab {
    namespace engine {
        namespace detail {


            template<typename T>
            inline void validateTIsSupported() {
                using U = typename std::remove_cv<typename std::remove_reference<T>::type>::type;
                static_assert(
                    std::is_same<U, bool>::value
                    || std::is_same<U, int>::value
                    || std::is_same<U, int8_t>::value
                    || std::is_same<U, int16_t>::value
                    || std::is_same<U, int32_t>::value
                    || std::is_same<U, int64_t>::value
                    || std::is_same<U, uint8_t>::value
                    || std::is_same<U, uint16_t>::value
                    || std::is_same<U, uint32_t>::value
                    || std::is_same<U, uint64_t>::value
                    || std::is_same<U, float>::value
                    || std::is_same<U, double>::value, "Attempted to use unsupported types.");
            }

            template<class T>
            matlab::data::Array createRhs(matlab::data::ArrayFactory& factory, T&& value) {
                validateTIsSupported<T>();
                using U = typename std::remove_cv<typename std::remove_reference<T>::type>::type;
                return factory.createArray<U>({ 1, 1 }, { value });
            }

            template<typename T, typename A>
            matlab::data::Array createRhs(matlab::data::ArrayFactory& factory, std::vector <T, A>&& value) {
                validateTIsSupported<T>();
                return factory.createArray({ 1, value.size() }, value.begin(), value.end());
            }

            template <std::size_t ...Ints>
            struct index_sequence {
                using value_type = std::size_t;
                static std::size_t size() { return sizeof...(Ints); }
            };

            template<std::size_t N, std::size_t... Values>
            struct make_index_sequence_impl {
                using type = typename make_index_sequence_impl<N - 1, Values..., sizeof...(Values)>::type;
            };

            template<std::size_t... Values>
            struct make_index_sequence_impl < 0, Values... > {
                using type = index_sequence < Values... >;
            };

            template<std::size_t N>
            using make_index_sequence = typename make_index_sequence_impl<N>::type;

            template<typename T>
            struct createLhs {
                static const size_t nlhs = 1;

                T operator()(std::vector<matlab::data::Array>&& lhs) const {
                    if (lhs.empty()) {
                        throw matlab::engine::TypeConversionException("The result is empty.");
                    }

                    T value;
                    try {
                        value = (*this)(matlab::data::TypedArray<T>(std::move(lhs.front())));
                    }
                    catch (const std::exception& e) {
                        throw matlab::engine::TypeConversionException(e.what());
                    }
                    return value;
                }

                T operator()(matlab::data::TypedArray<T> lhs) const {
                    validateTIsSupported<T>();
                    auto const begin = lhs.begin();
                    auto const end = lhs.end();
                    if (begin == end) {
                        throw matlab::engine::TypeConversionException("The result is empty.");
                    }
                    return *begin;
                }
            };

            template<>
            struct createLhs < void > {
                static const size_t nlhs = 0;
                void operator()(std::vector<matlab::data::Array>&& lhs) const {}
            };

            template<typename... TupleTypes>
            struct createLhs < std::tuple<TupleTypes...> > {
                static const size_t nlhs = sizeof...(TupleTypes);
                using T = std::tuple < TupleTypes... >;

                T operator()(std::vector<matlab::data::Array>&& lhs) const {
                    //we are not validating the LHS here as it can be any combinations of types for std::tuple.
                    if (lhs.size() < sizeof...(TupleTypes)) { throw std::runtime_error(""); }
                    return (*this)(std::move(lhs), detail::make_index_sequence<sizeof...(TupleTypes)>());
                }

            private:
                template<size_t Index>
                using TupleElement = typename std::remove_cv<typename std::remove_reference<typename std::tuple_element<Index, std::tuple<TupleTypes...> >::type>::type>::type;

                template<size_t... IndexList>
                std::tuple <TupleTypes...> operator()(std::vector<matlab::data::Array>&& lhs, detail::index_sequence<IndexList...>) const {
                    return std::tuple <TupleTypes...>(createLhs<TupleElement<IndexList>>()(std::move(lhs[IndexList]))...);
                }
            };
        }
    }
}

void implDeleter(matlab::data::impl::ArrayImpl** impl) {
    if (impl != nullptr)
        free(impl);
}

inline void writeToStreamBuffer(void* buffer, const char16_t* stream, size_t n) {
    std::shared_ptr<matlab::engine::StreamBuffer>* output = reinterpret_cast<std::shared_ptr<matlab::engine::StreamBuffer>*>(buffer);
    output->get()->sputn(stream, n);
}

inline void deleteStreamBufferImpl(void* impl) {
    delete static_cast<std::shared_ptr<matlab::engine::StreamBuffer>*>(impl);
}

/*** matlab::engine::MATLABEngine ***/
std::vector<matlab::data::Array> matlab::engine::MATLABEngine::feval(const std::u16string &function,
                                                                     const int nlhs,
                                                                     const std::vector<matlab::data::Array> &args,
                                                                     const std::shared_ptr<matlab::engine::StreamBuffer> &output,
                                                                     const std::shared_ptr<matlab::engine::StreamBuffer> &error) {
    int nrhs = static_cast<int>(args.size());

    matlab::data::impl::ArrayImpl** plhs_arr_impl = (nlhs == 0) ? nullptr : (matlab::data::impl::ArrayImpl**) malloc(sizeof(matlab::data::impl::ArrayImpl*) * nlhs);
    std::unique_ptr<matlab::data::impl::ArrayImpl*, void(*)(matlab::data::impl::ArrayImpl**)> plhs_impl_guard(plhs_arr_impl, &implDeleter);

    matlab::data::impl::ArrayImpl** args_arr_impl = (nrhs == 0) ? nullptr : (matlab::data::impl::ArrayImpl**) malloc(sizeof(matlab::data::impl::ArrayImpl*) * nrhs);
    std::unique_ptr<matlab::data::impl::ArrayImpl*, void(*)(matlab::data::impl::ArrayImpl**)> args_impl_guard(args_arr_impl, &implDeleter);

    void** plhs_impl = reinterpret_cast<void**>(plhs_arr_impl);
    void** args_impl = reinterpret_cast<void**>(args_arr_impl);

    arrayToImpl(nrhs, args_impl, args);

    void* mexcept = nullptr;

    std::string function_utf8 = matlab::engine::convertUTF16StringToUTF8String(function);

    void* output_ = output ? new std::shared_ptr<StreamBuffer>(std::move(output)) : nullptr;
    void* error_ = error ? new std::shared_ptr<StreamBuffer>(std::move(error)) : nullptr;

    int errID = 0;
    mexApiFeval(pImpl, nlhs, plhs_impl, nrhs, args_impl, function_utf8.c_str(), &errID, &mexcept, output_, error_, &writeToStreamBuffer, &deleteStreamBufferImpl);

    throwIfError(errID, mexcept);

    std::vector<matlab::data::Array> plhs;
    plhs.reserve(nlhs);
    implToArray(nlhs, plhs_impl, plhs);

    return plhs;
}

std::vector<matlab::data::Array> matlab::engine::MATLABEngine::feval(const std::string &function,
                                                                     const int nlhs,
                                                                     const std::vector<matlab::data::Array> &args,
                                                                     const std::shared_ptr<matlab::engine::StreamBuffer> &output,
                                                                     const std::shared_ptr<matlab::engine::StreamBuffer> &error) {
    return feval(std::u16string(function.begin(), function.end()), nlhs, args, output, error);
}

matlab::data::Array matlab::engine::MATLABEngine::feval(const std::u16string &function,
                                                        const std::vector<matlab::data::Array> &args,
                                                        const std::shared_ptr<matlab::engine::StreamBuffer> &output,
                                                        const std::shared_ptr<matlab::engine::StreamBuffer> &error) {

    std::vector<matlab::data::Array> out_vec = feval(function, 1, args, output, error);
    return out_vec.at(0);
}

matlab::data::Array matlab::engine::MATLABEngine::feval(const std::string &function,
                                                        const std::vector<matlab::data::Array> &args,
                                                        const std::shared_ptr<matlab::engine::StreamBuffer> &output,
                                                        const std::shared_ptr<matlab::engine::StreamBuffer> &error) {
    return feval(std::u16string(function.begin(), function.end()), args, output, error);
}

matlab::data::Array matlab::engine::MATLABEngine::feval(const std::u16string &function,
                                                        const matlab::data::Array &arg,
                                                        const std::shared_ptr<matlab::engine::StreamBuffer> &output,
                                                        const std::shared_ptr<matlab::engine::StreamBuffer> &error) {
    return feval(function, std::vector<matlab::data::Array>({ arg }), output, error);
}

matlab::data::Array matlab::engine::MATLABEngine::feval(const std::string &function,
                                                        const matlab::data::Array &arg,
                                                        const std::shared_ptr<matlab::engine::StreamBuffer> &output,
                                                        const std::shared_ptr<matlab::engine::StreamBuffer> &error) {
    return feval(std::u16string(function.begin(), function.end()), std::vector<matlab::data::Array>({arg}), output, error);
}

template<class ReturnType, typename...RhsArgs>
ReturnType matlab::engine::MATLABEngine::feval(const std::u16string &function,
                                               const std::shared_ptr<StreamBuffer> &output,
                                               const std::shared_ptr<StreamBuffer> &error,
                                               RhsArgs&&... rhsArgs) {
    matlab::data::ArrayFactory factory;
    std::vector<matlab::data::Array> rhsList({
        detail::createRhs(factory, std::forward<RhsArgs>(rhsArgs))...
    });

    auto const nlhs = detail::createLhs<ReturnType>::nlhs;
    std::vector<matlab::data::Array> f = feval(function, nlhs, rhsList, output, error);
    detail::createLhs<ReturnType> lhsFactory;
    return lhsFactory(std::move(f));
}

template<class ReturnType, typename...RhsArgs>
ReturnType matlab::engine::MATLABEngine::feval(const std::string &function,
                                               const std::shared_ptr<StreamBuffer> &output,
                                               const std::shared_ptr<StreamBuffer> &error,
                                               RhsArgs&&... rhsArgs) {
    return feval(std::u16string(function.begin(), function.end()), output, error, std::forward<RhsArgs>(rhsArgs)...);
}

template<class ReturnType, typename...RhsArgs>
ReturnType matlab::engine::MATLABEngine::feval(const std::u16string &function,
                                               RhsArgs&&... rhsArgs) {
    const std::shared_ptr<StreamBuffer> defaultStream;
    return feval<ReturnType>(function, defaultStream, defaultStream, std::forward<RhsArgs>(rhsArgs)...);
}

template<class ReturnType, typename...RhsArgs>
ReturnType matlab::engine::MATLABEngine::feval(const std::string &function,
                                               RhsArgs&&... rhsArgs) {
    return feval<ReturnType>(std::u16string(function.begin(), function.end()), std::forward<RhsArgs>(rhsArgs)...);
}

void matlab::engine::MATLABEngine::eval(const std::u16string &str,
                                        const std::shared_ptr<matlab::engine::StreamBuffer> &output,
                                        const std::shared_ptr<matlab::engine::StreamBuffer> &error) {
    matlab::data::ArrayFactory factory;
    auto mStr = factory.createScalar(str);

    feval(matlab::engine::convertUTF8StringToUTF16String("eval"), 0, std::vector<matlab::data::Array>({ mStr }), output, error);
}

matlab::data::Array matlab::engine::MATLABEngine::getVariable(const std::u16string &varName,
                                                              matlab::engine::WorkspaceType workspaceType) {
    const char* workspace = (workspaceType == matlab::engine::WorkspaceType::BASE) ? "base" : "global";
    void* mexcept = nullptr;

    int errID = 0;
    void* res = mexApiGetVariable(pImpl, workspace, varName.c_str(), &errID, &mexcept);

    throwIfError(errID, mexcept);

    matlab::data::Array ret = getArray(res);
    return ret;
}

matlab::data::Array matlab::engine::MATLABEngine::getVariable(const std::string &varName,
                                                              matlab::engine::WorkspaceType workspaceType) {
    return getVariable(std::u16string(varName.begin(), varName.end()), workspaceType);
}

void matlab::engine::MATLABEngine::setVariable(const std::u16string &varName,
                                               const matlab::data::Array &var,
                                               matlab::engine::WorkspaceType workspaceType) {
    const char* workspace = (workspaceType == matlab::engine::WorkspaceType::BASE) ? "base" : "global";
    void* mexcept = nullptr;

    void* varImpl = matlab::data::detail::Access::getImpl<matlab::data::impl::ArrayImpl>(var);

    int errID = 0;
    mexApiSetVariable(pImpl, varImpl, workspace, varName.c_str(), &errID, &mexcept);

    throwIfError(errID, mexcept);
}

void matlab::engine::MATLABEngine::setVariable(const std::string &varName,
                                               const matlab::data::Array &var,
                                               matlab::engine::WorkspaceType workspaceType) {
    setVariable(std::u16string(varName.begin(), varName.end()), var, workspaceType);
}

matlab::data::Array matlab::engine::MATLABEngine::getProperty(const matlab::data::Array &object,
                                                              size_t index,
                                                              const std::u16string &propertyName) {
    void* mexcept = nullptr;
    void* objImpl = matlab::data::detail::Access::getImpl<matlab::data::impl::ArrayImpl>(object);

    int errID = 0;
    void* impl = mexApiGetProperty(pImpl, objImpl, index, propertyName.c_str(), &errID, &mexcept);

    throwIfError(errID, mexcept);

    return matlab::data::detail::Access::createObj<matlab::data::Array>(reinterpret_cast<matlab::data::impl::ArrayImpl*>(impl));
}

matlab::data::Array matlab::engine::MATLABEngine::getProperty(const matlab::data::Array &object,
                                                              size_t index,
                                                              const std::string &propertyName) {
    return getProperty(object, index, std::u16string(propertyName.begin(), propertyName.end()));
}

matlab::data::Array matlab::engine::MATLABEngine::getProperty(const matlab::data::Array &object,
                                                              const std::u16string &propertyName) {
    return getProperty(object, 0, propertyName);
}

matlab::data::Array matlab::engine::MATLABEngine::getProperty(const matlab::data::Array &object,
                                                              const std::string &propertyName) {
    return getProperty(object, 0, std::u16string(propertyName.begin(), propertyName.end()));
}

void matlab::engine::MATLABEngine::setProperty(matlab::data::Array &object,
                                               size_t index,
                                               const std::u16string &propertyName,
                                               const matlab::data::Array &value) {
    void* mexcept = nullptr;
    void* objImpl = matlab::data::detail::Access::getImpl<matlab::data::impl::ArrayImpl>(object);
    void* propertyImpl = matlab::data::detail::Access::getImpl<matlab::data::impl::ArrayImpl>(value);

    int errID = 0;
    void* impl = mexApiSetProperty(pImpl, objImpl, index, propertyName.c_str(), propertyImpl, &errID, &mexcept);

    throwIfError(errID, mexcept);

    object = matlab::data::detail::Access::createObj<matlab::data::Array>(reinterpret_cast<matlab::data::impl::ArrayImpl*>(impl));
}

void matlab::engine::MATLABEngine::setProperty(matlab::data::Array &object,
                                               size_t index,
                                               const std::string &propertyName,
                                               const matlab::data::Array &value) {
    setProperty(object, index, std::u16string(propertyName.begin(), propertyName.end()), value);
}

void matlab::engine::MATLABEngine::setProperty(matlab::data::Array &object,
                                               const std::u16string &propertyName,
                                               const matlab::data::Array &value) {
    setProperty(object, 0, propertyName, value);
}

void matlab::engine::MATLABEngine::setProperty(matlab::data::Array &object,
                                               const std::string &propertyName,
                                               const matlab::data::Array &value) {
    setProperty(object, 0, std::u16string(propertyName.begin(), propertyName.end()), value);
}


namespace {
    template<typename T>
    void set_promise_exception(void *p, int excTypeNumber, void* msg){
        std::promise<T>* prom = reinterpret_cast<std::promise<T>*>(p);
        try {
            throwIfError(excTypeNumber, msg);
        }
        catch (const matlab::engine::MATLABSyntaxException& ex) {
            prom->set_exception(std::make_exception_ptr<matlab::engine::MATLABSyntaxException>(ex));
        }
        catch (const matlab::engine::MATLABExecutionException& ex) {
            prom->set_exception(std::make_exception_ptr<matlab::engine::MATLABExecutionException>(ex));
        }
        catch (const matlab::engine::CancelledException& ex) {
            prom->set_exception(std::make_exception_ptr<matlab::engine::CancelledException>(ex));
        }
        catch (const matlab::engine::InterruptedException& ex) {
            prom->set_exception(std::make_exception_ptr<matlab::engine::InterruptedException>(ex));
        }
        catch (const matlab::engine::MATLABException& ex) {
            prom->set_exception(std::make_exception_ptr<matlab::engine::MATLABException>(ex));
        }
        catch (const std::exception& ex) {
            prom->set_exception(std::make_exception_ptr<std::exception>(ex));
        }
        delete prom;
    }

    inline void set_eval_promise_data(void *p) {
        std::promise<void>* prom = reinterpret_cast<std::promise<void>*>(p);
        prom->set_value();
        delete prom;
    }

    inline void set_eval_promise_exception(void *p, int excTypeNumber, void* msg) {
        std::promise<void>* prom = reinterpret_cast<std::promise<void>*>(p);
        try {
            throwIfError(excTypeNumber, msg);
        }
        catch (const matlab::engine::MATLABSyntaxException& ex) {
            prom->set_exception(std::make_exception_ptr<matlab::engine::MATLABSyntaxException>(ex));
        }
        catch (const matlab::engine::MATLABExecutionException& ex) {
            prom->set_exception(std::make_exception_ptr<matlab::engine::MATLABExecutionException>(ex));
        }
        catch (const matlab::engine::CancelledException& ex) {
            prom->set_exception(std::make_exception_ptr<matlab::engine::CancelledException>(ex));
        }
        catch (const matlab::engine::InterruptedException& ex) {
            prom->set_exception(std::make_exception_ptr<matlab::engine::InterruptedException>(ex));
        }
        catch (const matlab::engine::MATLABException& ex) {
            prom->set_exception(std::make_exception_ptr<matlab::engine::MATLABException>(ex));
        }
        catch (const std::exception& ex) {
            prom->set_exception(std::make_exception_ptr<std::exception>(ex));
        }
        delete prom;
    }

    inline void set_feval_promise_data(void *p, int nlhs, bool straight, matlab::data::impl::ArrayImpl** plhs) {

        if (nlhs == 0 && straight) {
            std::promise<void>* prom = reinterpret_cast<std::promise<void>*>(p);
            prom->set_value();
            delete prom;
            return;
        }

        if (nlhs == 1 && straight) {
            std::promise<matlab::data::Array>* prom = reinterpret_cast<std::promise<matlab::data::Array>*>(p);
            matlab::data::Array v_ = matlab::data::detail::Access::createObj<matlab::data::Array>(plhs[0]);
            prom->set_value(v_);
            delete prom;
            return;
        }

        std::promise<std::vector<matlab::data::Array> >* prom = reinterpret_cast<std::promise<std::vector<matlab::data::Array> >*>(p);
        std::vector<matlab::data::Array> result;
        for (int i = 0; i < nlhs; i++) {
            matlab::data::Array v_ = matlab::data::detail::Access::createObj<matlab::data::Array>(plhs[i]);
            result.push_back(v_);
        }
        prom->set_value(result);
        delete prom;
    }

    template<class T>
    void set_exception(T p, std::exception_ptr e) {
        p->set_exception(e);
    }

    inline void set_feval_promise_exception(void *p, int nlhs, bool straight, int excTypeNumber, void* msg) {
        if (nlhs == 0 && straight) {
            set_promise_exception<void>(p, excTypeNumber, msg);
        }
        else if (nlhs == 1 && straight) {
            set_promise_exception<matlab::data::Array>(p, excTypeNumber, msg);
        }
        else {
            set_promise_exception<std::vector<matlab::data::Array>>(p, excTypeNumber, msg);
        }
    }
}

matlab::engine::FutureResult<std::vector<matlab::data::Array>> matlab::engine::MATLABEngine::fevalAsync(const std::u16string &function,
    const int nlhs,
    const std::vector<matlab::data::Array> &args,
    const std::shared_ptr<matlab::engine::StreamBuffer> &output,
    const std::shared_ptr<matlab::engine::StreamBuffer> &error) {

    int nrhs = static_cast<int>(args.size());

    matlab::data::impl::ArrayImpl** args_arr_impl = (nrhs == 0) ? nullptr : (matlab::data::impl::ArrayImpl**) malloc(sizeof(matlab::data::impl::ArrayImpl*) * nrhs);
    std::unique_ptr<matlab::data::impl::ArrayImpl*, void(*)(matlab::data::impl::ArrayImpl**)> args_impl_guard(args_arr_impl, &implDeleter);

    void** args_impl = reinterpret_cast<void**>(args_arr_impl);

    arrayToImpl(nrhs, args_impl, args);

    std::promise<std::vector<matlab::data::Array> >* p = new std::promise<std::vector<matlab::data::Array> >();
    std::future<std::vector<matlab::data::Array> > f = p->get_future();

    std::string function_utf8 = matlab::engine::convertUTF16StringToUTF8String(function);

    void* output_ = output ? new std::shared_ptr<StreamBuffer>(std::move(output)) : nullptr;
    void* error_ = error ? new std::shared_ptr<StreamBuffer>(std::move(error)) : nullptr;

    uintptr_t handle = mexApiFevalAsync(pImpl, nlhs, nrhs, args_impl, function_utf8.c_str(), false, &set_feval_promise_data, &set_feval_promise_exception, p, output_, error_, &writeToStreamBuffer, &deleteStreamBufferImpl);

    return matlab::engine::FutureResult<std::vector<matlab::data::Array>>(std::move(f), std::make_shared<matlab::engine::TaskReference>(handle, mexApiCancelFevalWithCompletion));
}


matlab::engine::FutureResult<std::vector<matlab::data::Array>> matlab::engine::MATLABEngine::fevalAsync(const std::string &function,
    const int nlhs,
    const std::vector<matlab::data::Array> &args,
    const std::shared_ptr<matlab::engine::StreamBuffer> &output,
    const std::shared_ptr<matlab::engine::StreamBuffer> &error) {
    return fevalAsync(std::u16string(function.begin(), function.end()), nlhs, args, output, error);
}


matlab::engine::FutureResult<matlab::data::Array> matlab::engine::MATLABEngine::fevalAsync(const std::u16string &function,
    const std::vector<matlab::data::Array> &args,
    const std::shared_ptr<matlab::engine::StreamBuffer> &output,
    const std::shared_ptr<matlab::engine::StreamBuffer> &error) {
    int nrhs = static_cast<int>(args.size());

    matlab::data::impl::ArrayImpl** args_arr_impl = (nrhs == 0) ? nullptr : (matlab::data::impl::ArrayImpl**) malloc(sizeof(matlab::data::impl::ArrayImpl*) * nrhs);
    std::unique_ptr<matlab::data::impl::ArrayImpl*, void(*)(matlab::data::impl::ArrayImpl**)> args_impl_guard(args_arr_impl, &implDeleter);

    void** args_impl = reinterpret_cast<void**>(args_arr_impl);

    arrayToImpl(nrhs, args_impl, args);

    std::promise<matlab::data::Array>* p = new std::promise<matlab::data::Array>();
    std::future<matlab::data::Array> f = p->get_future();

    std::string function_utf8 = matlab::engine::convertUTF16StringToUTF8String(function);

    void* output_ = output ? new std::shared_ptr<StreamBuffer>(std::move(output)) : nullptr;
    void* error_ = error ? new std::shared_ptr<StreamBuffer>(std::move(error)) : nullptr;

    uintptr_t handle = mexApiFevalAsync(pImpl, 1, nrhs, args_impl, function_utf8.c_str(), true, &set_feval_promise_data, &set_feval_promise_exception, p, output_, error_, &writeToStreamBuffer, &deleteStreamBufferImpl);

    return matlab::engine::FutureResult<matlab::data::Array>(std::move(f), std::make_shared<matlab::engine::TaskReference>(handle, mexApiCancelFevalWithCompletion));
}


matlab::engine::FutureResult<matlab::data::Array> matlab::engine::MATLABEngine::fevalAsync(const std::string &function,
    const std::vector<matlab::data::Array> &args,
    const std::shared_ptr<matlab::engine::StreamBuffer> &output,
    const std::shared_ptr<matlab::engine::StreamBuffer> &error) {
    return fevalAsync(std::u16string(function.begin(), function.end()), args, output, error);
}


matlab::engine::FutureResult<matlab::data::Array> matlab::engine::MATLABEngine::fevalAsync(const std::u16string &function,
    const matlab::data::Array &arg,
    const std::shared_ptr<matlab::engine::StreamBuffer> &output,
    const std::shared_ptr<matlab::engine::StreamBuffer> &error) {
    return fevalAsync(function, std::vector<matlab::data::Array>({ arg }), output, error);
}


matlab::engine::FutureResult<matlab::data::Array> matlab::engine::MATLABEngine::fevalAsync(const std::string &function,
    const matlab::data::Array &arg,
    const std::shared_ptr<matlab::engine::StreamBuffer> &output,
    const std::shared_ptr<matlab::engine::StreamBuffer> &error) {
    return fevalAsync(std::u16string(function.begin(), function.end()), arg, output, error);
}

template<class ReturnType, typename...RhsArgs>
matlab::engine::FutureResult<ReturnType> matlab::engine::MATLABEngine::fevalAsync(const std::u16string &function,
    const std::shared_ptr<matlab::engine::StreamBuffer> &output,
    const std::shared_ptr<matlab::engine::StreamBuffer> &error,
    RhsArgs&&... rhsArgs) {
    matlab::data::ArrayFactory factory;
    std::vector<matlab::data::Array> rhsList({
        detail::createRhs(factory, std::forward<RhsArgs>(rhsArgs))...
    });

    auto const nlhs = detail::createLhs<ReturnType>::nlhs;

    size_t nrhs = rhsList.size();
    matlab::data::impl::ArrayImpl** argsImpl = new matlab::data::impl::ArrayImpl*[nrhs];
    size_t i = 0;
    for (auto e : rhsList) {
        argsImpl[i++] = matlab::data::detail::Access::getImpl<matlab::data::impl::ArrayImpl>(e);
    }

    matlab::engine::FutureResult<std::vector<matlab::data::Array>> f = fevalAsync(function, nlhs, rhsList, output, error);

    // c++11 lambdas do not correctly handle move operations...
    // when c++14 is available, this should be:
    // auto convertToResultType = [copyableF = std::move(f)]()->ReturnType { ....... };
    auto copyableF = std::make_shared<FutureResult<std::vector<matlab::data::Array>>>(std::move(f));
    auto convertToResultType = [copyableF]() ->ReturnType {
        std::vector<matlab::data::Array> vec = copyableF->get();
        detail::createLhs<ReturnType> lhsFactory;
        return lhsFactory(std::move(vec));
    };

    std::future<ReturnType> future = std::async(std::launch::deferred, std::move(convertToResultType));
    return matlab::engine::FutureResult<ReturnType>(std::move(future), copyableF->getTaskReference());
}

template<class ReturnType, typename...RhsArgs>
matlab::engine::FutureResult<ReturnType> matlab::engine::MATLABEngine::fevalAsync(const std::string &function,
    const std::shared_ptr<matlab::engine::StreamBuffer> &output,
    const std::shared_ptr<matlab::engine::StreamBuffer> &error,
    RhsArgs&&... rhsArgs) {
    return fevalAsync<ReturnType>(std::u16string(function.begin(), function.end()), output, error, std::forward<RhsArgs>(rhsArgs)...);
}

template<class ReturnType, typename...RhsArgs>
matlab::engine::FutureResult<ReturnType> matlab::engine::MATLABEngine::fevalAsync(const std::u16string &function,
    RhsArgs&&... rhsArgs) {
    const std::shared_ptr<StreamBuffer> defaultStream;
    return fevalAsync<ReturnType>(function, defaultStream, defaultStream, std::forward<RhsArgs>(rhsArgs)...);
}

template<class ReturnType, typename...RhsArgs>
matlab::engine::FutureResult<ReturnType> matlab::engine::MATLABEngine::fevalAsync(const std::string &function,
    RhsArgs&&... rhsArgs) {
    return fevalAsync<ReturnType>(std::u16string(function.begin(), function.end()), std::forward<RhsArgs>(rhsArgs)...);
}

matlab::engine::FutureResult<void> matlab::engine::MATLABEngine::evalAsync(const std::u16string &str,
    const std::shared_ptr<matlab::engine::StreamBuffer> &output,
    const std::shared_ptr<matlab::engine::StreamBuffer> &error) {
    std::promise<void>* p = new std::promise<void>();
    std::future<void> f = p->get_future();
    void* output_ = output ? new std::shared_ptr<StreamBuffer>(std::move(output)) : nullptr;
    void* error_ = error ? new std::shared_ptr<StreamBuffer>(std::move(error)) : nullptr;
    uintptr_t handle = mexApiEvalAsync(pImpl, str.c_str(), &set_eval_promise_data, &set_eval_promise_exception, p, output_, error_, &writeToStreamBuffer, &deleteStreamBufferImpl);
    return matlab::engine::FutureResult<void>(std::move(f), std::make_shared<matlab::engine::TaskReference>(handle, mexApiCancelFevalWithCompletion));
}

matlab::engine::FutureResult<matlab::data::Array> matlab::engine::MATLABEngine::getVariableAsync(const std::u16string &varName,
    WorkspaceType workspaceType) {
    std::promise<matlab::data::Array>* p = new std::promise<matlab::data::Array>();
    std::future<matlab::data::Array> f = p->get_future();
    const char* workspace = (workspaceType == matlab::engine::WorkspaceType::BASE) ? "base" : "global";
    uintptr_t handle = mexApiGetVariableAsync(pImpl, workspace, varName.c_str(), &set_feval_promise_data, &set_feval_promise_exception, p);
    return matlab::engine::FutureResult<matlab::data::Array>(std::move(f), std::make_shared<matlab::engine::TaskReference>(handle, mexApiCancelFevalWithCompletion));
}

matlab::engine::FutureResult<matlab::data::Array> matlab::engine::MATLABEngine::getVariableAsync(const std::string &varName,
    WorkspaceType workspaceType) {
    return getVariableAsync(std::u16string(varName.begin(), varName.end()), workspaceType);
}

matlab::engine::FutureResult<void> matlab::engine::MATLABEngine::setVariableAsync(const std::u16string &varName,
    const matlab::data::Array &var,
    WorkspaceType workspaceType) {
    std::promise<void>* p = new std::promise<void>();
    std::future<void> f = p->get_future();
    const char* workspace = (workspaceType == matlab::engine::WorkspaceType::BASE) ? "base" : "global";
    matlab::data::impl::ArrayImpl* varImpl = matlab::data::detail::Access::getImpl<matlab::data::impl::ArrayImpl>(var);
    uintptr_t handle = mexApiSetVariableAsync(pImpl, varImpl, workspace, varName.c_str(), &set_feval_promise_data, &set_feval_promise_exception, p);
    return matlab::engine::FutureResult<void>(std::move(f), std::make_shared<matlab::engine::TaskReference>(handle, mexApiCancelFevalWithCompletion));
}

matlab::engine::FutureResult<void> matlab::engine::MATLABEngine::setVariableAsync(const std::string &varName,
    const matlab::data::Array &var,
    WorkspaceType workspaceType) {
    return setVariableAsync(std::u16string(varName.begin(), varName.end()), var, workspaceType);
}

matlab::engine::FutureResult<matlab::data::Array> matlab::engine::MATLABEngine::getPropertyAsync(const matlab::data::Array &object,
    size_t index,
    const std::u16string &propertyName) {
    std::promise<matlab::data::Array>* p = new std::promise<matlab::data::Array>();
    std::future<matlab::data::Array> f = p->get_future();
    matlab::data::impl::ArrayImpl* objectImpl = matlab::data::detail::Access::getImpl<matlab::data::impl::ArrayImpl>(object);
    uintptr_t handle = mexApiGetPropertyAsync(pImpl, objectImpl, index, propertyName.c_str(), &set_feval_promise_data, &set_feval_promise_exception, p);
    return matlab::engine::FutureResult<matlab::data::Array>(std::move(f), std::make_shared<matlab::engine::TaskReference>(handle, mexApiCancelFevalWithCompletion));
}

matlab::engine::FutureResult<matlab::data::Array> matlab::engine::MATLABEngine::getPropertyAsync(const matlab::data::Array &object,
    size_t index,
    const std::string &propertyName) {
    return getPropertyAsync(object, index, std::u16string(propertyName.begin(), propertyName.end()));
}

matlab::engine::FutureResult<matlab::data::Array> matlab::engine::MATLABEngine::getPropertyAsync(const matlab::data::Array &object,
    const std::u16string &propertyName) {
    return getPropertyAsync(object, 0, propertyName);
}

matlab::engine::FutureResult<matlab::data::Array> matlab::engine::MATLABEngine::getPropertyAsync(const matlab::data::Array &object,
    const std::string &propertyName) {
    return getPropertyAsync(object, std::u16string(propertyName.begin(), propertyName.end()));
}

matlab::engine::FutureResult<void> matlab::engine::MATLABEngine::setPropertyAsync(matlab::data::Array &object,
    size_t index,
    const std::u16string &propertyName,
    const matlab::data::Array &value) {
    std::promise<matlab::data::Array>* p = new std::promise<matlab::data::Array>();
    std::future<matlab::data::Array> f = p->get_future();
    matlab::data::impl::ArrayImpl* objectImpl = matlab::data::detail::Access::getImpl<matlab::data::impl::ArrayImpl>(object);
    matlab::data::impl::ArrayImpl* propImpl = matlab::data::detail::Access::getImpl<matlab::data::impl::ArrayImpl>(value);
    uintptr_t handle = mexApiSetPropertyAsync(pImpl, objectImpl, index, propertyName.c_str(), propImpl, &set_feval_promise_data, &set_feval_promise_exception, p);
    auto copyableF = std::make_shared<std::future<matlab::data::Array>>(std::move(f));
    auto convertToResultType = [&object, copyableF]() {
        matlab::data::Array vec = copyableF->get();
        object = std::move(vec);
    };
    std::future<void> future = std::async(std::launch::deferred, std::move(convertToResultType));
    return FutureResult<void>(std::move(future), std::make_shared<matlab::engine::TaskReference>(handle, mexApiCancelFevalWithCompletion));
}

matlab::engine::FutureResult<void> matlab::engine::MATLABEngine::setPropertyAsync(matlab::data::Array &object,
    size_t index,
    const std::string &propertyName,
    const matlab::data::Array &value) {
    return setPropertyAsync(object, index, std::u16string(propertyName.begin(), propertyName.end()), value);
}

matlab::engine::FutureResult<void> matlab::engine::MATLABEngine::setPropertyAsync(matlab::data::Array &object,
    const std::u16string &propertyName,
    const matlab::data::Array &value) {
    return setPropertyAsync(object, 0, propertyName, value);
}

matlab::engine::FutureResult<void> matlab::engine::MATLABEngine::setPropertyAsync(matlab::data::Array &object,
    const std::string &propertyName,
    const matlab::data::Array &value) {
    return setPropertyAsync(object, std::u16string(propertyName.begin(), propertyName.end()), value);
}

#endif
