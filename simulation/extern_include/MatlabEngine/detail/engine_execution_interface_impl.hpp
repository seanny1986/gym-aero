/* Copyright 2017 The MathWorks, Inc. */

#ifndef MATLAB_EXECUTION_INTERFACE_IMPL_HPP
#define MATLAB_EXECUTION_INTERFACE_IMPL_HPP

#include <vector>
#include <streambuf>
#include <memory>
#include <future>
#include <complex>
#include <MatlabEngine/cpp_engine_api.hpp>
#include <MatlabExecutionInterface/execution_interface.hpp>
#include <MatlabExecutionInterface/detail/value_future_impl.hpp>
#include <MatlabExecutionInterface/exception.hpp>
#include <MatlabExecutionInterface/task_reference.hpp>
#include <MatlabDataArray/detail/HelperFunctions.hpp>
#include <MatlabDataArray/StructArray.hpp>
#include <MatlabDataArray/StructRef.hpp>
#include <MatlabDataArray/Struct.hpp>
#include <MatlabDataArray/CharArray.hpp>
#include <MatlabDataArray/TypedArray.hpp>
#include <MatlabDataArray/Reference.hpp>


namespace {

    inline matlab::execution::MATLABExecutionException createMATLABExecutionException(const matlab::data::StructArray& mException);

    inline std::vector<matlab::execution::MATLABExecutionException> createCause(const matlab::data::CellArray& cause) {
        size_t nCauses = cause.getNumberOfElements();
        std::vector<matlab::execution::MATLABExecutionException> causes(nCauses);
        for (size_t i = 0; i < nCauses; i++) {
            matlab::data::Array exRef = cause[i];
            matlab::data::StructArray ex(exRef);
            causes[i] = createMATLABExecutionException(ex);
        }
        return causes;
    }

    inline std::vector<matlab::execution::StackFrame> createStackTrace(const matlab::data::StructArray& stack) {
        size_t nFrames = stack.getNumberOfElements();
        std::vector<matlab::execution::StackFrame> stackFrames(nFrames);

        for (size_t i = 0; i < nFrames; i++) {
            matlab::data::Array fileRef = stack[i]["File"];
            matlab::data::CharArray fileStr(fileRef);

            matlab::data::Array nameRef = stack[i]["Name"];
            matlab::data::CharArray nameStr(nameRef);

            matlab::data::Array lineRef = stack[i]["Line"];
            double line = lineRef[0];
            stackFrames[i] = matlab::execution::StackFrame(fileStr.toUTF16(), nameStr.toUTF16(), uint32_t(line));
        }

        return stackFrames;
    }

    inline matlab::execution::MATLABExecutionException createMATLABExecutionException(const matlab::data::StructArray& mException) {
        matlab::data::Array idRef = mException[0][std::string("identifier")];
        matlab::data::CharArray id(idRef);
        matlab::data::Array messageRef = mException[0][std::string("message")];
        matlab::data::CharArray message(messageRef);

        matlab::data::Array stackRef = mException[0][std::string("stack")];
        matlab::data::StructArray stack(stackRef);

        matlab::data::Array causeRef = mException[0][std::string("cause")];
        matlab::data::CellArray cause(causeRef);

        std::vector<matlab::execution::MATLABExecutionException> meCause = createCause(cause);
        std::vector<matlab::execution::StackFrame> meStack = createStackTrace(stack);
        return matlab::execution::MATLABExecutionException(id.toAscii(), message.toUTF16(), meStack, meCause);
    }

    inline matlab::execution::MATLABSyntaxException createMATLABSyntaxException(const matlab::data::StructArray& mException) {
        matlab::data::Array idRef = mException[0][std::string("identifier")];
        matlab::data::CharArray id(idRef);
        matlab::data::Array messageRef = mException[0][std::string("message")];
        matlab::data::CharArray message(messageRef);

        return matlab::execution::MATLABSyntaxException(id.toAscii(), message.toUTF16());
    }

    template<typename T>
    inline void set_promise_exception(void *p, size_t excTypeNumber, const void* msg) {
        std::promise<T>* prom = reinterpret_cast<std::promise<T>*>(p);
        ::detail::ExceptionType excType = static_cast<::detail::ExceptionType>(excTypeNumber);
        switch (excType) {
        case ::detail::ExceptionType::CANCELLED:{
            const char* message = reinterpret_cast<const char*>(msg);
            matlab::execution::CancelledException exception(message);
            prom->set_exception(std::make_exception_ptr<matlab::execution::CancelledException>(exception));
            break;
        }
        case ::detail::ExceptionType::INTERRUPTED: {
            const char* message = reinterpret_cast<const char*>(msg);
            matlab::execution::InterruptedException exception(message);
            prom->set_exception(std::make_exception_ptr<matlab::execution::InterruptedException>(exception));
            break;
        }
        case ::detail::ExceptionType::EXECUTION:
        case ::detail::ExceptionType::SYNTAX: {
            matlab::data::impl::ArrayImpl* exceptionImpl = const_cast<matlab::data::impl::ArrayImpl*>(reinterpret_cast<const matlab::data::impl::ArrayImpl*>(msg));
            matlab::data::Array mdaException = matlab::data::detail::Access::createObj<matlab::data::Array>(exceptionImpl);
            matlab::data::StructArray mException(mdaException);
            if (excType == ::detail::ExceptionType::SYNTAX) {
                matlab::execution::MATLABSyntaxException exception = createMATLABSyntaxException(mException);
                prom->set_exception(std::make_exception_ptr<matlab::execution::MATLABSyntaxException>(exception));
            }
            else {
                matlab::execution::MATLABExecutionException exception(createMATLABExecutionException(mException));
                prom->set_exception(std::make_exception_ptr<matlab::execution::MATLABExecutionException>(exception));
            }
            break;
        }
        case ::detail::ExceptionType::OTHER: {
            const char* message = reinterpret_cast<const char*>(msg);
            matlab::execution::Exception exception(message);
            prom->set_exception(std::make_exception_ptr<matlab::execution::Exception>(exception));
            break;
        }
        case ::detail::ExceptionType::STOPPED: {
            const char* message = reinterpret_cast<const char*>(msg);
            matlab::execution::MATLABNotAvailableException exception(message);
            prom->set_exception(std::make_exception_ptr<matlab::execution::MATLABNotAvailableException>(exception));
            break;
        }
        break;
        }
        delete prom;
    }
}

namespace matlab {
    namespace execution {

        inline ExecutionInterface::ExecutionInterface(uint64_t handle) : matlabHandle(handle) {
        }

        inline void set_eval_promise_data(void *p) {
            std::promise<void>* prom = reinterpret_cast<std::promise<void>*>(p);
            prom->set_value();
            delete prom;
        }
        
        inline void set_eval_promise_exception(void *p, size_t excTypeNumber, const void* msg) {
            set_promise_exception<void>(p, excTypeNumber, msg);
        }

        inline void set_feval_promise_data(void *p, size_t nlhs, bool straight, matlab::data::impl::ArrayImpl** plhs) {

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
            for (size_t i = 0; i < nlhs; i++) {
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

        inline void set_feval_promise_exception(void *p, size_t nlhs, bool straight, size_t excTypeNumber, const void* msg) {
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
        
        inline std::vector<matlab::data::Array> ExecutionInterface::feval(const std::u16string &function,
                                                             const size_t nlhs,
                                                             const std::vector<matlab::data::Array> &args,
                                                             const std::shared_ptr<StreamBuffer>& output,
                                                             const std::shared_ptr<StreamBuffer>& error) {
            return fevalAsync(function, nlhs, args, output, error).get();
        }

        inline std::vector<matlab::data::Array> ExecutionInterface::feval(const std::string &function,
                                                             const size_t nlhs,
                                                             const std::vector<matlab::data::Array> &args,
                                                             const std::shared_ptr<StreamBuffer>& output,
                                                             const std::shared_ptr<StreamBuffer>& error) {
            return feval(std::u16string(function.cbegin(), function.cend()), nlhs, args, output, error);
        }

        
        inline matlab::data::Array ExecutionInterface::feval(const std::u16string &function,
            const std::vector<matlab::data::Array> &args,
            const std::shared_ptr<StreamBuffer>& output,
            const std::shared_ptr<StreamBuffer>& error) {
            FutureResult<matlab::data::Array> future = fevalAsync(function, args, output, error);
            return future.get();
        }

        inline matlab::data::Array ExecutionInterface::feval(const std::string &function,
            const std::vector<matlab::data::Array> &args,
            const std::shared_ptr<StreamBuffer>& output,
            const std::shared_ptr<StreamBuffer>& error) {
            return feval(std::u16string(function.cbegin(), function.cend()), args, output, error);
        }
        
        inline matlab::data::Array ExecutionInterface::feval(const std::u16string &function,
        const matlab::data::Array &arg,
        const std::shared_ptr<StreamBuffer>& output,
        const std::shared_ptr<StreamBuffer>& error) {
            FutureResult<matlab::data::Array> future = fevalAsync(function, arg, output, error);
            return future.get();
        }

        inline matlab::data::Array ExecutionInterface::feval(const std::string &function,
        const matlab::data::Array &arg,
        const std::shared_ptr<StreamBuffer>& output,
        const std::shared_ptr<StreamBuffer>& error) {
            return feval(std::u16string(function.cbegin(), function.cend()), arg, output, error);
        }

        template<class ReturnType, typename...RhsArgs>
        ReturnType ExecutionInterface::feval(const std::u16string &function,
        const std::shared_ptr<StreamBuffer>& output,
        const std::shared_ptr<StreamBuffer>& error,
        RhsArgs&&... rhsArgs
        ) {
            return fevalAsync<ReturnType>(function, output, error, std::forward<RhsArgs>(rhsArgs)...).get();
        }
        
        template<class ReturnType, typename...RhsArgs>
        ReturnType ExecutionInterface::feval(const std::string &function,
        const std::shared_ptr<StreamBuffer>& output,
        const std::shared_ptr<StreamBuffer>& error,
        RhsArgs&&... rhsArgs
        ) {
            return feval<ReturnType>(std::u16string(function.cbegin(), function.cend()), 
                output, error, std::forward<RhsArgs>(rhsArgs)...);
        }

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
                return factory.createArray<U>({ 1, 1 }, {value});
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
                using type = index_sequence < Values... > ;
            };

            template<std::size_t N>
            using make_index_sequence = typename make_index_sequence_impl<N>::type;

            template<typename T>
            struct createLhs {
                static const size_t nlhs = 1;

                T operator()(std::vector<matlab::data::Array>&& lhs) const {
                    if (lhs.empty()) {
                        throw matlab::execution::TypeConversionException("The result is empty.");
                    }

                    T value;
                    try {
                        value = (*this)(matlab::data::TypedArray<T>(std::move(lhs.front())));
                    }
                    catch (const std::exception& e) {
                        throw matlab::execution::TypeConversionException(e.what());
                    }
                    return value;
                }

                T operator()(matlab::data::TypedArray<T> lhs) const {
                    validateTIsSupported<T>();
                    auto const begin = lhs.begin();
                    auto const end = lhs.end();
                    if (begin == end) {
                        throw matlab::execution::TypeConversionException("The result is empty.");
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
                using T = std::tuple < TupleTypes... > ;

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

        template<class ReturnType, typename...RhsArgs>
        ReturnType ExecutionInterface::feval(const std::u16string &function,
                              RhsArgs&&...rhsArgs
        ) {
            const std::shared_ptr<StreamBuffer> defaultStream;
            auto future = fevalAsync<ReturnType>(function, defaultStream, defaultStream, std::forward<RhsArgs>(rhsArgs)...);
            return future.get();
        }
        
        template<class ReturnType, typename...RhsArgs>
        ReturnType ExecutionInterface::feval(const std::string &function,
                              RhsArgs&&...rhsArgs
        ) {
            return feval<ReturnType>(std::u16string(function.cbegin(), function.cend()),
                std::forward<RhsArgs>(rhsArgs)...);
        }
        
        inline ExecutionInterface::~ExecutionInterface() {
            matlabHandle = 0;
        }

        inline FutureResult<std::vector<matlab::data::Array> > ExecutionInterface::fevalAsync(const std::u16string &function,
        const size_t nlhs,
        const std::vector<matlab::data::Array> &args,
        const std::shared_ptr<StreamBuffer> &output,
        const std::shared_ptr<StreamBuffer> &error
        ) {
            size_t nrhs = args.size();
            std::unique_ptr<matlab::data::impl::ArrayImpl*, void(*)(matlab::data::impl::ArrayImpl**)> argsImplPtr(new matlab::data::impl::ArrayImpl*[nrhs], [](matlab::data::impl::ArrayImpl** ptr) {
                delete[] ptr;
            });
            matlab::data::impl::ArrayImpl** argsImpl = argsImplPtr.get();
            size_t i = 0;
            for (auto e : args) {
                argsImpl[i++] = matlab::data::detail::Access::getImpl<matlab::data::impl::ArrayImpl>(e);
            }
            std::promise<std::vector<matlab::data::Array> >* p = new std::promise<std::vector<matlab::data::Array> >();
            std::future<std::vector<matlab::data::Array> > f = p->get_future();

            void* output_ = output ? new std::shared_ptr<StreamBuffer>(std::move(output)) : nullptr;
            void* error_ = error ? new std::shared_ptr<StreamBuffer>(std::move(error)) : nullptr;
            std::string utf8functionname = convertUTF16StringToASCIIString(function); 
            uintptr_t handle = cpp_engine_feval_with_completion(matlabHandle, utf8functionname.c_str(), nlhs, false, argsImpl, nrhs, &set_feval_promise_data, &set_feval_promise_exception, p, output_, error_, &writeToStreamBuffer, &deleteStreamBufferImpl);

            return FutureResult<std::vector<matlab::data::Array>>(std::move(f), std::make_shared<TaskReference>(handle, cpp_engine_cancel_feval_with_completion ));
        }

        inline FutureResult<std::vector<matlab::data::Array> > ExecutionInterface::fevalAsync(const std::string &function,
        const size_t nlhs,
        const std::vector<matlab::data::Array> &args,
        const std::shared_ptr<StreamBuffer> &output,
        const std::shared_ptr<StreamBuffer> &error
        ) {
            return fevalAsync(std::u16string(function.cbegin(), function.cend()), nlhs, args, output, error);
        }
        
        inline FutureResult<matlab::data::Array> ExecutionInterface::fevalAsync(const std::u16string &function,
        const std::vector<matlab::data::Array> &args,
        const std::shared_ptr<StreamBuffer> &output,
        const std::shared_ptr<StreamBuffer> &error
        ) {
            size_t nrhs = args.size();
            std::unique_ptr<matlab::data::impl::ArrayImpl*, void(*)(matlab::data::impl::ArrayImpl**)> argsImplPtr(new matlab::data::impl::ArrayImpl*[nrhs], [](matlab::data::impl::ArrayImpl** ptr) {
                delete[] ptr;
            });
            matlab::data::impl::ArrayImpl** argsImpl = argsImplPtr.get();
            size_t i = 0;
            for (auto e : args) {
                argsImpl[i++] = matlab::data::detail::Access::getImpl<matlab::data::impl::ArrayImpl>(e);
            }
            std::promise<matlab::data::Array>* p = new std::promise<matlab::data::Array>();
            std::future<matlab::data::Array> f = p->get_future();

            void* output_ = output ? new std::shared_ptr<StreamBuffer>(std::move(output)) : nullptr;
            void* error_ = error ? new std::shared_ptr<StreamBuffer>(std::move(error)) : nullptr;
            
            std::string utf8functionname = convertUTF16StringToASCIIString(function); 
            uintptr_t handle = cpp_engine_feval_with_completion(matlabHandle, utf8functionname.c_str(), 1, true, argsImpl, nrhs, &set_feval_promise_data, &set_feval_promise_exception, p, output_, error_, &writeToStreamBuffer, &deleteStreamBufferImpl);

            return FutureResult<matlab::data::Array>(std::move(f), std::make_shared<TaskReference>(handle, cpp_engine_cancel_feval_with_completion));
        }

        inline FutureResult<matlab::data::Array> ExecutionInterface::fevalAsync(const std::string &function,
        const std::vector<matlab::data::Array> &args,
        const std::shared_ptr<StreamBuffer> &output,
        const std::shared_ptr<StreamBuffer> &error
        ) {
            return fevalAsync(std::u16string(function.cbegin(), function.cend()), args, output, error);
        }
        
        inline FutureResult<matlab::data::Array> ExecutionInterface::fevalAsync(const std::u16string &function,
        const matlab::data::Array &arg,
        const std::shared_ptr<StreamBuffer> &output,
        const std::shared_ptr<StreamBuffer> &error
        ) {
            return fevalAsync(function, std::vector<matlab::data::Array>({ arg }), output, error);
        }

        inline FutureResult<matlab::data::Array> ExecutionInterface::fevalAsync(const std::string &function,
        const matlab::data::Array &arg,
        const std::shared_ptr<StreamBuffer> &output,
        const std::shared_ptr<StreamBuffer> &error
        ) {
            return fevalAsync(std::u16string(function.cbegin(), function.cend()), arg, output, error);
        }

        template<class ReturnType, typename... RhsArgs>
        FutureResult<ReturnType> ExecutionInterface::fevalAsync(const std::u16string &function,
            const std::shared_ptr<StreamBuffer> &output,
            const std::shared_ptr<StreamBuffer> &error,
            RhsArgs&&... rhsArgs
            ) {
            matlab::data::ArrayFactory factory;
            std::vector<matlab::data::Array> rhsList({
                detail::createRhs(factory, std::forward<RhsArgs>(rhsArgs))...
            });

            auto const nlhs = detail::createLhs<ReturnType>::nlhs;

            size_t nrhs = rhsList.size();
            std::unique_ptr<matlab::data::impl::ArrayImpl*, void(*)(matlab::data::impl::ArrayImpl**)> argsImplPtr(new matlab::data::impl::ArrayImpl*[nrhs], [](matlab::data::impl::ArrayImpl** ptr) {
                delete[] ptr;
            });
            matlab::data::impl::ArrayImpl** argsImpl = argsImplPtr.get();
            size_t i = 0;
            for (auto e : rhsList) {
                argsImpl[i++] = matlab::data::detail::Access::getImpl<matlab::data::impl::ArrayImpl>(e);
            }

            FutureResult<std::vector<matlab::data::Array>> f = fevalAsync(function, nlhs, rhsList, output, error);

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
            return FutureResult<ReturnType>(std::move(future), copyableF->getTaskReference());
        }

        template<class ReturnType, typename... RhsArgs>
        FutureResult<ReturnType> ExecutionInterface::fevalAsync(const std::string &function,
            const std::shared_ptr<StreamBuffer> &output,
            const std::shared_ptr<StreamBuffer> &error,
            RhsArgs&&... rhsArgs
            ) {
            return convertUTF8StringToUTF16String(std::u16string(function.cbegin(), function.cend()), 
                output, error, std::forward<RhsArgs>(rhsArgs)...);
        }

        template<class ReturnType, typename... RhsArgs>
        FutureResult<ReturnType> ExecutionInterface::fevalAsync(const std::u16string &function,
            RhsArgs&&... rhsArgs
            ) {
            const std::shared_ptr<StreamBuffer> defaultBuffer;
            return fevalAsync<ReturnType>(function, defaultBuffer, defaultBuffer, std::forward<RhsArgs>(rhsArgs)...);
        }
        
        template<class ReturnType, typename... RhsArgs>
        FutureResult<ReturnType> ExecutionInterface::fevalAsync(const std::string &function,
            RhsArgs&&... rhsArgs
            ) {
            return fevalAsync<ReturnType>(std::u16string(function.cbegin(), function.cend()), 
                std::forward<RhsArgs>(rhsArgs)...);
        }
        
        inline std::string ExecutionInterface::convertUTF16StringToASCIIString(const std::u16string &str)
        {
            std::unique_ptr<char []> asciistr_ptr(new char[str.size()+1]);
            asciistr_ptr.get()[str.size()] = '\0';
            const char* u16_src = reinterpret_cast<const char*>(str.c_str());
            for(size_t n = 0; n < str.size(); ++n)
            {   
               asciistr_ptr.get()[n] = u16_src[2*n];
            }
            return std::string(asciistr_ptr.get());
        }


        inline void writeToStreamBuffer(void* buffer, const char16_t* stream, size_t n) {
            std::shared_ptr<StreamBuffer>* output = reinterpret_cast<std::shared_ptr<StreamBuffer>*>(buffer);
            output->get()->sputn(stream, n);
        }

        inline void deleteStreamBufferImpl(void* impl) {
            delete static_cast<std::shared_ptr<StreamBuffer>*>(impl);
        }
    }
}

#endif /* MATLAB_EXECUTION_INTERFACE_IMPL_HPP */
