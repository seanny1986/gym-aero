/* Copyright 2016-2017 The MathWorks, Inc. */

#ifndef ARRAY_FACTORY_HELPERS_HPP_
#define ARRAY_FACTORY_HELPERS_HPP_

#include "FunctionType.hpp"
#include "publish_util.hpp"
#include "ExceptionHelpers.hpp"

namespace matlab {
    namespace data {
        namespace impl {
            class ArrayFactoryImpl;
            class ArrayImpl;
        }
        namespace detail {
            class NameListImpl;
        }
    }
}

namespace matlab {
    namespace data {
        namespace detail {
            template <typename ItType,
                      typename T = typename std::remove_cv<typename std::iterator_traits<ItType>::value_type>::type>
            typename std::enable_if<std::is_arithmetic<T>::value && !std::is_same<T, bool>::value, TypedArray<typename GetReturnType<T>::type>>::type 
            createArrayWithIterator(matlab::data::impl::ArrayFactoryImpl* impl, ArrayDimensions dims, ItType begin, ItType end) {
                matlab::data::impl::ArrayImpl* aImpl = nullptr;
                typedef int(*CreateArrayWithDimsAndDataFcnPtr)(matlab::data::impl::ArrayFactoryImpl* impl, 
                                                               int arrayType,
                                                               size_t* dims, 
                                                               size_t numDims,
                                                               const void* const dataStart,
                                                               size_t numEl,
                                                               matlab::data::impl::ArrayImpl**);
                static const CreateArrayWithDimsAndDataFcnPtr fcn = resolveFunction<CreateArrayWithDimsAndDataFcnPtr>
                    (FunctionType::CREATE_ARRAY_WITH_DIMS_AND_DATA);
                throwIfError(fcn(impl,
                                 static_cast<int>(GetArrayType<T>::type),
                                 &dims[0], 
                                 dims.size(),
                                 &(*(begin)),
                                 (end - begin),
                                 &aImpl));
                return Access::createObj<TypedArray<typename GetReturnType<T>::type>>(aImpl);
            }

            template <typename ItType,
                      typename T = typename std::remove_cv<typename std::iterator_traits<ItType>::value_type>::type>
            typename std::enable_if<std::is_same<T, bool>::value, TypedArray<bool>>::type  
            createArrayWithIterator(matlab::data::impl::ArrayFactoryImpl* impl, ArrayDimensions dims, ItType begin, ItType end) {
                matlab::data::impl::ArrayImpl* aImpl = nullptr;
                typedef int(*CreateArrayWithDimsFcnPtr)(matlab::data::impl::ArrayFactoryImpl*, 
                                                        int,
                                                        size_t*, 
                                                        size_t,
                                                        matlab::data::impl::ArrayImpl**);
                static const CreateArrayWithDimsFcnPtr fcn = resolveFunction<CreateArrayWithDimsFcnPtr>
                    (FunctionType::CREATE_ARRAY_WITH_DIMS);
                throwIfError(fcn(impl,
                                 static_cast<int>(GetArrayType<T>::type),
                                 &dims[0],
                                 dims.size(),
                                 &aImpl));
                auto retVal = Access::createObj<TypedArray<T>>(aImpl);

                auto it = begin;
                for (auto& elem : retVal) {
                    elem = *it;
                    if (++it == end) {
                        break;
                    }
                }
                return retVal;
            }

            template <typename ItType,
                      typename T = typename std::remove_cv<typename std::iterator_traits<ItType>::value_type>::type>
            typename std::enable_if<matlab::data::is_complex<T>::value || std::is_same<T, String>::value || std::is_same<T, const char16_t*>::value, TypedArray<typename GetReturnType<T>::type>>::type  
            createArrayWithIterator(matlab::data::impl::ArrayFactoryImpl* impl, ArrayDimensions dims, ItType begin, ItType end) {
                matlab::data::impl::ArrayImpl* aImpl = nullptr;
                typedef int(*CreateArrayWithDimsFcnPtr)(matlab::data::impl::ArrayFactoryImpl*, 
                                                        int,
                                                        size_t*, 
                                                        size_t,
                                                        matlab::data::impl::ArrayImpl**);
                static const CreateArrayWithDimsFcnPtr fcn = resolveFunction<CreateArrayWithDimsFcnPtr>
                    (FunctionType::CREATE_ARRAY_WITH_DIMS);
                throwIfError(fcn(impl,
                                 static_cast<int>(GetArrayType<typename GetReturnType<T>::type>::type),
                                 &dims[0],
                                 dims.size(),
                                 &aImpl));
                auto retVal = Access::createObj<TypedArray<typename GetReturnType<T>::type>>(aImpl);

                auto it = begin;
                for (auto elem : retVal) {
                    elem = *it;
                    if (++it == end) {
                        break;
                    }
                }
                return retVal;
            }

            template <typename ItType,
                      typename T = typename std::remove_cv<typename std::iterator_traits<ItType>::value_type>::type>
            typename std::enable_if<std::is_same<T, Array>::value, TypedArray<T>>::type  
            createArrayWithIterator(matlab::data::impl::ArrayFactoryImpl* impl, ArrayDimensions dims, ItType begin, ItType end) {
                matlab::data::impl::ArrayImpl* aImpl = nullptr;
                typedef int(*CreateArrayWithDimsFcnPtr)(matlab::data::impl::ArrayFactoryImpl*, 
                                                        int,
                                                        size_t*, 
                                                        size_t,
                                                        matlab::data::impl::ArrayImpl**);
                static const CreateArrayWithDimsFcnPtr fcn = resolveFunction<CreateArrayWithDimsFcnPtr>
                    (FunctionType::CREATE_ARRAY_WITH_DIMS);
                throwIfError(fcn(impl,
                                 static_cast<int>(GetArrayType<T>::type),
                                 &dims[0],
                                 dims.size(),
                                 &aImpl));
                             
                auto retVal = Access::createObj<TypedArray<T>>(aImpl);

                auto it = begin;
                for (auto elem : retVal) {
                    elem = T(*it);
                    if (++it == end) {
                        break;
                    }
                }
                return retVal;
            }

            template <typename ItType,
                      typename T = typename std::remove_cv<typename std::iterator_traits<ItType>::value_type>::type>
            typename std::enable_if<std::is_same<T, const char *>::value || std::is_same<T, std::string>::value, TypedArray<MATLABString>>::type
            createArrayWithIterator(matlab::data::impl::ArrayFactoryImpl* impl, ArrayDimensions dims, ItType begin, ItType end) {                
                if (std::find_if(begin, end, [](std::string str) {return (isAscii7(str) == false); }) != end) {
                    throw NonAsciiCharInInputDataException(std::string("Input data can only contain ASCII characters"));
                }

                matlab::data::impl::ArrayImpl* aImpl = nullptr;
                typedef int(*CreateArrayWithDimsFcnPtr)(matlab::data::impl::ArrayFactoryImpl*, 
                                                        int,
                                                        size_t*, 
                                                        size_t,
                                                        matlab::data::impl::ArrayImpl**);
                static const CreateArrayWithDimsFcnPtr fcn = resolveFunction<CreateArrayWithDimsFcnPtr>
                    (FunctionType::CREATE_ARRAY_WITH_DIMS);
                throwIfError(fcn(impl,
                                 static_cast<int>(ArrayType::MATLAB_STRING),
                                 &dims[0],
                                 dims.size(),
                                 &aImpl));
                auto retVal = Access::createObj<TypedArray<MATLABString>>(aImpl);

                auto it = begin;
                for (auto elem : retVal) {
                    elem = *it;
                    if (++it == end) {
                        break;
                    }
                }
                return retVal;
            }


            template <typename ItType,
                typename T = typename std::remove_cv<typename std::iterator_traits<ItType>::value_type>::type>
                typename std::enable_if<std::is_same<T, MATLABString>::value, TypedArray<MATLABString>>::type
                createArrayWithIterator(matlab::data::impl::ArrayFactoryImpl* impl, ArrayDimensions dims, ItType begin, ItType end) {

                matlab::data::impl::ArrayImpl* aImpl = nullptr;
                typedef int(*CreateArrayWithDimsFcnPtr)(matlab::data::impl::ArrayFactoryImpl*, 
                                                        int,
                                                        size_t*, 
                                                        size_t,
                                                        matlab::data::impl::ArrayImpl**);
                static const CreateArrayWithDimsFcnPtr fcn = resolveFunction<CreateArrayWithDimsFcnPtr>
                    (FunctionType::CREATE_ARRAY_WITH_DIMS);
                throwIfError(fcn(impl,
                                 static_cast<int>(ArrayType::MATLAB_STRING),
                                 &dims[0],
                                 dims.size(),
                                 &aImpl));
                auto retVal = Access::createObj<TypedArray<MATLABString>>(aImpl);
                
                auto it = begin;
                for (auto elem : retVal) {
                    MATLABString x = *it;
                    if (x) {
                        elem = x;
                    }
                    if (++it == end) {
                        break;
                    }
                }
                return retVal;
            }

        }
    }
}

#endif
