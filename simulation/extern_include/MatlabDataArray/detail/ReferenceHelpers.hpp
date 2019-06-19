/* Copyright 2016-2017 The MathWorks, Inc. */

#ifndef MATLAB_EXTDATA_REFERENCE_HELPERS_HPP
#define MATLAB_EXTDATA_REFERENCE_HELPERS_HPP

#include "publish_util.hpp"
#include "FunctionType.hpp"

#include "StringHelpers.hpp"
#include "exception_interface.hpp"
#include "ExceptionHelpers.hpp"
#include "FunctionType.hpp"


#include <stdexcept>

namespace matlab {
    namespace data {
        template <typename T> class Reference;
        
        namespace detail {

            template <typename T, typename U>
            inline typename std::enable_if<std::is_base_of<Array, U>::value && !std::is_same<Struct, T>::value>::type validateReference(ReferenceImpl* impl, bool unshare) {
                typedef int(*TypedArrayReferenceValidateTypeFcnPtr)(ReferenceImpl*, int, bool);
                static const TypedArrayReferenceValidateTypeFcnPtr fcn = resolveFunction<TypedArrayReferenceValidateTypeFcnPtr>
                    (FunctionType::TYPED_ARRAY_REFERENCE_VALIDATE_TYPE);
                throwIfError(fcn(impl, static_cast<int>(U::type), false));
            }
                
            template <typename T, typename U>
            inline typename std::enable_if<std::is_base_of<Array, U>::value && std::is_same<Struct, T>::value>::type validateReference(ReferenceImpl* impl, bool unshare) {
                throwIfError(static_cast<int>(ExceptionType::InvalidDataType));
            }

            template <typename T, typename U>
            inline typename std::enable_if<!std::is_base_of<Array, U>::value>::type validateReference(ReferenceImpl* impl, bool unshare) {
                static_assert(std::is_same<T, U>::value, "Reference type must match array type");
                typedef int(*ReferenceValidateIndexFcnPtr)(ReferenceImpl*);
                static const ReferenceValidateIndexFcnPtr fcn = resolveFunction<ReferenceValidateIndexFcnPtr>
                    (FunctionType::REFERENCE_VALIDATE_INDEX);
                throwIfError(fcn(impl));
            }

            template <typename T>
            inline typename std::enable_if<std::is_base_of<Array, T>::value>::type validateUntypedReference(ReferenceImpl* impl) {
                typedef int(*TypedArrayReferenceValidateTypeFcnPtr)(ReferenceImpl*, int, bool);
                static const TypedArrayReferenceValidateTypeFcnPtr fcn = resolveFunction<TypedArrayReferenceValidateTypeFcnPtr>
                    (FunctionType::TYPED_ARRAY_REFERENCE_VALIDATE_TYPE);
                throwIfError(fcn(impl, static_cast<int>(T::type), false));
            }
                
            template <typename T>
            inline typename std::enable_if<!std::is_base_of<Array, T>::value>::type validateUntypedReference(ReferenceImpl* impl) {
                typedef int(*ReferenceValidateTypeFcnPtr)(ReferenceImpl*, int);
                static const ReferenceValidateTypeFcnPtr fcn = resolveFunction<ReferenceValidateTypeFcnPtr>
                    (FunctionType::REFERENCE_VALIDATE_TYPE);
                throwIfError(fcn(impl, static_cast<int>(GetArrayType<T>::type)));
            }

            template <typename T>
            inline typename std::enable_if<!std::is_base_of<Array, T>::value>::type validateUntypedArrayReference(ReferenceImpl* impl) {
                typedef int(*TypedArrayReferenceValidateTypeFcnPtr)(ReferenceImpl*, int, bool);
                static const TypedArrayReferenceValidateTypeFcnPtr fcn = resolveFunction<TypedArrayReferenceValidateTypeFcnPtr>
                    (FunctionType::TYPED_ARRAY_REFERENCE_VALIDATE_TYPE);
                throwIfError(fcn(impl, static_cast<int>(GetArrayType<T>::type), false));
            }
            template <typename T>
            inline typename std::enable_if<std::is_base_of<Array, T>::value>::type validateUntypedArrayReference(ReferenceImpl* impl) {
                typedef int(*TypedArrayReferenceValidateTypeFcnPtr)(ReferenceImpl*, int, bool);
                static const TypedArrayReferenceValidateTypeFcnPtr fcn = resolveFunction<TypedArrayReferenceValidateTypeFcnPtr>
                    (FunctionType::TYPED_ARRAY_REFERENCE_VALIDATE_TYPE);
                throwIfError(fcn(impl, static_cast<int>(GetArrayType<T>::type), false));
            }

            template <typename T>
            inline T castToType(void* value, ArrayType type) {
                switch (type) {
                  case ArrayType::LOGICAL: return static_cast<T>(*static_cast<bool*>(value));
                  case ArrayType::DOUBLE: return static_cast<T>(*static_cast<double*>(value));
                  case ArrayType::SINGLE: return static_cast<T>(*static_cast<float*>(value));
                  case ArrayType::INT64: return static_cast<T>(*static_cast<int64_t*>(value));
                  case ArrayType::INT32: return static_cast<T>(*static_cast<int32_t*>(value));
                  case ArrayType::INT16: return static_cast<T>(*static_cast<int16_t*>(value));
                  case ArrayType::INT8: return static_cast<T>(*static_cast<int8_t*>(value));
                  case ArrayType::UINT64: return static_cast<T>(*static_cast<uint64_t*>(value));
                  case ArrayType::UINT32: return static_cast<T>(*static_cast<uint32_t*>(value));
                  case ArrayType::UINT16: return static_cast<T>(*static_cast<uint16_t*>(value));
                  case ArrayType::UINT8: return static_cast<T>(*static_cast<uint8_t*>(value));
                  default:
                    throw TypeMismatchException(std::string("Can't convert this element"));
                }
                return T();
            }

            template <>
            inline bool castToType<bool>(void* value, ArrayType type) {
                switch (type) {
                  case ArrayType::LOGICAL: return *static_cast<bool*>(value);
                  case ArrayType::DOUBLE: return *static_cast<double*>(value) != 0;
                  case ArrayType::SINGLE: return *static_cast<float*>(value) != 0;
                  case ArrayType::INT64: return *static_cast<int64_t*>(value) != 0;
                  case ArrayType::INT32: return *static_cast<int32_t*>(value) != 0;
                  case ArrayType::INT16: return *static_cast<int16_t*>(value) != 0;
                  case ArrayType::INT8: return *static_cast<int8_t*>(value) != 0;
                  case ArrayType::UINT64: return *static_cast<uint64_t*>(value) != 0;
                  case ArrayType::UINT32: return *static_cast<uint32_t*>(value) != 0;
                  case ArrayType::UINT16: return *static_cast<uint16_t*>(value) != 0;
                  case ArrayType::UINT8: return *static_cast<uint8_t*>(value) != 0;
                  default:
                    throw TypeMismatchException(std::string("Can't convert this element"));
                }
                return false;
            }

            template <>
            inline CHAR16_T castToType<CHAR16_T>(void* value, ArrayType type) {
                switch (type) {
                  case ArrayType::CHAR: return *static_cast<CHAR16_T*>(value);
                  default:
                    throw TypeMismatchException(std::string("Can't convert this element"));
                }
                return false;
            }

            template <typename U>
            inline typename std::enable_if<std::is_arithmetic<U>::value, U&>::type getElement(std::shared_ptr<ReferenceImpl> impl) {
                void* value = nullptr;
                typedef int(*TypedReferenceGetPodValueFcnPtr)(ReferenceImpl*,
                                                              void**);
                static const TypedReferenceGetPodValueFcnPtr fcn = resolveFunction<TypedReferenceGetPodValueFcnPtr>
                    (FunctionType::TYPED_REFERENCE_GET_POD_VALUE);
                throwIfError(fcn(impl.get(), &value));
                return *(static_cast<U*>(value));
            }

            template <typename U>
            inline typename std::enable_if<is_complex<U>::value, U>::type getElement(std::shared_ptr<ReferenceImpl> impl) {                
                void* real = nullptr;
                void* imag = nullptr;
                typedef int(*TypedReferenceGetComplexValueFcnPtr)(ReferenceImpl*,
                                                                  void**,
                                                                  void**);
                static const TypedReferenceGetComplexValueFcnPtr fcn = resolveFunction<TypedReferenceGetComplexValueFcnPtr>
                    (FunctionType::TYPED_REFERENCE_GET_COMPLEX_VALUE);
                throwIfError(fcn(impl.get(),
                                 &real,
                                 &imag));

                typename U::value_type r = *(static_cast<typename U::value_type*>(real));
                typename U::value_type i = *(static_cast<typename U::value_type*>(imag));

                return U(r,i);
            }

            template <typename U>
            inline typename std::enable_if<std::is_same<MATLABString, U>::value, MATLABString>::type getElement(std::shared_ptr<ReferenceImpl> impl) {
                const char16_t* str = nullptr;
                size_t strlen = 0;
                typedef int(*StringGetValueFcnPtr)(ReferenceImpl*, const char16_t**, size_t*);
                static const StringGetValueFcnPtr fcn = resolveFunction<StringGetValueFcnPtr>
                    (FunctionType::STRING_GET_VALUE);
                throwIfError(fcn(impl.get(), &str, &strlen));
                if (str != nullptr) {
                    return MATLABString(String(str, strlen));
                }

                return MATLABString();
            }

            template <typename U>
            inline typename std::enable_if<std::is_same<Array, U>::value, Array>::type getElement(std::shared_ptr<ReferenceImpl> impl) {
                impl::ArrayImpl* arr_impl = nullptr;
                typedef void(*ArrayReferenceSharedCopyFcnPtr)(ReferenceImpl*,
                                                              matlab::data::impl::ArrayImpl**);
                static const ArrayReferenceSharedCopyFcnPtr fcn = resolveFunction<ArrayReferenceSharedCopyFcnPtr>
                    (FunctionType::ARRAY_REFERENCE_SHARED_COPY);
                fcn(impl.get(), &arr_impl);
                return Access::createObj<U>(arr_impl);
            }

            template <typename U>
            inline typename std::enable_if<std::is_same<Struct, U>::value, Struct>::type getElement(std::shared_ptr<ReferenceImpl> impl) {
                return Access::createObj<U>(impl);
            }

            template <typename U>
            inline typename std::enable_if<std::is_arithmetic<U>::value>::type setElement(ReferenceImpl* impl, U rhs, int type) {
                typedef int(*TypedReferenceSetPodValueFcnPtr)(ReferenceImpl*, int, void*);
                static const TypedReferenceSetPodValueFcnPtr fcn = resolveFunction<TypedReferenceSetPodValueFcnPtr>
                    (FunctionType::TYPED_REFERENCE_SET_POD_VALUE);
                throwIfError(fcn(impl, type, &rhs));
            }

            template <typename U>
            inline typename std::enable_if<is_complex<U>::value>::type setElement(ReferenceImpl* impl, U rhs, int type) {
                typename U::value_type real = rhs.real();
                typename U::value_type imag = rhs.imag();

                typedef int(*TypedReferenceSetComplexValueFcnPtr)(ReferenceImpl*, int, void*, void*);
                static const TypedReferenceSetComplexValueFcnPtr fcn = resolveFunction<TypedReferenceSetComplexValueFcnPtr>
                    (FunctionType::TYPED_REFERENCE_SET_COMPLEX_VALUE);
                throwIfError(fcn(impl, type, &real, &imag));
            }

            template <typename U>
            inline typename std::enable_if<std::is_same<String, U>::value>::type setElement(ReferenceImpl* impl, U rhs, int type) {
                typedef int(*ReferenceSetChar16StringFcnPtr)(ReferenceImpl*, const char16_t*, size_t);
                static const ReferenceSetChar16StringFcnPtr fcn = resolveFunction<ReferenceSetChar16StringFcnPtr>
                    (FunctionType::REFERENCE_SET_CHAR16_STRING);
                throwIfError(fcn(impl, rhs.c_str(), rhs.size()));
            }

            template <typename U>
            inline typename std::enable_if<std::is_same<std::string, U>::value>::type setElement(ReferenceImpl* impl, U rhs, int type) {
                typedef int(*ReferenceSetStringFcnPtr)(ReferenceImpl*, const char*, size_t);
                static const ReferenceSetStringFcnPtr fcn = resolveFunction<ReferenceSetStringFcnPtr>
                    (FunctionType::REFERENCE_SET_STRING);
                throwIfError(fcn(impl, rhs.c_str(), rhs.size()));
            }

            template <typename U>
            inline typename std::enable_if<std::is_base_of<Array, U>::value>::type setElement(ReferenceImpl* impl, U rhs, int type) {
                typedef int(*ReferenceSetReferenceValueFcnPtr)(ReferenceImpl*, matlab::data::impl::ArrayImpl*);
                static const ReferenceSetReferenceValueFcnPtr fcn = resolveFunction<ReferenceSetReferenceValueFcnPtr>
                    (FunctionType::REFERENCE_SET_REFERENCE_VALUE);
                throwIfError(fcn(impl, Access::getImpl<matlab::data::impl::ArrayImpl>(rhs)));
            }

            template <typename U>
            inline typename std::enable_if<std::is_same<MATLABString, U>::value>::type setElement(ReferenceImpl* impl, U rhs, int type) {
                if (rhs) {
                    String elem = *rhs;
                    typedef int(*ReferenceSetChar16StringFcnPtr)(ReferenceImpl*, const char16_t*, size_t);
                    static const ReferenceSetChar16StringFcnPtr fcn = resolveFunction<ReferenceSetChar16StringFcnPtr>
                        (FunctionType::REFERENCE_SET_CHAR16_STRING);
                    throwIfError(fcn(impl, elem.c_str(), elem.size()));
                }
                else {
                    typedef int(*ReferenceSetMissingChar16StringFcnPtr)(ReferenceImpl*);
                    static const ReferenceSetMissingChar16StringFcnPtr fcn = resolveFunction<ReferenceSetMissingChar16StringFcnPtr>
                        (FunctionType::REFERENCE_SET_MISSING_CHAR16_STRING);
                    throwIfError(fcn(impl));
                }
            }

            template <typename U>
            inline typename std::enable_if<!std::is_same<Enumeration, U>::value &&
                                           !std::is_same<MATLABString, U>::value, std::string>::type getString(ReferenceImpl* impl) {
                throwIfError(static_cast<int>(ExceptionType::InvalidDataType));
            }

            template <typename U>
            inline typename std::enable_if<std::is_same<Enumeration, U>::value, std::string>::type getString(ReferenceImpl* impl) {
                const char* str = nullptr;
                size_t strlen = 0;
                typedef int(*EnumArrayGetValueFcnPtr)(ReferenceImpl*, const char**, size_t*);
                static const EnumArrayGetValueFcnPtr fcn = resolveFunction<EnumArrayGetValueFcnPtr>
                    (FunctionType::ENUM_ARRAY_GET_VALUE);
                throwIfError(fcn(impl, &str, &strlen));
                return std::string(str, strlen);
            }

            template <typename U>
            inline typename std::enable_if<std::is_same<MATLABString, U>::value, std::string>::type getString(ReferenceImpl* impl) {
                const char16_t* str = nullptr;
                size_t strlen = 0;
                bool missing = false;
                typedef int(*StringGetValueFcnPtr)(ReferenceImpl* impl, const char16_t**, size_t*);
                static const StringGetValueFcnPtr fcn = resolveFunction<StringGetValueFcnPtr>
                    (FunctionType::STRING_GET_VALUE);
                throwIfError(fcn(impl, &str, &strlen));

                if (str == nullptr) {
                    throw std::runtime_error("Missing string");
                }
                String temp(str, strlen);
                if (!isAscii7(temp)) {
                    throw NonAsciiCharInInputDataException(std::string("Input data can only contain ASCII characters"));
                }
                return std::string(temp.begin(), temp.end());
            }

            template <typename U>
            inline typename std::enable_if<!std::is_same<Enumeration, U>::value &&
                                           !std::is_same<MATLABString, U>::value>::type setString(ReferenceImpl* impl, std::string rhs) {
                throwIfError(static_cast<int>(ExceptionType::InvalidDataType));
            }

            template <typename U>
            inline typename std::enable_if<std::is_same<Enumeration, U>::value>::type setString(ReferenceImpl* impl, std::string rhs) {
                typedef int(*EnumArraySetValueFcnPtr)(ReferenceImpl*, const char*, size_t);
                static const EnumArraySetValueFcnPtr fcn = resolveFunction<EnumArraySetValueFcnPtr>
                    (FunctionType::ENUM_ARRAY_SET_VALUE);
                throwIfError(fcn(impl, rhs.c_str(), rhs.size()));
            }
            
            template <typename U>
            inline typename std::enable_if<std::is_same<MATLABString, U>::value>::type setString(ReferenceImpl* impl, std::string rhs) {
                if (!isAscii7(rhs)) {
                    throw NonAsciiCharInInputDataException(std::string("Input data can only contain ASCII characters"));
                }
                String tmp(rhs.begin(), rhs.end());
                setElement<String>(impl, std::move(tmp), 0);
            }

            template <typename U>
            inline typename std::enable_if<std::is_same<MATLABString, U>::value>::type setString(ReferenceImpl* impl, String rhs) {
                setElement<String>(impl, std::move(rhs), 0);
            }

            template <typename U>
            inline typename std::enable_if<std::is_same<MATLABString, U>::value>::type setString(ReferenceImpl* impl, MATLABString rhs) {
                setElement<MATLABString>(impl, std::move(rhs), 0);
            }

            inline ReferenceImpl* getRef(ReferenceImpl* impl, const char* field, size_t len, bool unshare) {
                ReferenceImpl* retVal = nullptr;
                size_t numIdx = 0;
                typedef int(*StructReferenceGetIndexFcnPtr)(ReferenceImpl*, const char*, size_t, size_t*);
                static const StructReferenceGetIndexFcnPtr fcn = resolveFunction<StructReferenceGetIndexFcnPtr>
                    (FunctionType::STRUCT_REFERENCE_GET_INDEX);
                throwIfError(fcn(impl, field, len, &numIdx));
                typedef int(*ReferenceGetReferenceValueFcnPtr)(ReferenceImpl*, bool, ReferenceImpl**);
                static const ReferenceGetReferenceValueFcnPtr fcn2 = resolveFunction<ReferenceGetReferenceValueFcnPtr>
                    (FunctionType::REFERENCE_GET_REFERENCE_VALUE);
                fcn2(impl, unshare, &retVal);
                typedef int(*ReferenceAddIndexFcnPtr)(ReferenceImpl*, size_t);
                static const ReferenceAddIndexFcnPtr fcn3 = resolveFunction<ReferenceAddIndexFcnPtr>
                    (FunctionType::REFERENCE_ADD_INDEX);
                fcn3(retVal, numIdx);
                return retVal;
            }

            template <typename T, bool is_const_ref>
            typename std::enable_if<std::is_arithmetic<T>::value, T>::type castTo(std::shared_ptr<ReferenceImpl> impl) {
                void* value = nullptr;
                typedef int(*TypedReferenceGetPodValueFcnPtr)(ReferenceImpl*, void**);
                static const TypedReferenceGetPodValueFcnPtr fcn = resolveFunction<TypedReferenceGetPodValueFcnPtr>
                    (FunctionType::TYPED_REFERENCE_GET_POD_VALUE);
                throwIfError(fcn(impl.get(), &value));
                typedef int(*ReferenceGetContainerTypeFcnPtr)(ReferenceImpl*, int*);
                static const ReferenceGetContainerTypeFcnPtr fcn2 = resolveFunction<ReferenceGetContainerTypeFcnPtr>
                    (FunctionType::REFERENCE_GET_CONTAINER_TYPE);
                int type;
                throwIfError(fcn2(impl.get(), &type));
                return castToType<T>(value, static_cast<ArrayType>(type));
            }

            template <typename T, bool is_const_ref>
            typename std::enable_if<std::is_base_of<Array, T>::value, T>::type castTo(std::shared_ptr<ReferenceImpl> impl) {
                typedef int(*TypedArrayReferenceValidateTypeFcnPtr)(ReferenceImpl*, int, bool);
                static const TypedArrayReferenceValidateTypeFcnPtr fcn2 = resolveFunction<TypedArrayReferenceValidateTypeFcnPtr>
                    (FunctionType::TYPED_ARRAY_REFERENCE_VALIDATE_TYPE);
                throwIfError(fcn2(impl.get(), static_cast<int>(T::type), false));
                impl::ArrayImpl* arr_impl = nullptr;
                typedef void(*ArrayReferenceSharedCopyFcnPtr)(ReferenceImpl*,
                                                              matlab::data::impl::ArrayImpl**);
                static const ArrayReferenceSharedCopyFcnPtr fcn = resolveFunction<ArrayReferenceSharedCopyFcnPtr>
                    (FunctionType::ARRAY_REFERENCE_SHARED_COPY);
                fcn(impl.get(), &arr_impl);
                return Access::createObj<T>(arr_impl);
            }


            template <typename T, bool is_const_ref>
            typename std::enable_if<is_complex<T>::value, T>::type castTo(std::shared_ptr<ReferenceImpl> impl) {
                void* real = nullptr;
                void* imag = nullptr;
                typedef int(*TypedReferenceGetComplexValueFcnPtr)(ReferenceImpl*, void**, void**);
                static const TypedReferenceGetComplexValueFcnPtr fcn = resolveFunction<TypedReferenceGetComplexValueFcnPtr>
                    (FunctionType::TYPED_REFERENCE_GET_COMPLEX_VALUE);
                throwIfError(fcn(impl.get(),
                                         &real,
                                         &imag));

                typename T::value_type r = *(static_cast<typename T::value_type*>(real));
                typename T::value_type i = *(static_cast<typename T::value_type*>(imag));

                return T(r,i);
            }

            template <typename T, bool is_const_ref>
            typename std::enable_if<std::is_same<T, String>::value, String>::type castTo(std::shared_ptr<ReferenceImpl> impl) {
                typedef int(*ReferenceGetContainerTypeFcnPtr)(ReferenceImpl*, int*);
                static const ReferenceGetContainerTypeFcnPtr fcn2 = resolveFunction<ReferenceGetContainerTypeFcnPtr>
                    (FunctionType::REFERENCE_GET_CONTAINER_TYPE);
                int type;
                throwIfError(fcn2(impl.get(), &type));
                if (ArrayType(type) == ArrayType::MATLAB_STRING) {                
                    const char16_t* str = nullptr;
                    size_t strlen = 0;
                    typedef int(*StringGetValueFcnPtr)(ReferenceImpl* impl,
                                                       const char16_t**,
                                                       size_t*);
                    static const StringGetValueFcnPtr fcn = resolveFunction<StringGetValueFcnPtr>
                        (FunctionType::STRING_GET_VALUE);
                    throwIfError(fcn(impl.get(),
                                             &str,
                                             &strlen));
                    return String(str, strlen);
                }
                throw TypeMismatchException(std::string("Can't convert this element to a matlab::data::String"));
            }

            template <typename T, bool is_const_ref>
            typename std::enable_if<std::is_base_of<ReferenceHolder, T>::value, T>::type castTo(std::shared_ptr<ReferenceImpl> impl) {
                static_assert(!is_const_ref, "Can only get a reference to a non const ref");
                validateUntypedReference<typename T::ref_type>(impl.get());
                return Access::createObj<T>(impl);
            }

            template <typename T, bool is_const_ref>
            typename std::enable_if<std::is_same<T, std::string>::value, std::string>::type castTo(std::shared_ptr<ReferenceImpl> impl) {
                typedef int(*ReferenceGetContainerTypeFcnPtr)(ReferenceImpl*, int*);
                static const ReferenceGetContainerTypeFcnPtr fcn2 = resolveFunction<ReferenceGetContainerTypeFcnPtr>
                    (FunctionType::REFERENCE_GET_CONTAINER_TYPE);
                int arrType;
                throwIfError(fcn2(impl.get(), &arrType));
                if (ArrayType(arrType) == ArrayType::ENUM) {                   
                    const char* str = nullptr;
                    size_t strlen = 0;
                    typedef int(*EnumArrayGetValueFcnPtr)(ReferenceImpl* impl,
                                                          const char**,
                                                          size_t*);
                    static const EnumArrayGetValueFcnPtr fcn = resolveFunction<EnumArrayGetValueFcnPtr>
                        (FunctionType::ENUM_ARRAY_GET_VALUE);
                    throwIfError(fcn(impl.get(),
                                             &str,
                                             &strlen));
                    return std::string(str, strlen);
                }
                else if (ArrayType(arrType) == ArrayType::MATLAB_STRING) {
                    const char16_t* str = nullptr;
                    size_t strlen = 0;
                    typedef int(*StringGetValueFcnPtr)(ReferenceImpl* impl,
                                                       const char16_t**,
                                                       size_t*);
                    static const StringGetValueFcnPtr fcn = resolveFunction<StringGetValueFcnPtr>
                        (FunctionType::STRING_GET_VALUE);
                    throwIfError(fcn(impl.get(),
                                             &str,
                                             &strlen));
                    String temp(str, strlen);
                    if (!isAscii7(temp)) {
                        throw NonAsciiCharInInputDataException(std::string("Input data can only contain ASCII characters"));
                    }
                    return std::string(temp.begin(), temp.end());
                }
                throw TypeMismatchException(std::string("Can't convert this element to a std::string"));
            }
        }
    }
}

#endif
