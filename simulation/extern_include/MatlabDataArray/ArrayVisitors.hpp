/* Copyright 2017 The MathWorks, Inc. */

#ifndef MATLAB_DATA_VISITORS_HPP
#define MATLAB_DATA_VISITORS_HPP

#include <complex>

#include <type_traits>
 
#include "ArrayType.hpp"
#include "TypedArray.hpp"
#include "Struct.hpp"
#include "Object.hpp"
#include "ObjectArray.hpp"
#include "StructArray.hpp"
#include "EnumArray.hpp"
#include "CharArray.hpp"
#include "SparseArray.hpp"
#include "Exception.hpp"
#include "TypedArrayRef.hpp"
#include "SparseArrayRef.hpp"

namespace matlab {
    namespace data {

        template<typename V>
        auto apply_visitor(Array a, V visitor) -> decltype (visitor(CharArray(std::move(a))))
        {
            switch (a.getType()) {
              case ArrayType::CHAR: return(visitor(CharArray(std::move(a))));
              case ArrayType::MATLAB_STRING: return(visitor(TypedArray<MATLABString>(std::move(a))));
              case ArrayType::LOGICAL:   return(visitor(TypedArray<bool>(std::move(a))));
              case ArrayType::DOUBLE: return(visitor(TypedArray<double>(std::move(a))));
              case ArrayType::SINGLE: return(visitor(TypedArray<float>(std::move(a))));
              case ArrayType::INT8:   return(visitor(TypedArray<int8_t>(std::move(a))));
              case ArrayType::UINT8:  return(visitor(TypedArray<uint8_t>(std::move(a))));
              case ArrayType::INT16:  return(visitor(TypedArray<int16_t>(std::move(a))));
              case ArrayType::UINT16: return(visitor(TypedArray<uint16_t>(std::move(a))));
              case ArrayType::INT32:  return(visitor(TypedArray<int32_t>(std::move(a))));
              case ArrayType::UINT32: return(visitor(TypedArray<uint32_t>(std::move(a))));
              case ArrayType::INT64:  return(visitor(TypedArray<int64_t>(std::move(a))));
              case ArrayType::UINT64: return(visitor(TypedArray<uint64_t>(std::move(a))));
              case ArrayType::COMPLEX_DOUBLE: return(visitor(TypedArray<std::complex<double>>(std::move(a))));
              case ArrayType::COMPLEX_SINGLE: return(visitor(TypedArray<std::complex<float>>(std::move(a))));
              case ArrayType::COMPLEX_INT8:   return(visitor(TypedArray<std::complex<int8_t>>(std::move(a))));
              case ArrayType::COMPLEX_UINT8:  return(visitor(TypedArray<std::complex<uint8_t>>(std::move(a))));
              case ArrayType::COMPLEX_INT16:  return(visitor(TypedArray<std::complex<int16_t>>(std::move(a))));
              case ArrayType::COMPLEX_UINT16: return(visitor(TypedArray<std::complex<uint16_t>>(std::move(a))));
              case ArrayType::COMPLEX_INT32:  return(visitor(TypedArray<std::complex<int32_t>>(std::move(a))));
              case ArrayType::COMPLEX_UINT32: return(visitor(TypedArray<std::complex<uint32_t>>(std::move(a))));
              case ArrayType::COMPLEX_INT64:  return(visitor(TypedArray<std::complex<int64_t>>(std::move(a))));
              case ArrayType::COMPLEX_UINT64: return(visitor(TypedArray<std::complex<uint64_t>>(std::move(a))));
              case ArrayType::CELL: return (visitor(TypedArray<Array>(std::move(a))));
              case ArrayType::STRUCT: return (visitor(StructArray(std::move(a))));
              case ArrayType::VALUE_OBJECT: return (visitor(ObjectArray(std::move(a))));
              case ArrayType::HANDLE_OBJECT_REF: return(visitor(ObjectArray(std::move(a))));
              case ArrayType::ENUM: return (visitor(EnumArray(std::move(a))));
              case ArrayType::SPARSE_LOGICAL:   return(visitor(SparseArray<bool>(std::move(a))));
              case ArrayType::SPARSE_DOUBLE: return(visitor(SparseArray<double>(std::move(a))));
              case ArrayType::SPARSE_COMPLEX_DOUBLE: return(visitor(SparseArray<std::complex<double>>(std::move(a))));

              default:
                throw InvalidArrayTypeException("Array has no type");
                break;

            } /* switch */
        } /* apply_visitor */
        
        template<typename V>
        auto apply_visitor_ref(const ArrayRef& a, V visitor) -> decltype (visitor(static_cast<CharArrayRef>(a)))
        {
            switch (a.getType()) {
              case ArrayType::CHAR: return(visitor(static_cast<CharArrayRef>(a)));
              case ArrayType::MATLAB_STRING: return(visitor(static_cast<TypedArrayRef<MATLABString>>(a)));            
              case ArrayType::LOGICAL:   return(visitor(static_cast<TypedArrayRef<bool>>(a)));
              case ArrayType::DOUBLE: return(visitor(static_cast<TypedArrayRef<double>>(a)));
              case ArrayType::SINGLE: return(visitor(static_cast<TypedArrayRef<float>>(a)));
              case ArrayType::INT8:   return(visitor(static_cast<TypedArrayRef<int8_t>>(a)));
              case ArrayType::UINT8:  return(visitor(static_cast<TypedArrayRef<uint8_t>>(a)));
              case ArrayType::INT16:  return(visitor(static_cast<TypedArrayRef<int16_t>>(a)));
              case ArrayType::UINT16: return(visitor(static_cast<TypedArrayRef<uint16_t>>(a)));
              case ArrayType::INT32:  return(visitor(static_cast<TypedArrayRef<int32_t>>(a)));
              case ArrayType::UINT32: return(visitor(static_cast<TypedArrayRef<uint32_t>>(a)));
              case ArrayType::INT64:  return(visitor(static_cast<TypedArrayRef<int64_t>>(a)));
              case ArrayType::UINT64: return(visitor(static_cast<TypedArrayRef<uint64_t>>(a)));
              case ArrayType::COMPLEX_DOUBLE: return(visitor(static_cast<TypedArrayRef<std::complex<double>>>(a)));
              case ArrayType::COMPLEX_SINGLE: return(visitor(static_cast<TypedArrayRef<std::complex<float>>>(a)));
              case ArrayType::COMPLEX_INT8:   return(visitor(static_cast<TypedArrayRef<std::complex<int8_t>>>(a)));
              case ArrayType::COMPLEX_UINT8:  return(visitor(static_cast<TypedArrayRef<std::complex<uint8_t>>>(a)));
              case ArrayType::COMPLEX_INT16:  return(visitor(static_cast<TypedArrayRef<std::complex<int16_t>>>(a)));
              case ArrayType::COMPLEX_UINT16: return(visitor(static_cast<TypedArrayRef<std::complex<uint16_t>>>(a)));
              case ArrayType::COMPLEX_INT32:  return(visitor(static_cast<TypedArrayRef<std::complex<int32_t>>>(a)));
              case ArrayType::COMPLEX_UINT32: return(visitor(static_cast<TypedArrayRef<std::complex<uint32_t>>>(a)));
              case ArrayType::COMPLEX_INT64:  return(visitor(static_cast<TypedArrayRef<std::complex<int64_t>>>(a)));
              case ArrayType::COMPLEX_UINT64: return(visitor(static_cast<TypedArrayRef<std::complex<uint64_t>>>(a)));
              case ArrayType::CELL: return (visitor(static_cast<TypedArrayRef<Array>>(a)));
              case ArrayType::STRUCT: return (visitor(static_cast<StructArrayRef>(a)));
              case ArrayType::VALUE_OBJECT: return (visitor(static_cast<TypedArrayRef<Object>>(a)));
              case ArrayType::HANDLE_OBJECT_REF: return(visitor(static_cast<TypedArrayRef<Object>>(a)));
              case ArrayType::ENUM: return (visitor(static_cast<EnumArrayRef>(a)));
              case ArrayType::SPARSE_LOGICAL:   return(visitor(static_cast<SparseArrayRef<bool>>(a)));
              case ArrayType::SPARSE_DOUBLE: return(visitor(static_cast<SparseArrayRef<double>>(a)));
              case ArrayType::SPARSE_COMPLEX_DOUBLE: return(visitor(static_cast<SparseArrayRef<std::complex<double>>>(a)));
                
              default:
                throw InvalidArrayTypeException("Array has no type");
                break;

            } /* switch */
        } /* apply_visitor */
    }
}

#endif
