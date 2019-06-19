/* Copyright 2016-2017 The MathWorks, Inc. */

#ifndef MATLAB_DATA_GET_ARRAY_TYPE_HPP_
#define MATLAB_DATA_GET_ARRAY_TYPE_HPP_

#include "ArrayType.hpp"
#include "String.hpp"
#include "matlab_data_array_defs.hpp"

#include <complex>

namespace matlab {
    namespace data {

        class Array;
        class Struct;
        class Enumeration;
        class Object;
        template<typename T> class SparseArray;
        
        struct GetCellType { static const ArrayType type = ArrayType::CELL; };
        struct GetStringType { static const ArrayType type = ArrayType::UNKNOWN; };
        
        template<typename T> struct GetArrayType;
        template<> struct GetArrayType<bool> { static const ArrayType type = ArrayType::LOGICAL; };
        template<> struct GetArrayType<CHAR16_T> { static const ArrayType type = ArrayType::CHAR; };

        template<> struct GetArrayType<MATLABString> { static const ArrayType type = ArrayType::MATLAB_STRING; };
        template<> struct GetArrayType<double> { static const ArrayType type = ArrayType::DOUBLE; };
        template<> struct GetArrayType<float> { static const ArrayType type = ArrayType::SINGLE; };

        template<> struct GetArrayType<int8_t> { static const ArrayType type = ArrayType::INT8; };
        template<> struct GetArrayType<int16_t> { static const ArrayType type = ArrayType::INT16; };
        template<> struct GetArrayType<int32_t> { static const ArrayType type = ArrayType::INT32; };
        template<> struct GetArrayType<int64_t> { static const ArrayType type = ArrayType::INT64; };

        template<> struct GetArrayType<uint8_t> { static const ArrayType type = ArrayType::UINT8; };
        template<> struct GetArrayType<uint16_t> { static const ArrayType type = ArrayType::UINT16; };
        template<> struct GetArrayType<uint32_t> { static const ArrayType type = ArrayType::UINT32; };
        template<> struct GetArrayType<uint64_t> { static const ArrayType type = ArrayType::UINT64; };
#if !defined(__linux__) && !defined(_WIN32)
        template<> struct GetArrayType<unsigned long> { static const ArrayType type = ArrayType::UINT64; };
#endif
        template<> struct GetArrayType<std::complex<int8_t>> { static const ArrayType type = ArrayType::COMPLEX_INT8; };
        template<> struct GetArrayType<std::complex<int16_t>> { static const ArrayType type = ArrayType::COMPLEX_INT16; };
        template<> struct GetArrayType<std::complex<int32_t>> { static const ArrayType type = ArrayType::COMPLEX_INT32; };
        template<> struct GetArrayType<std::complex<int64_t>> { static const ArrayType type = ArrayType::COMPLEX_INT64; };

        template<> struct GetArrayType<std::complex<uint8_t>> { static const ArrayType type = ArrayType::COMPLEX_UINT8; };
        template<> struct GetArrayType<std::complex<uint16_t>> { static const ArrayType type = ArrayType::COMPLEX_UINT16; };
        template<> struct GetArrayType<std::complex<uint32_t>> { static const ArrayType type = ArrayType::COMPLEX_UINT32; };
        template<> struct GetArrayType<std::complex<uint64_t>> { static const ArrayType type = ArrayType::COMPLEX_UINT64; };
        template<> struct GetArrayType<std::complex<double>> { static const ArrayType type = ArrayType::COMPLEX_DOUBLE; };
        template<> struct GetArrayType<std::complex<float>> { static const ArrayType type = ArrayType::COMPLEX_SINGLE; };

        template<> struct GetArrayType<Array> { static const ArrayType type = ArrayType::CELL; };
        template<> struct GetArrayType<Struct> { static const ArrayType type = ArrayType::STRUCT; };
        template<> struct GetArrayType<Enumeration> { static const ArrayType type = ArrayType::ENUM; };
        template<> struct GetArrayType<Object> { static const ArrayType type = ArrayType::OBJECT; };
        template<> struct GetArrayType<SparseArray<bool>> { static const ArrayType type = ArrayType::SPARSE_LOGICAL;  };
        template<> struct GetArrayType<SparseArray<double>> { static const ArrayType type = ArrayType::SPARSE_DOUBLE; };
        template<> struct GetArrayType<SparseArray<std::complex<double>>> { static const ArrayType type = ArrayType::SPARSE_COMPLEX_DOUBLE; };

        template<typename T> struct GetSparseArrayType;
        template<> struct GetSparseArrayType<bool> { static const ArrayType type = ArrayType::SPARSE_LOGICAL;  };
        template<> struct GetSparseArrayType<double> { static const ArrayType type = ArrayType::SPARSE_DOUBLE; };
        template<> struct GetSparseArrayType<std::complex<double>> { static const ArrayType type = ArrayType::SPARSE_COMPLEX_DOUBLE; };


    }
}

#endif
