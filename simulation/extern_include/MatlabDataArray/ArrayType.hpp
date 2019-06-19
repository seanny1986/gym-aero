/* Copyright 2016-2017 The MathWorks, Inc. */

#ifndef MATLAB_DATA_ARRAY_TYPE_HPP
#define MATLAB_DATA_ARRAY_TYPE_HPP


namespace matlab {
    namespace data {
        enum class ArrayType : int {
            LOGICAL,
            CHAR,
            MATLAB_STRING,           
            DOUBLE,
            SINGLE,
            INT8,
            UINT8,
            INT16,
            UINT16,
            INT32,
            UINT32,
            INT64,
            UINT64,
            COMPLEX_DOUBLE,
            COMPLEX_SINGLE,
            COMPLEX_INT8,
            COMPLEX_UINT8,
            COMPLEX_INT16,
            COMPLEX_UINT16,
            COMPLEX_INT32,
            COMPLEX_UINT32,
            COMPLEX_INT64,
            COMPLEX_UINT64,
            CELL,
            STRUCT,
            OBJECT,
            VALUE_OBJECT,
            HANDLE_OBJECT_REF,
            ENUM,
            SPARSE_LOGICAL,
            SPARSE_DOUBLE,
            SPARSE_COMPLEX_DOUBLE,
            UNKNOWN,
        };
    }
}

#endif
