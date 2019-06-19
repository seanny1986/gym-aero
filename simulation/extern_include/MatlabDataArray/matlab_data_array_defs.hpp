/* Copyright 2014-2016 The MathWorks, Inc. */

#ifndef MATLAB_DATA_ARRAY_DEFS_H
#define MATLAB_DATA_ARRAY_DEFS_H

#include <complex>
#include "Optional.hpp"

#ifndef _HAS_CHAR16_T_LANGUAGE_SUPPORT
  #define _HAS_CHAR16_T_LANGUAGE_SUPPORT 0
#endif

#if (defined(__cplusplus) && (__cplusplus >= 201103L)) || _HAS_CHAR16_T_LANGUAGE_SUPPORT
  #define U16_STRING_LITERAL_PREFIX u
  typedef char16_t CHAR16_T;
#else
  #define U16_STRING_LITERAL_PREFIX L
  typedef wchar_t CHAR16_T;
  #if !defined(_CHAR16T)
    #define _CHAR16T
    typedef wchar_t char16_t;
  #endif
#endif


namespace matlab {
    namespace data {

        enum class DataOrder : int {
            COLUMN_MAJOR,
            ROW_MAJOR
        };

        
        template <typename T> struct is_complex : std::false_type {};
        template <typename T> struct is_complex<std::complex<T>> : std::true_type {};
        template <typename T> struct is_complex<std::complex<T> const> : std::true_type {};

        template <typename T> struct is_const_complex : std::false_type {};
        template <typename T> struct is_const_complex<std::complex<T> const> : std::true_type {};

        template <typename T> struct is_optional : std::false_type {};
        template <typename T> struct is_optional<optional<T>> : std::true_type {};
        template <typename T> struct is_optional<optional<T> const> : std::true_type {};

        template <typename T> struct is_const_optional : std::false_type {};
        template <typename T> struct is_const_optional<optional<T> const> : std::true_type {};
    }
}

#endif
