/* Copyright 2014-2017 The MathWorks, Inc. */

#ifndef CHAR_ARRAY_HPP_
#define CHAR_ARRAY_HPP_

#include "matlab_data_array_defs.hpp"
#include "TypedArray.hpp"
#include "TypedArrayRef.hpp"

#include "detail/publish_util.hpp"
#include "detail/FunctionType.hpp"

#include <stdint.h>
#include <memory>
#include <string>

namespace matlab {
    namespace data {
        namespace impl {
            class ArrayImpl;
        }

        namespace detail {
            class Access;
        }

        /**
         * CharArray class provides an API for accessing char16_t data.
         * It inherits most of its functionality from TypedArray<char16_t> class
         * and adds an API to get the contents of a Char array as a
         * string of char16_t.
         */
        class CharArray : public TypedArray<CHAR16_T>
        {
          public:

            static const ArrayType type = ArrayType::CHAR;

            /**
             * move assignment operator
             *
             * @param rhs - rvalue to be moved
             * @return CharArray& the updated instance
             * @throw none
             */
            CharArray& operator=(CharArray&& rhs) MW_NOEXCEPT
            {
                TypedArray<CHAR16_T>::operator=(std::move(rhs));
                return *this;
            }

            /**
             * move constructor
             *
             * @param rhs - rvalue to be moved
             * @return CharArray the new instance
             * @throw none
             */
            CharArray(CharArray&& rhs) MW_NOEXCEPT :
                TypedArray<CHAR16_T>(std::move(rhs)) {}

            /**
             * assignment operator - creates a shared data copy
             *
             * @param rhs - rvalue to be copied
             * @return CharArray& the updated instance
             * @throw none
             */
            CharArray& operator=(CharArray const& rhs) MW_NOEXCEPT 
            {
                TypedArray<CHAR16_T>::operator=(rhs);
                return *this;
            }

            /**
             * copy constructor - creates a shared data copy
             *
             * @param rhs - CharArray to be copied
             * @return CharArray the new instance
             * @throw none
             */
            CharArray(const CharArray &rhs) MW_NOEXCEPT :
                TypedArray<CHAR16_T>(rhs) {}

            /**
             * Construct a CharArray from an Array
             *
             * @param rhs - rvalue to be moved
             * @return CharArray the new instance
             * @throw InvalidArrayTypeException if type of rhs type is not CHAR
             */
            CharArray(Array&& rhs) :
                TypedArray<CHAR16_T>(std::move(rhs)) {}

            /**
             * move assignment operator from an Array
             *
             * @param rhs - rvalue to be moved
             * @return CharArray& the updated instance
             * @throw InvalidArrayTypeException if type of rhs type is not CHAR
             */
            CharArray& operator=(Array&& rhs)
            {
                TypedArray<CHAR16_T>::operator=(std::move(rhs));
                return *this;
            }

            /**
             * copy constructor - creates a shared data copy from an Array
             *
             * @param rhs - CharArray to be copied
             * @return CharArray the new instance
             * @throw InvalidArrayTypeException if type of rhs type is not CHAR
             */
            CharArray(const Array& rhs) :
                TypedArray<CHAR16_T>(rhs) {}

            /**
             * assignment operator - creates a shared data copy fron an Array
             *
             * @param rhs - rvalue to be copied
             * @return CharArray& the updated instance
             * @throw InvalidArrayTypeException if type of rhs type is not CHAR
             */
            CharArray& operator=(const Array& rhs)
            {
                TypedArray<CHAR16_T>::operator=(rhs);
                return *this;
            }


            /**
             * Return contents of a CHAR array as a utf16 string
             *
             * @return std::basic_string<CHAR16_T> string
             * @throws none
             */
            String toUTF16() const MW_NOEXCEPT 
            {
                const char16_t* str = nullptr;
                size_t strLen = 0;
                typedef void(*CharArrayGetStringFcnPtr)(impl::ArrayImpl*,
                                                        const char16_t**,
                                                        size_t*);
                static const CharArrayGetStringFcnPtr fcn = detail::resolveFunction<CharArrayGetStringFcnPtr>
                    (detail::FunctionType::CHAR_ARRAY_GET_STRING);
                fcn(detail::Access::getImpl<impl::ArrayImpl>(*this), &str, &strLen);
                return String(str, strLen);
            }

            /**
             * Return contents of a CHAR array as an ascii string
             *
             * @return std::string string
             * @throws NonAsciiCharInRequestedAsciiOutputException - if data contains non-ascii characters
             */
            std::string toAscii() const
            {
                const char16_t* strVal = nullptr;
                size_t strLen = 0;
                typedef int(*CharArrayGetAsciiFcnPtr)(impl::ArrayImpl*,
                                                      const char16_t**,
                                                      size_t*);
                static const CharArrayGetAsciiFcnPtr fcn = detail::resolveFunction<CharArrayGetAsciiFcnPtr>
                    (detail::FunctionType::CHAR_ARRAY_GET_ASCII);
                detail::throwIfError(fcn(detail::Access::getImpl<impl::ArrayImpl>(*this), &strVal, &strLen));
                return std::string(strVal, strVal+strLen);
            }

          private:
            friend class detail::Access;

            CharArray(impl::ArrayImpl* impl) MW_NOEXCEPT :
                TypedArray<CHAR16_T>(impl) {}

            CharArray() = delete;
        };

        using CharArrayRef = TypedArrayRef<CHAR16_T>;
    }
}

#endif
