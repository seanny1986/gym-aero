/* Copyright 2015-2017 The MathWorks, Inc. */

#ifndef MATLAB_DATA_ENUM_ARRAY_HPP
#define MATLAB_DATA_ENUM_ARRAY_HPP

#include "Enumeration.hpp"
#include "TypedArray.hpp"
#include "TypedArrayRef.hpp"

#include "detail/publish_util.hpp"
#include "detail/FunctionType.hpp"
#include "detail/HelperFunctions.hpp"

#include <memory>
#include <string>

namespace matlab {
    namespace data {
        namespace detail {
            class Access;
        }

        /**
         * Enum Array is a subclass of TypedArray for Enumeration objects.
         */
        class EnumArray : public TypedArray<Enumeration> {
          public:

            static const ArrayType type = ArrayType::ENUM;

            /**
             * move constructor
             *
             * @param rhs - rvalue to be moved
             * @return EnumArray the new instance
             * @throw none
             */
            EnumArray(EnumArray&& rhs) MW_NOEXCEPT :
                TypedArray<Enumeration>(std::move(rhs)) {}

            /**
             * move assignment operator
             *
             * @param rhs - rvalue to be moved
             * @return EnumArray& the updated instance
             * @throw none
             */
            EnumArray& operator=(EnumArray&& rhs) MW_NOEXCEPT {
                TypedArray<Enumeration>::operator=(std::move(rhs));
                return *this;
            }

            /**
             * copy constructor - creates a shared data copy
             *
             * @param rhs - EnumArray to be copied
             * @return EnumArray the new instance
             * @throw none
             */
            EnumArray(const EnumArray& rhs) MW_NOEXCEPT :
                TypedArray<Enumeration>(rhs) {}

            /**
             * assignment operator - creates a shared data copy
             *
             * @param rhs - rvalue to be copied
             * @return EnumArray& the updated instance
             * @throw none
             */
            EnumArray& operator=(const EnumArray& rhs) MW_NOEXCEPT {
                TypedArray<Enumeration>::operator=(rhs);
                return *this;
            }

            /**
             * copy constructor - creates a shared data copy from an Array
             *
             * @param rhs - EnumArray to be copied
             * @return EnumArray the new instance
             * @throw InvalidArrayTypeException if type of rhs type is not ENUM
             */
            EnumArray(const Array& rhs) :
                TypedArray<Enumeration>(rhs) {}

            /**
             * assignment operator - creates a shared data copy fron an Array
             *
             * @param rhs - rvalue to be copied
             * @return EnumArray& the updated instance
             * @throw InvalidArrayTypeException if type of rhs type is not ENUM
             */
            EnumArray& operator=(const Array& rhs) {
                TypedArray<Enumeration>::operator=(rhs);
                return *this;
            }

            /**
             * Construct a EnumArray from an Array
             *
             * @param rhs - rvalue to be moved
             * @return EnumArray the new instance
             * @throw InvalidArrayTypeException if type of rhs type is not ENUM
             */
            EnumArray(Array&& rhs) :
                TypedArray<Enumeration>(std::move(rhs)) {}

            /**
             * move assignment operator from an Array
             *
             * @param rhs - rvalue to be moved
             * @return EnumArray& the updated instance
             * @throw InvalidArrayTypeException if type of rhs type is not ENUM
             */
            EnumArray& operator=(Array&& rhs)  {
                TypedArray<Enumeration>::operator=(std::move(rhs));
                return *this;
            }

            /**
             * Return class name for this Enum Array
             *
             * @return class name
             * @throw none
             */
            std::string getClassName() const MW_NOEXCEPT {
                const char* str = nullptr;
                size_t strlen = 0;
                typedef void(*EnumArrayGetClassFcnPtr)(impl::ArrayImpl*, const char**, size_t*);
                static const EnumArrayGetClassFcnPtr fcn = detail::resolveFunction<EnumArrayGetClassFcnPtr>
                    (detail::FunctionType::ENUM_ARRAY_GET_CLASS);
                fcn(detail::Access::getImpl<impl::ArrayImpl>(*this),
                    &str,
                    &strlen);
                return std::string(str, strlen);
            }
            
          private:
            friend class detail::Access;

            EnumArray(impl::ArrayImpl* impl) MW_NOEXCEPT :
                TypedArray<Enumeration>(impl) {}

            EnumArray() = delete;
        };
        
        using EnumArrayRef = TypedArrayRef<Enumeration>;
    }
}

#endif
