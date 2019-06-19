/* Copyright 2014-2017 The MathWorks, Inc. */

#ifndef MATLAB_DATA_STRUCT_ARRAY_HPP
#define MATLAB_DATA_STRUCT_ARRAY_HPP

#include "TypedArray.hpp"
#include "Range.hpp"
#include "MatlabFieldIdentifier.hpp"
#include "StructRef.hpp"
#include "ForwardIterator.hpp"
#include "Struct.hpp"

#include "detail/publish_util.hpp"
#include "detail/FunctionType.hpp"

#include "detail/HelperFunctions.hpp"

#include <memory>

namespace matlab {
    namespace data {
        namespace impl {
            class ArrayImpl;
        }
        namespace detail {
            class Access;
        }

        /**
         * Struct Array 
         */
        class StructArray : public TypedArray<Struct> {
          public:

            static const ArrayType type = ArrayType::STRUCT;

            /**
             * StructArray destructor
             *
             * @throw none
             */
            ~StructArray() MW_NOEXCEPT {}

            /**
             * StructArray move constructor
             *
             * @rhs - StructArray to be moved
             * @return - newly constructed StructArray
             * @throw - none
             */
            StructArray(StructArray&& rhs) MW_NOEXCEPT :
                TypedArray<Struct>::TypedArray(std::move(rhs)) {}

            /**
             * StructArray move assignment
             *
             * @rhs - StructArray to be moved
             * @return - updated StructArray
             * @throw - none
             */
            StructArray& operator=(StructArray&& rhs) MW_NOEXCEPT {
                TypedArray<Struct>::operator=(std::move(rhs));
                return *this;
            }

            /**
             * StructArray assignment operator - creates a shared copy of the input
             *
             * @rhs - StructArray to be copied
             * @return - updated StructArray
             * @throw - none
             */
            StructArray& operator=(StructArray const& rhs) MW_NOEXCEPT {
                TypedArray<Struct>::operator=(rhs);
                return *this;
            }                

            /**
             * StructArray move constructor
             *
             * @rhs - StructArray to be moved
             * @return - newly constructed StructArray
             * @throw - none
             */
            StructArray(const StructArray &rhs) MW_NOEXCEPT :
                TypedArray<Struct>::TypedArray(rhs) {}

            /**
             * Construct a StructArray from an Array r-value
             *
             * @param rhs - Array to be moved into the StructArray
             * @return - newly constructed StructArray
             * @throw InvalidArrayTypeException if type of rhs doesn't match
             */
            StructArray(Array&& rhs) :
                TypedArray<Struct>(std::move(rhs)) {}

            /**
             * Construct a StructArray from an Array
             *
             * @param rhs - Array to be moved into the StructArray
             * @return - shared copy of the input rhs
             * @throw InvalidArrayTypeException if type of rhs doesn't match
             */
            StructArray(const Array& rhs) :
                TypedArray<Struct>(rhs) {}

            /**
             * assignment operator - creates a shared data copy fron an Array
             *
             * @param rhs - rvalue to be copied
             * @return StructArray& the updated instance
             * @throw InvalidArrayTypeException if type of rhs type is not STRUCT
             */
            StructArray& operator=(const Array& rhs) {
                TypedArray<Struct>::operator=(rhs);
                return *this;
            }

            /**
             * move assignment operator from an Array
             *
             * @param rhs - rvalue to be moved
             * @return StructArray& the updated instance
             * @throw InvalidArrayTypeException if type of rhs type is not a STRUCT
             */
            StructArray& operator=(Array&& rhs)  {
                TypedArray<Struct>::operator=(std::move(rhs));
                return *this;
            }

            /**
             * Returns a Range<ForwardIterator, MATLABFieldIdentifier const> which will iterate over all
             * of the fieldNames in a struct array.
             *
             * @return Range<ForwardIterator, MATLABFieldIdentifier const>
             * @throw none
             */
            Range<ForwardIterator, MATLABFieldIdentifier const> getFieldNames() const MW_NOEXCEPT {
                typedef detail::ForwardIteratorImpl*(*StructArrayBeginIdFcnPtr)(impl::ArrayImpl*);
                static const StructArrayBeginIdFcnPtr b_fcn = detail::resolveFunction<StructArrayBeginIdFcnPtr>
                    (detail::FunctionType::STRUCT_ARRAY_BEGIN_ID);
                typedef detail::ForwardIteratorImpl*(*StructArrayEndIdFcnPtr)(impl::ArrayImpl*);
                static const StructArrayEndIdFcnPtr e_fcn = detail::resolveFunction<StructArrayEndIdFcnPtr>
                    (detail::FunctionType::STRUCT_ARRAY_END_ID);
                return Range<ForwardIterator, MATLABFieldIdentifier const>(detail::Access::createObj<ForwardIterator<MATLABFieldIdentifier const>>(b_fcn(detail::Access::getImpl<impl::ArrayImpl>(*this))),
                                                                           detail::Access::createObj<ForwardIterator<MATLABFieldIdentifier const>>(e_fcn(detail::Access::getImpl<impl::ArrayImpl>(*this))));
            }

            /**
             * get the number of fields in this struct
             *
             * @return size_t the number of fields
             * @throw none
             */
            size_t getNumberOfFields() const MW_NOEXCEPT {
                typedef size_t(*StructArrayGetNumFieldsFcnPtr)(impl::ArrayImpl*);
                static const StructArrayGetNumFieldsFcnPtr fcn = detail::resolveFunction<StructArrayGetNumFieldsFcnPtr>
                    (detail::FunctionType::STRUCT_ARRAY_GET_NUM_FIELDS);
                return fcn(detail::Access::getImpl<impl::ArrayImpl>(*this));
            }

          private:
            friend class detail::Access;
            
            StructArray() = delete;
            
            StructArray(impl::ArrayImpl* impl) MW_NOEXCEPT :
                TypedArray<Struct>(impl) {}
        };
    }
}

#endif
