/* Copyright 2015-2017 The MathWorks, Inc. */

#ifndef MATLAB_EXTDATA_ARRAY_ELEMENT_REF_HPP
#define MATLAB_EXTDATA_ARRAY_ELEMENT_REF_HPP

#include "matlab_data_array_defs.hpp"
#include "ArrayType.hpp"
#include "Exception.hpp"
#include "String.hpp"
#include "GetArrayType.hpp"
#include "ReferenceHolder.hpp"

#include "detail/publish_util.hpp"
#include "detail/FunctionType.hpp"
#include "detail/HelperFunctions.hpp"
#include "detail/ExceptionHelpers.hpp"
#include "detail/ReferenceHelpers.hpp"

#include <string>
#include <cstring>
#include <functional>
#include <memory>
#include <type_traits>

namespace matlab {
    namespace data {
        namespace detail {
            class Access;
            class ReferenceImpl;
        }

        class Array;
        template <bool> class IndexableArrayRef;
        
        /**
         * ArrayElementRef is created when using operator[] into an Array or a
         * Reference<Array>. It is untyped since Array and Reference<Array> do not have
         * type information. It collects up the indexes specified by the user and can be
         * cast to an element if the array holds primitive data or can be used to create a 
         * Reference<T> instance. The template argument indicates if this reference is
         * const. The non-const version of this class support assignment. Note that
         * indexing is zero-based.
         *
         * @code
         *    Array arr = factory.createArray<double>({2,2});
         *    arr[0][0] = 5.5;
         *    double val = arr[0][0];
         *
         *    Array cell_arr = factory.createArray<Array>({1,2});
         *    cell_arr[0] = factory.createScalar(10.5);
         *    cell_arr[1] = factory.createScalar(false);
         *
         *    ArrayRef              ref_to_element       = cell_arr[0];
         *    TypedArrayRef<double> typed_ref_to_element = cell_arr[0];
         *
         *    Array const            shared_copy_of_element       = cell_arr[1];
         *    TypedArray<bool> const typed_shared_copy_of_element = cell_arr[1];
         *
         * @endcode
         */
        template <bool is_const_ref>
        class ArrayElementRef {
          public:
            /**
             * operator[] adds another index to the indexing operation. It is called when 2
             * or more indices are used in an indexing operation.
             *
             * @param idx - the index to be added
             *
             * @return ArrayElementRef<is_const_ref> - a new ArrayElementRef<is_const_ref> with the additional index
             *
             * @throw TooManyIndicesProvidedException - if the number of indices is more than the number of dimensions in the Array
             * @throw StringIndexMustBeLastException - if a numeric index is provided after a string index
             */
            ArrayElementRef<is_const_ref> operator[](size_t idx) {
                typedef int(*ReferenceAddIndexFcnPtr)(detail::ReferenceImpl*, size_t idx);
                static const ReferenceAddIndexFcnPtr fcn = detail::resolveFunction<ReferenceAddIndexFcnPtr>(detail::FunctionType::REFERENCE_ADD_INDEX);
                detail::throwIfError(fcn(pImpl.get(), idx));
                return ArrayElementRef<is_const_ref>(pImpl);
            }
            
            /**
             * operator[] adds string index to the indexing operation. It is only valid if the
             * Array being indexed support string indexing (StructArrays)
             *
             * @code
             *  double val = arr[2][7]["f1"];
             * @endcode
             *
             * @param idx - the index to be added
             *
             * @return ArrayElementRef<is_const_ref> - a new ArrayElementRef<is_const_ref> with the string index
             *
             * @throw StringIndexNotValidException - if this Array does not support indexing with a string
             * @throw CanOnlyUseOneStringIndexException - if more than one string index is supplied
             */
            ArrayElementRef<is_const_ref> operator[](std::string idx) {
                typedef int(*ReferenceAddStringIndexFcnPtr)(detail::ReferenceImpl*, const char*, size_t idx);
                static const ReferenceAddStringIndexFcnPtr fcn = detail::resolveFunction<ReferenceAddStringIndexFcnPtr>(detail::FunctionType::REFERENCE_ADD_STRING_INDEX);
                detail::throwIfError(fcn(pImpl.get(), idx.c_str(), idx.size()));
                return ArrayElementRef<is_const_ref>(pImpl);
            }

            /**
             * Cast a value. Can be cast to the element type of the array, or to a reference to an element of the 
             * array if the element type is not a primitive type
             *
             * @code
             *    double val = dblArray_const[1][2];
             * @endcode
             *
             * @return T - the value
             *
             * @throw NotEnoughIndicesProvidedException - if not enough indices were provided
             * @throw InvalidArrayIndexException - if an array index is out of range
             * @throw TypeMismatchException - if the element of the Array cannot be cast to T
             */
            template <typename T>
            operator T() const {
                return detail::castTo<T, is_const_ref>(pImpl);
            }
                
            /**
             * Assign data into the Array.
             * throw an error if the Array does not contain the same type being assigned.
             *
             * @code
             *    arr[1][2] = 5.0;
             * @endcode
             *
             * @param rhs - primitive data to be assigned into the Array
             *
             * @return ArrayElementRef<is_const_ref>& - this
             *
             * @throw InvalidArrayIndexException - if one of the indices is out of range for this Array
             * @throw NotEnoughIndicesProvidedException - if not enough indices were provided
             * @throw TypeMismatchException - if the element of the Array does not contain T's
             */
            template <typename T>
            ArrayElementRef<is_const_ref>& operator= (T rhs) {
                static_assert(!is_const_ref, "Can't modify a const reference");
                typedef typename std::conditional<std::is_same<std::string, T>::value || std::is_same<String, T>::value, GetStringType, GetArrayType<T>>::type GetNonArrayType;
                typedef typename std::conditional<std::is_base_of<Array, T>::value, GetCellType, GetNonArrayType>::type GetType;
                detail::setElement(pImpl.get(), std::move(rhs), static_cast<int>(GetType::type));
                return *this;
            }
                
          private:
            std::shared_ptr<detail::ReferenceImpl> pImpl;

            ArrayElementRef(ArrayElementRef<is_const_ref>&& rhs) MW_NOEXCEPT :
                pImpl(std::move(rhs.pImpl)) {}

            ArrayElementRef(detail::ReferenceImpl* impl) MW_NOEXCEPT :
                pImpl(std::shared_ptr<detail::ReferenceImpl>
                      (impl, [](detail::ReferenceImpl* ptr) {
                          typedef void(*ReferenceDestroyFcnPtr)(detail::ReferenceImpl*);
                          static const ReferenceDestroyFcnPtr fcn2 = detail::resolveFunction<ReferenceDestroyFcnPtr>(detail::FunctionType::REFERENCE_DESTROY);
                          fcn2(ptr);
                      })) {}

            ArrayElementRef(std::shared_ptr<detail::ReferenceImpl> impl) MW_NOEXCEPT
                : pImpl(impl) {}

            friend class detail::Access;
            friend class Array;
            friend class IndexableArrayRef<true>;
            friend class IndexableArrayRef<false>;

            ArrayElementRef() = delete;
            ArrayElementRef(const ArrayElementRef<is_const_ref>& rhs) = delete;
            ArrayElementRef<is_const_ref>& operator=(const ArrayElementRef<is_const_ref>& rhs) = delete;
            ArrayElementRef<is_const_ref>& operator=(ArrayElementRef<is_const_ref>&& rhs) = delete;
        };
    }
}

#endif
