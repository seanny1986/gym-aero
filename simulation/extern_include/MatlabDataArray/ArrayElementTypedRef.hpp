/* Copyright 2015-2017 The MathWorks, Inc. */

#ifndef MATLAB_EXTDATA_ARRAY_ELEMENT_TYPED_REF_HPP
#define MATLAB_EXTDATA_ARRAY_ELEMENT_TYPED_REF_HPP

#include "Enumeration.hpp"
#include "String.hpp"
#include "Exception.hpp"
#include "MDArray.hpp"
#include "ReferenceHolder.hpp"
#include "ArrayReferenceExt.hpp"
#include "MATLABStringReferenceExt.hpp"
#include "Struct.hpp"
#include "GetArrayType.hpp"
#include "Reference.hpp"

#include "detail/HelperFunctions.hpp"
#include "detail/ReferenceHelpers.hpp"

#include <memory>
#include <type_traits>

namespace matlab {
    namespace data {
        class Array;
        template <typename T> class TypedArray;
        template <typename T> class Reference;
        
        namespace detail {
            class Access;
        }

        template <typename T> struct GetElementReferenceType      { using type = ReferenceHolder; };
        template <> struct GetElementReferenceType<Array>         { using type = ArrayReferenceExt; };
        template <> struct GetElementReferenceType<MATLABString > { using type = MATLABStringReferenceExt; };

        /**
         * ArrayElementTypedRef is created when using operator[] into a TypedArray<T> or a
         * TypedArrayRef<T>. It collects up the indexes specified by the user and can be
         * cast to T without the added cost of type checking that exists when using
         * ArrayElementRef. Primitive types can be cast to T& which is not supported with
         * ArrayElementRef. The non-const version of this class supports assignment.
         *
         * @code
         *    TypedArray<double> arr = factory.createArray<double>({2,2});
         *    arr[0][0] = 5.5;
         *    double& val = arr[0][0];
         *
         *    CellArray cell_arr = factory.createArray<Array>({1,2});
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
        template <typename T, bool is_const_ref>
        class ArrayElementTypedRef : public GetElementReferenceType<T>::type {
          public:

            using Parent = typename GetElementReferenceType<T>::type;
            using Parent::pImpl;
            
            typedef typename std::conditional<std::is_arithmetic<T>::value && !is_const_ref, T&, T>::type reference;

            /**
             * operator[] adds another index to the indexing operation. It is called when 2
             * or more indices are used in an indexing operation.
             *
             * @code
             *  double val = arr[2][7][3];
             * @endcode
             *
             * @param idx - the index to be added
             *
             * @return ArrayElementTypedRef<T, is_const_ref> - a new ArrayElementTypedRef<T, is_const_ref> with the additional index
             *
             * @throw TooManyIndicesProvidedException - if the number of indices is more than the number of dimensions in the Array
             * @throw StringIndexMustBeLastException - if a numeric index is provided after a string index
             *
             */
            ArrayElementTypedRef<T, is_const_ref> operator[](size_t idx) {
                typedef int(*ReferenceAddIndexFcnPtr)(detail::ReferenceImpl*, size_t);
                static const ReferenceAddIndexFcnPtr fcn = detail::resolveFunction<ReferenceAddIndexFcnPtr>(detail::FunctionType::REFERENCE_ADD_INDEX);
                detail::throwIfError(fcn(pImpl.get(), idx));
                return ArrayElementTypedRef<T, is_const_ref>(pImpl);
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
             * @return ArrayElementTypedRef<Array, is_const_ref> - a new ArrayElementTypedRef<Array, is_const_ref> with the string index
             *
             * @throw StringIndexNotValidException - if this Array does not support indexing with a string
             * @throw CanOnlyUseOneStringIndexException - if more than one string index is supplied
             */
            ArrayElementTypedRef<Array, is_const_ref> operator[](std::string idx) {
                typedef int(*ReferenceAddStringIndexFcnPtr)(detail::ReferenceImpl* impl, const char*, size_t idx);
                static const ReferenceAddStringIndexFcnPtr fcn = detail::resolveFunction<ReferenceAddStringIndexFcnPtr>(detail::FunctionType::REFERENCE_ADD_STRING_INDEX);
                detail::throwIfError(fcn(pImpl.get(), idx.c_str(), idx.size()));
                return detail::Access::createObj<ArrayElementTypedRef<Array, is_const_ref>>(pImpl);
            }
            
            /**
             * Cast to reference.
             *
             * @code
             *    double& val = dblArray[1][2];
             * @endcode
             *
             * @return reference - the value
             *
             * @throw NotEnoughIndicesProvidedException - if not enough indices were provided
             * @throw InvalidArrayIndexException - if an array index is out of range
             */
            operator reference() const {
                return detail::getElement<T>(pImpl);
            }

            /**
             * Cast to std::string - valid for elements of a StringArray and an EnumArray
             *
             * @code
             *    std::string val = strArray[1][2];
             * @endcode
             *
             * @return std::string - the element of the array
             *
             * @throw NotEnoughIndicesProvidedException - if not enough indices were provided
             * @throw InvalidArrayIndexException - if an array index is out of range
             * @throw TypeMismatchException - if the element of the Array cannot be cast to a std::string
             * @throw std::runtime_error - if indexing into a StringArray and the element is missing
             */
            operator std::string() const {
                return detail::getString<T>(pImpl.get());
            }

            /**
             * Assign a value into an Array. The Array being indexed must be non-const.
             *
             * @code
             *    arr[1][2] = std::complex<double>(5.4, 3.1);
             * @endcode
             *
             * @param rhs - value to be assigned into the Array
             *
             * @return ArrayElementTypedRef<T, is_const_ref>& - this
             *
             * @throw NotEnoughIndicesProvidedException - if not enough indices were provided
             * @throw InvalidArrayIndexException - if an array index is out of range
             */
            ArrayElementTypedRef<T, is_const_ref>& operator= (T rhs) {
                static_assert(!is_const_ref, "Can't modify a reference to const data");
                detail::setElement(pImpl.get(), std::move(rhs), static_cast<int>(GetArrayType<T>::type));
                return *this;
            }

           /**
            * Assign a String value into an Array. The Array being indexed must be of type
            * MATLABString and be non-const.
            *
            * @code
            *    arr[1][2] = u"MathWorks";
            * @endcode
            *
            * @param rhs - String value to be assigned into the Array
            *
            * @return ArrayElementTypedRef<T, <is_const_ref>& - this 
            *
            * @throw NotEnoughIndicesProvidedException - if not enough indices were provided
            * @throw InvalidArrayIndexException - if an array index is out of range
            */
            ArrayElementTypedRef<T, is_const_ref>& operator= (String rhs) {
                static_assert(std::is_same<T, MATLABString>::value, "T should be MATLABString");
                static_assert(!is_const_ref, "Can't modify a reference to const data");
                detail::setString<MATLABString>(pImpl.get(), std::move(rhs));
                return *this;
            }


            /**
             * Assign a std::string into an Array. The Array being indexed must be non-const and must
             * support string assignments
             *
             * @code
             *    arr[1][2] = "MyString";
             * @endcode
             *
             * @param rhs - value to be assigned into the Array
             *
             * @return ArrayElementTypedRef<T, is_const_ref>& - this
             *
             * @throw NotEnoughIndicesProvidedException - if not enough indices were provided
             * @throw InvalidArrayIndexException - if an array index is out of range
             * @throw TypeMismatchException - if a string cannot be assigned to the Array
             */

            ArrayElementTypedRef<T, is_const_ref>& operator= (std::string rhs) {
                static_assert(!is_const_ref, "Can't modify a reference to const data");
                detail::setString<T>(pImpl.get(), std::move(rhs));
                return *this;
            }
            
            /**
             * cast to a Reference to an element. Used when the element is not a primitive. Reference<U> can be used
             * to modify the element being referred to without make a shared copy
             *
             * @code
             *    Reference<Array> ref = cellArr[0][0];
             * @endcode
             *
             * @return Reference<U>
             *
             * @throw NotEnoughIndicesProvidedException - if not enough indices were provided
             * @throw InvalidArrayIndexException - if an array index is out of range
             * @throw TypeMismatchException - if the element of the Array does not match <U>
             */
            template <typename U>
            operator Reference<U>() const {
                static_assert(!is_const_ref, "Can only get a reference to a non const ref");
                detail::validateReference<T, U>(pImpl.get(), true);
                return detail::Access::createObj<Reference<U>>(pImpl);
            }

            /**
             * If this is a Reference<Array>, allow it to be cast to a specific Array, i.e., TypedArray<double>, StringArray, CellArray, etc.
             *
             * @return U
             *
             * @throw NotEnoughIndicesProvidedException - if not enough indices were provided
             * @throw InvalidArrayIndexException - if an array index is out of range
             * @throw TypeMismatchException - if the element of the Array does not match <U>
             */
            template <typename U, typename Type = T, typename = typename std::enable_if<std::is_base_of<Array, Type>::value>::type>
            operator U() const {
                static_assert(std::is_base_of<Array, U>::value, "Can only be used to cast to an Array");
                return detail::castTo<U, is_const_ref>(pImpl);
            }

            /**
             * Assign a value into a TypedArray from another TypedArray of the same type
             *
             * @code
             *    auto arr1 = factory.createScalar(10.1);
             *    auto arr2 = factory.createScalar(10.2);
             *
             *    arr1[0] = arr[2]; // assigns the value 10.2 into arr1
             * @endcode
             *
             * @param rhs - reference to value to be assigned into the Array
             *
             * @return ArrayElementTypedRef<T, is_const_ref>& - this
             *
             * @throw NotEnoughIndicesProvidedException - if not enough indices were provided
             * @throw InvalidArrayIndexException - if an array index is out of range
             */
            template <bool is_rhs_const>
            ArrayElementTypedRef<T, is_const_ref>& operator=(ArrayElementTypedRef<T, is_rhs_const>&& rhs) {
                static_assert(!is_const_ref, "Can't modify a reference to const data");
                auto val = static_cast<T>(rhs);
                return operator=(val);
            }
                

          private:

            ArrayElementTypedRef(ArrayElementTypedRef<T, is_const_ref>&& rhs) MW_NOEXCEPT :
                Parent(std::move(rhs.pImpl)) {}

            ArrayElementTypedRef(detail::ReferenceImpl* impl) MW_NOEXCEPT :
                Parent(impl) {}

            ArrayElementTypedRef(std::shared_ptr<detail::ReferenceImpl> impl) MW_NOEXCEPT :
                Parent(impl) {}


            friend class detail::Access;
            friend class TypedArray<T>;
            friend class Reference<TypedArray<T>>;
            friend class ArrayElementTypedRef<Struct, true>;
            friend class ArrayElementTypedRef<Struct, false>;
            
            ArrayElementTypedRef() = delete;
            ArrayElementTypedRef(const ArrayElementTypedRef<T, is_const_ref>& rhs) = delete;
            ArrayElementTypedRef<T, is_const_ref>& operator=(const ArrayElementTypedRef<T, is_const_ref>& rhs) = delete;
        };

        template<typename T, bool is_const_ref>
        bool operator ==(ArrayElementTypedRef<T, is_const_ref> const& lhs, T const& rhs) MW_NOEXCEPT {
            using reference = typename ArrayElementTypedRef<T, is_const_ref>::reference;
            return static_cast<reference>(lhs) == rhs;
        }

        template<typename T, bool is_const_ref>
        bool operator ==(T const& lhs, ArrayElementTypedRef<T, is_const_ref> const& rhs) MW_NOEXCEPT {
            using reference = typename ArrayElementTypedRef<T, is_const_ref>::reference;
            return lhs == static_cast<reference>(rhs);
        }

        template<typename T, bool is_const_ref>
        bool operator ==(ArrayElementTypedRef<T, is_const_ref> const& lhs, ArrayElementTypedRef<T, is_const_ref> const& rhs) MW_NOEXCEPT {
            using reference = typename ArrayElementTypedRef<T, is_const_ref>::reference;
            return static_cast<reference>(lhs) == static_cast<reference>(rhs);
        }
        
        template<typename T, bool is_const_ref>
        bool operator ==(ArrayElementTypedRef<T, is_const_ref> const& lhs, Reference<T> const& rhs) MW_NOEXCEPT {
            using reference = typename ArrayElementTypedRef<T, is_const_ref>::reference;
            return static_cast<reference>(lhs) == static_cast<reference>(rhs);
        }

        template<typename T, bool is_const_ref>
        bool operator ==(Reference<T> const& lhs, ArrayElementTypedRef<T, is_const_ref> const& rhs) MW_NOEXCEPT {
            using reference = typename ArrayElementTypedRef<T, is_const_ref>::reference;
            return static_cast<reference>(lhs) == static_cast<reference>(rhs);
        }

        template<bool is_const_ref>
        bool operator ==(ArrayElementTypedRef<MATLABString, is_const_ref> const& lhs, String const& rhs) MW_NOEXCEPT {
            typedef bool(*ReferenceMATLABStringEqualString16FcnPtr)(detail::ReferenceImpl* impl, const char16_t*, size_t idx);
            static const ReferenceMATLABStringEqualString16FcnPtr fcn = detail::resolveFunction<ReferenceMATLABStringEqualString16FcnPtr>
                (detail::FunctionType::REFERENCE_MATLAB_STRING_EQUAL_STRING16);
            return fcn(detail::Access::getImpl<detail::ReferenceImpl>(lhs), rhs.c_str(), rhs.size());
        }
            
        template<bool is_const_ref>
        bool operator ==(String const& lhs, ArrayElementTypedRef<MATLABString, is_const_ref> const& rhs) MW_NOEXCEPT {
            typedef bool(*ReferenceMATLABStringEqualString16FcnPtr)(detail::ReferenceImpl* impl, const char16_t*, size_t idx);
            static const ReferenceMATLABStringEqualString16FcnPtr fcn = detail::resolveFunction<ReferenceMATLABStringEqualString16FcnPtr>
                (detail::FunctionType::REFERENCE_MATLAB_STRING_EQUAL_STRING16);
            return fcn(detail::Access::getImpl<detail::ReferenceImpl>(rhs), lhs.c_str(), lhs.size());
        }
        
        template<bool is_const_ref>
        bool operator ==(ArrayElementTypedRef<MATLABString, is_const_ref> const& lhs, MATLABString const& rhs) MW_NOEXCEPT {
            if (rhs.has_value()) {
                const String& str = *rhs;
                return (lhs == str);
            }
            return false;
        }
        
        template<bool is_const_ref>
        bool operator ==(MATLABString const& lhs, ArrayElementTypedRef<MATLABString, is_const_ref> const& rhs) MW_NOEXCEPT {
            if (lhs.has_value()) {
                const String& str = *lhs;
                return (rhs == str);
            }
            return false;
        }
        
        template<bool is_const_ref>
        bool operator ==(ArrayElementTypedRef<MATLABString, is_const_ref> const& lhs, std::string const& rhs) MW_NOEXCEPT {
            typedef bool(*ReferenceMATLABStringEqualStringFcnPtr)(detail::ReferenceImpl*,
                                                                 const char*,
                                                                 size_t idx);
            static const ReferenceMATLABStringEqualStringFcnPtr fcn = detail::resolveFunction<ReferenceMATLABStringEqualStringFcnPtr>
                (detail::FunctionType::REFERENCE_MATLAB_STRING_EQUAL_STRING);
            return fcn(detail::Access::getImpl<detail::ReferenceImpl>(lhs), rhs.c_str(), rhs.size());
        }
        
        template<bool is_const_ref>
        bool operator ==(std::string const& lhs, ArrayElementTypedRef<MATLABString, is_const_ref> const& rhs) MW_NOEXCEPT {
            typedef bool(*ReferenceMATLABStringEqualStringFcnPtr)(detail::ReferenceImpl*,
                                                                  const char*,
                                                                  size_t idx);
            static const ReferenceMATLABStringEqualStringFcnPtr fcn = detail::resolveFunction<ReferenceMATLABStringEqualStringFcnPtr>
                (detail::FunctionType::REFERENCE_MATLAB_STRING_EQUAL_STRING);
            return fcn(detail::Access::getImpl<detail::ReferenceImpl>(rhs), lhs.c_str(), lhs.size());
        }
        
        template<bool is_const_ref>
        bool operator ==(Reference<MATLABString> const& lhs, ArrayElementTypedRef<MATLABString, is_const_ref> const& rhs) MW_NOEXCEPT {
            typedef bool(*ReferenceMATLABStringEqualReferenceFcnPtr)(detail::ReferenceImpl*,
                                                                     detail::ReferenceImpl*);
            static const ReferenceMATLABStringEqualReferenceFcnPtr fcn = detail::resolveFunction<ReferenceMATLABStringEqualReferenceFcnPtr>
                (detail::FunctionType::REFERENCE_MATLAB_STRING_EQUAL_REFERENCE);
            return fcn(detail::Access::getImpl<detail::ReferenceImpl>(lhs),
                       detail::Access::getImpl<detail::ReferenceImpl>(rhs));
        }
        
        template<bool is_const_ref>
        bool operator ==(ArrayElementTypedRef<MATLABString, is_const_ref> const& lhs, Reference<MATLABString> const& rhs) MW_NOEXCEPT {
            typedef bool(*ReferenceMATLABStringEqualReferenceFcnPtr)(detail::ReferenceImpl*,
                                                                     detail::ReferenceImpl*);
            static const ReferenceMATLABStringEqualReferenceFcnPtr fcn = detail::resolveFunction<ReferenceMATLABStringEqualReferenceFcnPtr>
                (detail::FunctionType::REFERENCE_MATLAB_STRING_EQUAL_REFERENCE);
            return fcn(detail::Access::getImpl<detail::ReferenceImpl>(lhs),
                       detail::Access::getImpl<detail::ReferenceImpl>(rhs));
        }
        
        template<bool is_const_ref>
        bool operator ==(ArrayElementTypedRef<MATLABString, is_const_ref> const& lhs, ArrayElementTypedRef<MATLABString, is_const_ref> const& rhs) MW_NOEXCEPT {
            typedef bool(*ReferenceMATLABStringEqualReferenceFcnPtr)(detail::ReferenceImpl*,
                                                                     detail::ReferenceImpl*);
            static const ReferenceMATLABStringEqualReferenceFcnPtr fcn = detail::resolveFunction<ReferenceMATLABStringEqualReferenceFcnPtr>
                (detail::FunctionType::REFERENCE_MATLAB_STRING_EQUAL_REFERENCE);
            return fcn(detail::Access::getImpl<detail::ReferenceImpl>(lhs),
                       detail::Access::getImpl<detail::ReferenceImpl>(rhs));
        }
    }
}

#endif
