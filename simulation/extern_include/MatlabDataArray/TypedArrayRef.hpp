/* Copyright 2015-2017 The MathWorks, Inc. */

#ifndef MATLAB_EXTDATA_ARRAY_TYPEDARRAY_REF_HPP
#define MATLAB_EXTDATA_ARRAY_TYPEDARRAY_REF_HPP

#include "ArrayElementTypedRef.hpp"
#include "ArrayReferenceExt.hpp"
#include "Reference.hpp"
#include "TypedIterator.hpp"
#include "MDArray.hpp"
#include "Exception.hpp"
#include "String.hpp"
#include "Enumeration.hpp"
#include "Struct.hpp"
#include "Range.hpp"
#include "MatlabFieldIdentifier.hpp"
#include "TypedArray.hpp"
#include "ForwardIterator.hpp"

#include "detail/publish_util.hpp"
#include "detail/FunctionType.hpp"

#include <type_traits>

namespace matlab {
    namespace data {

        class CharArrayExt : public Reference<Array> {
          public:

            /**
             * Return contents of a CHAR array as a utf16 string
             *
             * @return std::basic_string<CHAR16_T> string
             * @throws none
             */
            String toUTF16() const MW_NOEXCEPT
            {
                const char16_t* str = nullptr;
                size_t strLen = 0;;
                typedef void(*CharArrayRefGetStringFcnPtr)(detail::ReferenceImpl*,
                                                           const char16_t**,
                                                           size_t*);
                static const CharArrayRefGetStringFcnPtr fcn = detail::resolveFunction<CharArrayRefGetStringFcnPtr>
                    (detail::FunctionType::CHAR_ARRAY_REF_GET_STRING);
                fcn(pImpl.get(), &str, &strLen);
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
                typedef int(*CharArrayRefGetAsciiFcnPtr)(detail::ReferenceImpl*,
                                                         const char16_t**,
                                                         size_t*);
                static const CharArrayRefGetAsciiFcnPtr fcn = detail::resolveFunction<CharArrayRefGetAsciiFcnPtr>
                    (detail::FunctionType::CHAR_ARRAY_REF_GET_ASCII);
                detail::throwIfError(fcn(pImpl.get(), &strVal, &strLen));
                return std::string(strVal, strVal+strLen);
            }

          protected:
            CharArrayExt(std::shared_ptr<detail::ReferenceImpl> impl) MW_NOEXCEPT :
                Reference<Array>(impl) {}

            CharArrayExt(const Reference<Array>& rhs ) :
                Reference<Array>(rhs) {}

          private:
            CharArrayExt() = delete;
        };

        class EnumArrayExt : public Reference<Array> {
          public:
            /**
             * Return class name for this Enum Array
             *
             * @return class name
             * @throw none
             */
            std::string getClassName() const MW_NOEXCEPT {
                const char* str = nullptr;
                size_t strlen = 0;
                typedef void(*EnumRefGetClassFcnPtr)(detail::ReferenceImpl*,
                                                     const char**,
                                                     size_t*);
                static const EnumRefGetClassFcnPtr fcn = detail::resolveFunction<EnumRefGetClassFcnPtr>
                    (detail::FunctionType::ENUM_REF_GET_CLASS);
                fcn(pImpl.get(),
                    &str,
                    &strlen);
                return std::string(str, strlen);
            }

          protected:
            EnumArrayExt(std::shared_ptr<detail::ReferenceImpl> impl) MW_NOEXCEPT :
                Reference<Array>(impl) {}

            EnumArrayExt(const Reference<Array>& rhs ) :
                Reference<Array>(rhs) {}

          private:
            EnumArrayExt() = delete;
        };
        
        class StructArrayExt : public Reference<Array> {
          public:
            /**
             * Returns a Range<ForwardIterator, MATLABFieldIdentifier const> which will iterate over all
             * of the fieldNames in a struct array.
             *
             * @return Range<ForwardIterator, MATLABFieldIdentifier const>
             * @throw none
             */
            Range<ForwardIterator, MATLABFieldIdentifier const> getFieldNames() const MW_NOEXCEPT {
                typedef detail::ForwardIteratorImpl*(*StructArrayRefBeginIDFcnPtr)(detail::ReferenceImpl*);
                static const StructArrayRefBeginIDFcnPtr b_fcn = detail::resolveFunction<StructArrayRefBeginIDFcnPtr>
                    (detail::FunctionType::STRUCT_ARRAY_REF_BEGIN_ID);
                typedef detail::ForwardIteratorImpl*(*StructArrayRefEndIDFcnPtr)(detail::ReferenceImpl*);
                static const StructArrayRefEndIDFcnPtr e_fcn = detail::resolveFunction<StructArrayRefEndIDFcnPtr>
                    (detail::FunctionType::STRUCT_ARRAY_REF_END_ID);
                return Range<ForwardIterator, MATLABFieldIdentifier const>(detail::Access::createObj<ForwardIterator<MATLABFieldIdentifier const>>(b_fcn(pImpl.get())),
                                                                           detail::Access::createObj<ForwardIterator<MATLABFieldIdentifier const>>(e_fcn(pImpl.get())));
            }

            /**
             * get the number of fields in this struct
             *
             * @return size_t the number of fields
             * @throw none
             */
            size_t getNumberOfFields() const MW_NOEXCEPT {
                typedef size_t(*StructArrayRefGetNumFieldsFcnPtr)(detail::ReferenceImpl*);
                static const StructArrayRefGetNumFieldsFcnPtr fcn = detail::resolveFunction<StructArrayRefGetNumFieldsFcnPtr>
                    (detail::FunctionType::STRUCT_ARRAY_REF_GET_NUM_FIELDS);
                return fcn(pImpl.get());
            }

          protected:
            StructArrayExt(std::shared_ptr<detail::ReferenceImpl> impl) MW_NOEXCEPT :
                Reference<Array>(impl) {}

            StructArrayExt(const Reference<Array>& rhs ) :
                Reference<Array>(rhs) {}

          private:
            StructArrayExt() = delete;
        };

        template <typename T> struct GetReferenceArrayExt      { using type = Reference<Array>; };
        template <> struct GetReferenceArrayExt<CHAR16_T>      { using type = CharArrayExt; };
        template <> struct GetReferenceArrayExt<Enumeration>   { using type = EnumArrayExt; };
        template <> struct GetReferenceArrayExt<Struct> { using type = StructArrayExt; };

        /**
         * The Reference<TypedArray<T>> class is a specialization of Reference. It provides standard array operation
         * found in ArrayRef as well as iterators and type-safe indexing. Reference<TypedArray<T>> is not thread-safe - they should not
         * be passed between threads. 
         *
         * @code
         *    auto cellArray = factory.createArray<Array>({2,2});
         *    cellArray[0][0] = factory.createArray({1,5}, {1.0, 2.0, 3.0, 4.0, 5.0});
         *    TypedArrayRef<double> ref(cellArray[0][0]);
         *
         *    for (auto& elem : ref) {
         *        elem += 1;
         *    }
         *    ref[4] = 10.0;
         * @endcode
         */
        template <typename T>
        class Reference<TypedArray<T>> : public GetReferenceArrayExt<T>::type {
          public:

            using Parent = typename GetReferenceArrayExt<T>::type;
            using Parent::pImpl;

            typedef TypedIterator<T> iterator;
            typedef TypedIterator<typename std::add_const<T>::type> const_iterator;

            typedef typename std::remove_const<T>::type arr_elem_type;
            typedef TypedArray<T> ref_type;

            /**
             * Create a Reference<TypedArray<T>> from a Reference<Array>
             *
             * @code
             *    Reference<Array> ref = cellArr[0][0];
             *    TypedArrayRef<double> d_ref(ref);
             * @endcode
             *
             * @return Reference<TypedArray<T>>
             *
             * @throw TypeMismatchException - if the element of the Array does not match <T>
             */
            Reference(const Reference<Array>& rhs ) :
                Parent(rhs) {
                detail::validateUntypedArrayReference<T>(pImpl.get());
            }


            /**
             * Enables [] indexing on a reference to an Array.
             *
             * The return value ArrayElementRef<is_const_ref> allows the element of the array to be
             * modified or retrieved: For example:
             *     auto cell_arr = factory.createArray<Array>({2,2});
             *     cell_arr[0][0] = factory.createArray<double>({3,3});
             *     Reference<Array> ref = cell_arr[0][0];
             *     ref[0][0] = 5.5;
             *
             * @param idx - the first array index
             *
             * @return ArrayElementRef<is_const_ref> - contains the index specified
             *
             * @throw InvalidFieldNameException if the Reference<Array> was created with an invalid fieldname for a struct
             * @throw CantIndexIntoEmptyArrayException - if the array is empty
             */
            ArrayElementTypedRef<arr_elem_type, std::is_const<T>::value> operator[](size_t idx) {
                detail::ReferenceImpl* newRef = nullptr;
                typedef int(*ArrayReferenceGetReferenceFcnPtr)(detail::ReferenceImpl*,
                                                               bool,
                                                               size_t,
                                                               detail::ReferenceImpl**);
                static const ArrayReferenceGetReferenceFcnPtr fcn = detail::resolveFunction<ArrayReferenceGetReferenceFcnPtr>
                    (detail::FunctionType::ARRAY_REFERENCE_GET_REFERENCE);
                detail::throwIfError(fcn(pImpl.get(), !std::is_const<T>::value, idx, &newRef));
                return detail::Access::createObj<ArrayElementTypedRef<arr_elem_type, std::is_const<T>::value>>(newRef);
            }

            /**
             * Assign a TypedArray into an Array. The Array being indexed must be non-const.
             *
             * @param rhs - value to be assigned into the Array
             *
             * @return Reference<TypedArray<T>>& - this
             * @throw none
             */
            Reference<TypedArray<T>>& operator= (TypedArray<T> rhs) MW_NOEXCEPT {
                detail::setElement(pImpl.get(), std::move(rhs), static_cast<int>(GetArrayType<T>::type));
                return *this;
            }

            /**
             * Enables [] indexing on a reference to an Array.
             *
             * The return value ArrayElementRef<true> allows the element of the array to be
             * modified or retrieved: For example:
             *     auto cell_arr = factory.createArray<Array>({2,2});
             *     cell_arr[0][0] = factory.createScalar(5.5);
             *     Reference<Array const> ref = cell_arr[0][0];
             *     double val = ref[0];
             *
             * @param idx - the first array index
             *
             * @return ArrayElementRef<true> - contains the index specified
             *
             * @throw InvalidFieldNameException if the Reference<Array> was created with an invalid fieldname for a struct
             * @throw CantIndexIntoEmptyArrayException - if the array is empty
             */
            ArrayElementTypedRef<arr_elem_type, true> operator[](size_t idx) const {
                detail::ReferenceImpl* newRef = nullptr;
                typedef int(*ArrayReferenceGetReferenceFcnPtr)(detail::ReferenceImpl*,
                                                               bool,
                                                               size_t,
                                                               detail::ReferenceImpl**);
                static const ArrayReferenceGetReferenceFcnPtr fcn = detail::resolveFunction<ArrayReferenceGetReferenceFcnPtr>
                    (detail::FunctionType::ARRAY_REFERENCE_GET_REFERENCE);
                detail::throwIfError(fcn(pImpl.get(), false, idx, &newRef));
                return detail::Access::createObj<ArrayElementTypedRef<arr_elem_type, true>>(newRef);
            }

            /**
             * Return an iterator to the beginning of the array
             *
             * @return iterator
             * @throw none
             */
            iterator begin() MW_NOEXCEPT {
                typedef detail::IteratorImpl*(*ArrayReferenceBeginFcnPtr)(detail::ReferenceImpl*, bool);
                static const ArrayReferenceBeginFcnPtr fcn = detail::resolveFunction<ArrayReferenceBeginFcnPtr>
                    (detail::FunctionType::ARRAY_REFERENCE_BEGIN);
                return detail::Access::createObj<iterator>(fcn(pImpl.get(), !std::is_const<T>::value));
            }

            /**
             * Return an const_iterator to the beginning of the array
             *
             * @return const_iterator
             * @throw none
             */
            const_iterator begin() const MW_NOEXCEPT {
                return cbegin();
            }

            /**
             * Return an const_iterator to the beginning of the array
             *
             * @return const_iterator
             * @throw none
             */
            const_iterator cbegin() const MW_NOEXCEPT {
                typedef detail::IteratorImpl*(*ArrayReferenceBeginFcnPtr)(detail::ReferenceImpl*, bool);
                static const ArrayReferenceBeginFcnPtr fcn = detail::resolveFunction<ArrayReferenceBeginFcnPtr>
                    (detail::FunctionType::ARRAY_REFERENCE_BEGIN);
                return detail::Access::createObj<const_iterator>(fcn(pImpl.get(), false));
            }

            /**
             * Return an iterator to the end of the array
             *
             * @return iterator
             * @throw none
             */
            iterator end() MW_NOEXCEPT {
                typedef detail::IteratorImpl*(*ArrayReferenceEndFcnPtr)(detail::ReferenceImpl*, bool);
                static const ArrayReferenceEndFcnPtr fcn = detail::resolveFunction<ArrayReferenceEndFcnPtr>
                    (detail::FunctionType::ARRAY_REFERENCE_END);
                return detail::Access::createObj<iterator>(fcn(pImpl.get(), !std::is_const<T>::value));
            }

            /**
             * Return an const_iterator to the end of the array
             *
             * @return const_iterator
             * @throw none
             */
            const_iterator end() const MW_NOEXCEPT {
                return cend();
            }

            /**
             * Return an const_iterator to the end of the array
             *
             * @return const_iterator
             * @throw none
             */
            const_iterator cend() const MW_NOEXCEPT {
                typedef detail::IteratorImpl*(*ArrayReferenceEndFcnPtr)(detail::ReferenceImpl*, bool);
                static const ArrayReferenceEndFcnPtr fcn = detail::resolveFunction<ArrayReferenceEndFcnPtr>
                    (detail::FunctionType::ARRAY_REFERENCE_END);
                return detail::Access::createObj<const_iterator>(fcn(pImpl.get(), false));
            }

          protected:

            friend class detail::Access;
            
            Reference(std::shared_ptr<detail::ReferenceImpl> impl) MW_NOEXCEPT :
                Parent(impl) {}

            Reference() = delete;
        };

        using ArrayRef = Reference<Array>;
        
        template <typename T>
        using TypedArrayRef = Reference<TypedArray<T>>;

        using CellArrayRef = Reference<TypedArray<Array>>;
        using StructArrayRef = Reference<TypedArray<Struct>>;
        
        /**
         * Get a range containing the elements of the Array. Iterators contained in the range are const.
         *
         * @param arr - Array
         * @return Range<TypedIterator, T const>
         * @throw InvalidArrayTypeException - if the array does not contain T's
         */
        template <typename T>
        Range<TypedIterator, T const> getReadOnlyElements(const Array& arr) {
            typedef int(*TypedArrayIsValidConversionFcnPtr)(int, int, bool*);
            static const TypedArrayIsValidConversionFcnPtr fcn = detail::resolveFunction<TypedArrayIsValidConversionFcnPtr>
                (detail::FunctionType::TYPED_ARRAY_IS_VALID_CONVERSION);
            bool result;
            detail::throwIfError(fcn(static_cast<int>(GetArrayType<T>::type), static_cast<int>(arr.getType()), &result));
            if (!result) {
                throw InvalidArrayTypeException("Can't convert the Array to this TypedArray");
            }
            typedef detail::IteratorImpl*(*TypedArrayBeginFcnPtr)(impl::ArrayImpl*, bool);
            static const TypedArrayBeginFcnPtr b_fcn = detail::resolveFunction<TypedArrayBeginFcnPtr>
                (detail::FunctionType::TYPED_ARRAY_BEGIN);
            typedef detail::IteratorImpl*(*TypedArrayEndFcnPtr)(impl::ArrayImpl*, bool);
            static const TypedArrayEndFcnPtr e_fcn = detail::resolveFunction<TypedArrayEndFcnPtr>
                (detail::FunctionType::TYPED_ARRAY_END);
            return Range<TypedIterator, T const>(detail::Access::createObj<TypedIterator<T const>>(b_fcn(detail::Access::getImpl<impl::ArrayImpl>(arr), false)),
                                                 detail::Access::createObj<TypedIterator<T const>>(e_fcn(detail::Access::getImpl<impl::ArrayImpl>(arr), false)));
        }

        /**
         * Get a range containing the elements of the Array. Iterators contained in the range are non-const.
         *
         * @param arr - Array
         * @return Range<TypedIterator, T>
         * @throw InvalidArrayTypeException - if the array does not contain T's
         */
        template <typename T>
        Range<TypedIterator, T> getWritableElements(Array& arr) {
            typedef int(*TypedArrayIsValidConversionFcnPtr)(int, int, bool*);
            static const TypedArrayIsValidConversionFcnPtr fcn = detail::resolveFunction<TypedArrayIsValidConversionFcnPtr>
                (detail::FunctionType::TYPED_ARRAY_IS_VALID_CONVERSION);
            bool result;
            detail::throwIfError(fcn(static_cast<int>(GetArrayType<T>::type), static_cast<int>(arr.getType()), &result));
            if (!result) {
                throw InvalidArrayTypeException("Can't convert the Array to this TypedArray");
            }
            impl::ArrayImpl* newImpl = nullptr;
            typedef bool(*ArrayUnshareFcnPtr)(impl::ArrayImpl*, bool, impl::ArrayImpl**);
            static const ArrayUnshareFcnPtr u_fcn = detail::resolveFunction<ArrayUnshareFcnPtr>
                (detail::FunctionType::ARRAY_UNSHARE);
            if (u_fcn(arr.pImpl.get(), arr.pImpl.unique(), &newImpl)) {
                arr.pImpl.reset(newImpl, [](impl::ArrayImpl* ptr) {
                    typedef void(*ArrayDestroyFcnPtr)(impl::ArrayImpl*);
                    static const ArrayDestroyFcnPtr d_fcn = detail::resolveFunction<ArrayDestroyFcnPtr>
                        (detail::FunctionType::ARRAY_DESTROY);
                    d_fcn(ptr);
                });
            }
            typedef detail::IteratorImpl*(*TypedArrayBeginFcnPtr)(impl::ArrayImpl*, bool);
            static const TypedArrayBeginFcnPtr b_fcn = detail::resolveFunction<TypedArrayBeginFcnPtr>
                (detail::FunctionType::TYPED_ARRAY_BEGIN);
            typedef detail::IteratorImpl*(*TypedArrayEndFcnPtr)(impl::ArrayImpl*, bool);
            static const TypedArrayEndFcnPtr e_fcn = detail::resolveFunction<TypedArrayEndFcnPtr>
                (detail::FunctionType::TYPED_ARRAY_END);
            return Range<TypedIterator, T>(detail::Access::createObj<TypedIterator<T>>(b_fcn(arr.pImpl.get(), true)),
                                           detail::Access::createObj<TypedIterator<T>>(e_fcn(arr.pImpl.get(), true)));
        }

        /**
         * Get a range containing the elements of the Reference<Array>. Iterators contained in the range are const.
         *
         * @param ref - Reference<Array>
         * @return Range<TypedIterator, T const>
         * @throw InvalidArrayTypeException - if the array does not contain T's
         */
        template <typename T>
        Range<TypedIterator, T const> getReadOnlyElements(const Reference<Array>& ref) {
            typedef int(*TypedArrayReferenceValidateTypeFcnPtr)(detail::ReferenceImpl*, int, bool);
            static const TypedArrayReferenceValidateTypeFcnPtr fcn = detail::resolveFunction<TypedArrayReferenceValidateTypeFcnPtr>
                (detail::FunctionType::TYPED_ARRAY_REFERENCE_VALIDATE_TYPE);
            detail::throwIfError(fcn(detail::Access::getImpl<detail::ReferenceImpl>(ref), static_cast<int>(ref.getType()), false));
            typedef detail::IteratorImpl*(*ArrayReferenceBeginFcnPtr)(detail::ReferenceImpl*, bool);
            static const ArrayReferenceBeginFcnPtr b_fcn = detail::resolveFunction<ArrayReferenceBeginFcnPtr>
                (detail::FunctionType::ARRAY_REFERENCE_BEGIN);
            typedef detail::IteratorImpl*(*ArrayReferenceEndFcnPtr)(detail::ReferenceImpl*, bool);
            static const ArrayReferenceEndFcnPtr e_fcn = detail::resolveFunction<ArrayReferenceEndFcnPtr>
                (detail::FunctionType::ARRAY_REFERENCE_END);
            return Range<TypedIterator, T const>(detail::Access::createObj<TypedIterator<T const>>(b_fcn(detail::Access::getImpl<detail::ReferenceImpl>(ref), false)),
                                                 detail::Access::createObj<TypedIterator<T const>>(e_fcn(detail::Access::getImpl<detail::ReferenceImpl>(ref), false)));
        }

        /**
         * Get a range containing the elements of the Reference<Array>. Iterators contained in the range are non-const.
         *
         * @param ref - Reference<Array>
         * @return Range<TypedIterator, T>
         * @throw InvalidArrayTypeException - if the array does not contain T's
         */
        template <typename T>
        Range<TypedIterator, T> getWritableElements(Reference<Array>& ref) {
            typedef int(*TypedArrayReferenceValidateTypeFcnPtr)(detail::ReferenceImpl*, int, bool);
            static const TypedArrayReferenceValidateTypeFcnPtr fcn = detail::resolveFunction<TypedArrayReferenceValidateTypeFcnPtr>
                (detail::FunctionType::TYPED_ARRAY_REFERENCE_VALIDATE_TYPE);
            detail::throwIfError(fcn(detail::Access::getImpl<detail::ReferenceImpl>(ref), static_cast<int>(ref.getType()), true));
            typedef detail::IteratorImpl*(*ArrayReferenceBeginFcnPtr)(detail::ReferenceImpl*, bool);
            static const ArrayReferenceBeginFcnPtr b_fcn = detail::resolveFunction<ArrayReferenceBeginFcnPtr>
                (detail::FunctionType::ARRAY_REFERENCE_BEGIN);
            typedef detail::IteratorImpl*(*ArrayReferenceEndFcnPtr)(detail::ReferenceImpl*, bool);
            static const ArrayReferenceEndFcnPtr e_fcn = detail::resolveFunction<ArrayReferenceEndFcnPtr>
                (detail::FunctionType::ARRAY_REFERENCE_END);
            return Range<TypedIterator, T>(detail::Access::createObj<TypedIterator<T>>(b_fcn(detail::Access::getImpl<detail::ReferenceImpl>(ref), true)),
                                           detail::Access::createObj<TypedIterator<T>>(e_fcn(detail::Access::getImpl<detail::ReferenceImpl>(ref), true)));
        }        
    }
}

#endif
