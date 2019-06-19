/* Copyright 2014-2018 The MathWorks, Inc. */

#ifndef TYPED_ARRAY_HPP_
#define TYPED_ARRAY_HPP_


#include "matlab_data_array_defs.hpp"
#include "matlab_extdata_defs.hpp"
#include "ArrayDimensions.hpp"
#include "ArrayType.hpp"
#include "GetArrayType.hpp"
#include "Range.hpp"
#include "Exception.hpp"

#include "ArrayElementTypedRef.hpp"
#include "TypedIterator.hpp"
#include "MDArray.hpp"
#include "Reference.hpp"

#include "Optional.hpp"

#include "detail/publish_util.hpp"
#include "detail/FunctionType.hpp"

#include <functional>
#include <stdint.h>
#include <exception>
#include <type_traits>

namespace matlab {
    namespace data {
        namespace impl {
            class ArrayImpl;
        }

        namespace detail {
            class Access;
        }
        

        /**
         * TypedArray provides an API to access numeric, logical and char data. 
         */
        template<typename T>
        class TypedArray : public Array {
          public:

            using iterator = TypedIterator<T>;
            using const_iterator = TypedIterator<typename std::add_const<T>::type>;

            using reference = typename iterator::reference;
            using const_reference = typename const_iterator::reference;

            using data_type = T;

            static const ArrayType type = GetArrayType<typename std::remove_const<T>::type>::type;

            /**
             * Destructor
             *
             * @throw none
             */
            virtual ~TypedArray() MW_NOEXCEPT
            {}
            
            /**
             * Move assignment operator
             *
             * @param rhs - TypedArray<T> to be moved
             * @return - the updated TypedArray
             * @throw none
             */
            TypedArray<T>& operator=(TypedArray<T>&& rhs) MW_NOEXCEPT {
                Array::operator=(std::move(rhs));
                return *this;
            }

            /**
             * Move constructor
             *
             * @param rhs - TypedArray<T> to be moved
             * @return - newly constructed TypedArray
             * @throw none
             */
            TypedArray(TypedArray<T>&& rhs) MW_NOEXCEPT :
                Array(std::move(rhs)) {}

            /**
             * Construct a TypedArray from an Array
             *
             * @param rhs - Array to be moved
             * @return - newly constructed TypedArray
             * @throw InvalidArrayTypeException if type of rhs does not match the type for TypedArray<T>
             */
            TypedArray(Array&& rhs) :
                Array(std::move(rhs)) {
                typedef int(*TypedArrayIsValidConversionFcnPtr)(int, int, bool*);
                static const TypedArrayIsValidConversionFcnPtr fcn = detail::resolveFunction<TypedArrayIsValidConversionFcnPtr>
                    (detail::FunctionType::TYPED_ARRAY_IS_VALID_CONVERSION);
                bool result;
                detail::throwIfError(fcn(static_cast<int>(GetArrayType<T>::type), static_cast<int>(getType()), &result));
                if (!result) {
                    rhs = std::move(*this);
                    throw InvalidArrayTypeException("Can't convert the Array to this TypedArray");
                }
            }

            /**
             * move assignment 
             *
             * @param rhs - Array to be moved
             * @return - the updated TypedArray
             * @throw InvalidArrayTypeException if type of rhs does not match the type for TypedArray<T>
             */
            TypedArray<T>& operator=(Array&& rhs) {
                typedef int(*TypedArrayIsValidConversionFcnPtr)(int, int, bool*);
                static const TypedArrayIsValidConversionFcnPtr fcn = detail::resolveFunction<TypedArrayIsValidConversionFcnPtr>
                    (detail::FunctionType::TYPED_ARRAY_IS_VALID_CONVERSION);
                bool result;
                detail::throwIfError(fcn(static_cast<int>(GetArrayType<T>::type), static_cast<int>(rhs.getType()), &result));
                if (!result) {
                    throw InvalidArrayTypeException("Can't convert the Array to this TypedArray");
                }
                Array::operator=(std::move(rhs));
                return *this;
            }

            /**
             * Copy constructor - creates a shared copy of the input argument
             *
             * @param rhs - TypedArray<T> to be copied
             * @return - newly constructed TypedArray
             * @throw none
             */
            TypedArray(const TypedArray<T> &rhs) MW_NOEXCEPT :
                Array(rhs) {}

            /**
             * Copy construct a TypedArray from an Array 
             *
             * @param rhs - Array to be copied
             * @return - newly constructed TypedArray
             * @throw InvalidArrayTypeException if type of rhs does not match the type for TypedArray<T>
             *                                  or is not recognized as valid.
             */
            TypedArray(const Array& rhs) :
                Array(rhs) {
                typedef int(*TypedArrayIsValidConversionFcnPtr)(int, int, bool*);
                static const TypedArrayIsValidConversionFcnPtr fcn = detail::resolveFunction<TypedArrayIsValidConversionFcnPtr>
                    (detail::FunctionType::TYPED_ARRAY_IS_VALID_CONVERSION);
                bool result;
                detail::throwIfError(fcn(static_cast<int>(GetArrayType<T>::type), static_cast<int>(rhs.getType()), &result));
                if (!result) {
                    throw InvalidArrayTypeException("Can't convert the Array to this TypedArray");
                }
            }

            /**
             * Operator= that assigns a shared copy of the input Array to this
             *
             * @param rhs - Array to be copied
             * @return - the updated TypedArray
             * @throw InvalidArrayTypeException if type of rhs does not match the type for TypedArray<T>
             */
            TypedArray<T>& operator=(const Array& rhs) {
                typedef int(*TypedArrayIsValidConversionFcnPtr)(int, int, bool*);
                static const TypedArrayIsValidConversionFcnPtr fcn = detail::resolveFunction<TypedArrayIsValidConversionFcnPtr>
                    (detail::FunctionType::TYPED_ARRAY_IS_VALID_CONVERSION);
                bool result;
                detail::throwIfError(fcn(static_cast<int>(GetArrayType<T>::type), static_cast<int>(rhs.getType()), &result));
                if (!result) {
                    throw InvalidArrayTypeException("Can't convert the Array to this TypedArray");
                }
                Array::operator=(rhs);
                return *this;
            }

            /**
             * Operator= that assigns a shared copy of the input TypedArray to this
             *
             * @param rhs - Array to be copied
             * @return - the updated TypedArray
             * @throw none
             */
            TypedArray<T>& operator=(const TypedArray<T>& rhs) MW_NOEXCEPT {
                Array::operator=(rhs);
                return *this;
            }

            /**
             * Return an iterator to the beginning of the array
             *
             * @return iterator
             * @throw none
             */
            iterator begin() MW_NOEXCEPT {
                impl::ArrayImpl* newImpl = nullptr;
                typedef bool(*ArrayUnshareFcnPtr)(impl::ArrayImpl*, bool, impl::ArrayImpl**);
                static const ArrayUnshareFcnPtr fcn = detail::resolveFunction<ArrayUnshareFcnPtr>
                    (detail::FunctionType::ARRAY_UNSHARE);
                if (fcn(pImpl.get(), pImpl.unique(), &newImpl)) {
                    pImpl.reset(newImpl, [](impl::ArrayImpl* ptr) {
                        typedef void(*ArrayDestroyFcnPtr)(impl::ArrayImpl*);
                        static const ArrayDestroyFcnPtr fcn2 = detail::resolveFunction<ArrayDestroyFcnPtr>
                            (detail::FunctionType::ARRAY_DESTROY);
                        fcn2(ptr);
                    });
                }
                typedef detail::IteratorImpl*(*TypedArrayBeginFcnPtr)(impl::ArrayImpl*, bool);
                static const TypedArrayBeginFcnPtr fcn3 = detail::resolveFunction<TypedArrayBeginFcnPtr>
                    (detail::FunctionType::TYPED_ARRAY_BEGIN);
                return detail::Access::createObj<iterator>(fcn3(pImpl.get(), !std::is_const<T>::value));
            }

            /**
             * Return an const_iterator to the beginning of the array
             *
             * @return const_iterator
             * @throw none
             */
            const_iterator begin() const MW_NOEXCEPT {
                typedef detail::IteratorImpl*(*TypedArrayBeginFcnPtr)(impl::ArrayImpl*, bool);
                static const TypedArrayBeginFcnPtr fcn = detail::resolveFunction<TypedArrayBeginFcnPtr>
                    (detail::FunctionType::TYPED_ARRAY_BEGIN);
                return detail::Access::createObj<const_iterator>(fcn(pImpl.get(), false));
            }

            /**
             * Return an const_iterator to the beginning of the array
             *
             * @return const_iterator
             * @throw none
             */
            const_iterator cbegin() const MW_NOEXCEPT {
                return begin();
            }

            /**
             * Return an iterator to the end of the array
             *
             * @return iterator
             * @throw none
             */
            iterator end() MW_NOEXCEPT {
                impl::ArrayImpl* newImpl = nullptr;
                typedef bool(*ArrayUnshareFcnPtr)(impl::ArrayImpl*, bool, impl::ArrayImpl**);
                static const ArrayUnshareFcnPtr fcn = detail::resolveFunction<ArrayUnshareFcnPtr>
                    (detail::FunctionType::ARRAY_UNSHARE);
                if (fcn(pImpl.get(), pImpl.unique(), &newImpl)) {
                    pImpl.reset(newImpl, [](impl::ArrayImpl* ptr) {
                        typedef void(*ArrayDestroyFcnPtr)(impl::ArrayImpl*);
                        static const ArrayDestroyFcnPtr fcn2 = detail::resolveFunction<ArrayDestroyFcnPtr>
                            (detail::FunctionType::ARRAY_DESTROY);
                        fcn2(ptr);
                    });
                }
                typedef detail::IteratorImpl*(*TypedArrayEndFcnPtr)(impl::ArrayImpl*, bool);
                static const TypedArrayEndFcnPtr fcn3 = detail::resolveFunction<TypedArrayEndFcnPtr>
                    (detail::FunctionType::TYPED_ARRAY_END);
                return detail::Access::createObj<iterator>(fcn3(pImpl.get(), !std::is_const<T>::value));
            }

            /**
             * Return an const_iterator to the end of the array
             *
             * @return const_iterator
             * @throw none
             */
            const_iterator end() const MW_NOEXCEPT {
                typedef detail::IteratorImpl*(*TypedArrayEndFcnPtr)(impl::ArrayImpl*, bool);
                static const TypedArrayEndFcnPtr fcn = detail::resolveFunction<TypedArrayEndFcnPtr>
                    (detail::FunctionType::TYPED_ARRAY_END);
                return detail::Access::createObj<const_iterator>(fcn(pImpl.get(), false));
            }

            /**
             * Return an const_iterator to the end of the array
             *
             * @return const_iterator
             * @throw none
             */
            const_iterator cend() const MW_NOEXCEPT {
                return end();
            }

            /**
             * Enables [] indexing on a TypedArray.
             *
             * The return value ArrayElementTypedRef<T, std::is_const<T>::value> allows the element of the array to be
             * retrieved or modified (if not const): For example:
             *     arr[1][1] = 5.5;
             *     double val = arr[0][3];
             *
             * @param idx - the first array index
             * @return ArrayElementTypedRef<T, std::is_const<T>::value> - contains the index specified
             * @throw InvalidArrayIndexException - if the index is invalid
             * @throw CantIndexIntoEmptyArrayException - if the array is empty
             */
            ArrayElementTypedRef<T, std::is_const<T>::value> operator[](size_t idx) {
                impl::ArrayImpl* newImpl = nullptr;
                typedef bool(*ArrayUnshareFcnPtr)(impl::ArrayImpl*, bool, impl::ArrayImpl**);
                static const ArrayUnshareFcnPtr fcn = detail::resolveFunction<ArrayUnshareFcnPtr>
                    (detail::FunctionType::ARRAY_UNSHARE);
                if (fcn(pImpl.get(), pImpl.unique(), &newImpl)) {
                    pImpl.reset(newImpl, [](impl::ArrayImpl* ptr) {
                        typedef void(*ArrayDestroyFcnPtr)(impl::ArrayImpl*);
                        static const ArrayDestroyFcnPtr fcn2 = detail::resolveFunction<ArrayDestroyFcnPtr>
                            (detail::FunctionType::ARRAY_DESTROY);
                        fcn2(ptr);
                    });
                }
                detail::ReferenceImpl* impl = nullptr;
                typedef int(*ArrayCreateReferenceFcnPtr)(impl::ArrayImpl*, size_t, detail::ReferenceImpl**);
                static const ArrayCreateReferenceFcnPtr fcn3 = detail::resolveFunction<ArrayCreateReferenceFcnPtr>
                    (detail::FunctionType::ARRAY_CREATE_REFERENCE);
                detail::throwIfError(fcn3(pImpl.get(), idx, &impl));
                return detail::Access::createObj<ArrayElementTypedRef<T, std::is_const<T>::value>>(impl);
            }

            /**
             * Enables [] indexing on a TypedArray.
             *
             * The return value ArrayElementTypedRef<T, true> allows the element of the array to be
             * modified or retrieved: For example:
             *     double val = arr[0][3];
             *
             * @param idx - the first array index
             * @return ArrayElementTypedRef<T, true> - contains the index specified
             * @throw InvalidArrayIndexException - if the index is invalid
             * @throw CantIndexIntoEmptyArrayException - if the array is empty
             */
            ArrayElementTypedRef<T, true> operator[](size_t idx) const {
                detail::ReferenceImpl* impl = nullptr;
                typedef int(*ArrayCreateReferenceFcnPtr)(impl::ArrayImpl*, size_t, detail::ReferenceImpl**);
                static const ArrayCreateReferenceFcnPtr fcn = detail::resolveFunction<ArrayCreateReferenceFcnPtr>
                    (detail::FunctionType::ARRAY_CREATE_REFERENCE);
                detail::throwIfError(fcn(pImpl.get(), idx, &impl));
                return detail::Access::createObj<ArrayElementTypedRef<T, true>>(impl);
            }

            /**
             * Release the underlying buffer from the Array. If the Array is currently
             * shared, a copy of the buffer is made; otherwise no copy is made. After the
             * buffer is released, the array contains no elements.
             *
             * @return buffer_ptr_t<T> - unique_ptr with the data pointer
             * @throw InvalidArrayTypeException - if this TypedArray does not support releasing the buffer
             */
            buffer_ptr_t<T> release() {
                void* buffer = nullptr;
                buffer_deleter_t deleter = nullptr;
                detail::ReferenceImpl* impl = nullptr;
                typedef int(*ArrayReleaseBufferFcnPtr)(impl::ArrayImpl*, bool, void**, void(**deleter)(void*));
                static const ArrayReleaseBufferFcnPtr fcn = detail::resolveFunction<ArrayReleaseBufferFcnPtr>
                    (detail::FunctionType::ARRAY_RELEASE_BUFFER);
                detail::throwIfError(fcn(pImpl.get(), pImpl.unique(), &buffer, &deleter));
                return buffer_ptr_t<T>(static_cast<T*>(buffer), deleter);
            }

          protected:
            friend class detail::Access;
            
            TypedArray(impl::ArrayImpl* impl) MW_NOEXCEPT :
                Array(impl) {}

            TypedArray() = delete;

        };

        using CellArray = TypedArray<Array>;
        using StringArray = TypedArray<MATLABString>;
    }
}

#endif
