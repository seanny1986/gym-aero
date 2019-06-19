/* Copyright 2017 The MathWorks, Inc. */

#ifndef MATLAB_EXTDATA_ARRAY_SPARSEARRAY_REF_HPP
#define MATLAB_EXTDATA_ARRAY_SPARSEARRAY_REF_HPP

#include "Reference.hpp"
#include "TypedIterator.hpp"
#include "SparseArray.hpp"
#include "ArrayReferenceExt.hpp"

#include "detail/publish_util.hpp"
#include "detail/FunctionType.hpp"

#include <type_traits>

namespace matlab {
    namespace data {
        /**
         * This specialization of Reference class adds APIs to a reference to a SparseArray. Reference<> is not thread-safe - it should not
         * be passed between threads. 
         *
         * @code
         *    auto cellArray = factory.createArray<Array>({2,2});
         *    cellArray[0][0] = factory.createSparseArray<double>(...);
         *    SparseArrayRef<double> ref(cellArray[0][0]);
         *    
         *    auto nnz = ref.getNumberOfNonZeroElements();
         *
         * @endcode
         */
        template <typename T>
        class Reference<SparseArray<T>> : public ArrayReferenceExt {
          public:

            typedef TypedIterator<T> iterator;
            typedef TypedIterator<typename std::add_const<T>::type> const_iterator;

            /**
             * Construct a Reference<SparseArray<T>> from a Reference<Array>
             *
             * @param - rhs Reference value to be copied
             * @return - newly constructed Reference
             * @throw TypeMismatchException - if the element of the Array is not a SparseArray<T>
             */
            Reference(const Reference<Array>& rhs) :
                ArrayReferenceExt(rhs) {
                detail::validateUntypedArrayReference<SparseArray<T>>(pImpl.get());
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

            /**
             * This API returns the number of non-zero elements in the array. 
             * Since sparse arrays only store non-zero elements, this API effectively
             * returns the actual array size.  It is different from array dimensions
             * that specify the full array size.
             * @return Number of non-zero elements in the array
             * @throw None
             */
            size_t getNumberOfNonZeroElements() const MW_NOEXCEPT {
                size_t val;
                typedef void(*SparseArrayRefGetNumNonZeroElementsFcnPtr)(detail::ReferenceImpl*, size_t*);
                static const SparseArrayRefGetNumNonZeroElementsFcnPtr fcn = detail::resolveFunction<SparseArrayRefGetNumNonZeroElementsFcnPtr>
                    (detail::FunctionType::SPARSE_ARRAY_REF_GET_NUM_NONZERO_ELEMENTS);
                fcn(pImpl.get(), &val);
                return val;
            }

          protected:

            friend class detail::Access;
            
            Reference(std::shared_ptr<detail::ReferenceImpl> impl) MW_NOEXCEPT :
                ArrayReferenceExt(impl) {}

            Reference() = delete;
        };

        template <typename T>
        using SparseArrayRef = Reference<SparseArray<T>>;
    }
}

#endif
