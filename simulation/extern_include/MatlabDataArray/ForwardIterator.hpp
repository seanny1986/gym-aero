/* Copyright 2016-2017 The MathWorks, Inc. */

#ifndef MATLAB_DATA_FORWARD_ITERATOR_HPP
#define MATLAB_DATA_FORWARD_ITERATOR_HPP
        
#include "detail/publish_util.hpp"
#include "detail/FunctionType.hpp"

#include <memory>
#include <iterator>
#include <functional>

namespace matlab {
    namespace data {
        namespace detail {
            class Access;
            class RefCounted;
            class ForwardIteratorImpl;
        }

        template<typename T>
        class ForwardIterator {
          public:

            typedef std::ptrdiff_t difference_type;
            typedef T value_type;
            typedef T* pointer;
            typedef T reference;
            typedef std::forward_iterator_tag iterator_category;

            /**
             * ForwardIterator copy constructor. Guaranteed not to throw.
             *
             * @param rhs ForwardIterator object to be copied.
             * @throw none.
             */
            ForwardIterator(const ForwardIterator<T> &rhs) MW_NOEXCEPT :
                fIndex(rhs.fIndex) {
                typedef detail::ForwardIteratorImpl*(*ForwardIteratorCloneFcnPtr)(detail::ForwardIteratorImpl*);
                static const ForwardIteratorCloneFcnPtr fcn = detail::resolveFunction<ForwardIteratorCloneFcnPtr>
                    (detail::FunctionType::FORWARD_ITERATOR_CLONE);
                pImpl = std::shared_ptr<detail::ForwardIteratorImpl>
                    (fcn(rhs.pImpl.get()), 
                     [](detail::ForwardIteratorImpl* ptr) {
                        typedef void(*ForwardIteratorDestroyFcnPtr)(detail::ForwardIteratorImpl*);
                        static const ForwardIteratorDestroyFcnPtr fcn2 = detail::resolveFunction<ForwardIteratorDestroyFcnPtr>
                            (detail::FunctionType::FORWARD_ITERATOR_DESTROY);
                        fcn2(ptr);
                    });
            }
            
            /**
             * ForwardIterator operator=.
             *
             * @param rhs ForwardIterator object to assign to this instance.
             * @return the updated instance.
             * @throw none
             */
            ForwardIterator<T>& operator=(const ForwardIterator<T> &rhs) MW_NOEXCEPT {
                typedef detail::ForwardIteratorImpl*(*ForwardIteratorCloneFcnPtr)(detail::ForwardIteratorImpl*);
                static const ForwardIteratorCloneFcnPtr fcn = detail::resolveFunction<ForwardIteratorCloneFcnPtr>
                    (detail::FunctionType::FORWARD_ITERATOR_CLONE);
                pImpl = std::shared_ptr<detail::ForwardIteratorImpl>
                    (fcn(rhs.pImpl.get()), 
                     [](detail::ForwardIteratorImpl* ptr) {
                        typedef void(*ForwardIteratorDestroyFcnPtr)(detail::ForwardIteratorImpl*);
                        static const ForwardIteratorDestroyFcnPtr fcn2 = detail::resolveFunction<ForwardIteratorDestroyFcnPtr>
                            (detail::FunctionType::FORWARD_ITERATOR_DESTROY);
                        fcn2(ptr);
                    });
                fRef = rhs.fRef;
                fIndex = rhs.fIndex;
                return *this;
            }
 
            /**
             * Pre-increment 
             *
             * @return ForwardIterator<T>& - reference to this.
             * @throw none
             */
            ForwardIterator<T>& operator++() MW_NOEXCEPT {
                ++fIndex;
                typedef void(*ForwardIteratorPlusPlusFcnPtr)(detail::ForwardIteratorImpl*);
                static const ForwardIteratorPlusPlusFcnPtr fcn = detail::resolveFunction<ForwardIteratorPlusPlusFcnPtr>
                    (detail::FunctionType::FORWARD_ITERATOR_PLUS_PLUS);
                fcn(pImpl.get());
                return *this;
            }

            /**
             * Post-increment
             *
             * @return ForwardIterator<T> - newly created ForwardIterator
             * @throw none
             */
            ForwardIterator<T> operator++(int) MW_NOEXCEPT {
                ForwardIterator<T> temp(*this);
                ++fIndex;
                typedef void(*ForwardIteratorPlusPlusFcnPtr)(detail::ForwardIteratorImpl*);
                static const ForwardIteratorPlusPlusFcnPtr fcn = detail::resolveFunction<ForwardIteratorPlusPlusFcnPtr>
                    (detail::FunctionType::FORWARD_ITERATOR_PLUS_PLUS);
                fcn(pImpl.get());
                return temp;
            }

            /**
             * operator== determine if 2 iterators are pointing to the same element
             * @param rhs - the iterator to be compared with
             * @return bool - true if the iterators are pointing to the same element
             * @throw none
             */
            bool operator==(const ForwardIterator<T> &rhs) const MW_NOEXCEPT {
                typedef bool(*ForwardIteratorEqualFcnPtr)(detail::ForwardIteratorImpl*, detail::ForwardIteratorImpl*);
                static const ForwardIteratorEqualFcnPtr fcn = detail::resolveFunction<ForwardIteratorEqualFcnPtr>
                    (detail::FunctionType::FORWARD_ITERATOR_EQUAL);
                return fcn(pImpl.get(), rhs.pImpl.get());
            }
            
            /**
             * operator!= determine if 2 iterators are pointing to the same element
             * @param rhs - the iterator to be compared with
             * @return bool - true if this iterator points to a different element
             * @throw none
             */
            bool operator!=(const ForwardIterator<T> &rhs) const MW_NOEXCEPT {
                typedef bool(*ForwardIteratorEqualFcnPtr)(detail::ForwardIteratorImpl*, detail::ForwardIteratorImpl*);
                static const ForwardIteratorEqualFcnPtr fcn = detail::resolveFunction<ForwardIteratorEqualFcnPtr>
                    (detail::FunctionType::FORWARD_ITERATOR_EQUAL);
                return !fcn(pImpl.get(), rhs.pImpl.get());
            }

            /**
             * operator* return a shared copy of the element that the iterator is pointing to
             *
             * @return T - shared copy of the element that the iterator is pointing to
             * @throw none
             */
            reference operator*() const MW_NOEXCEPT {
                detail::RefCounted* value = nullptr;
                typedef void(*ForwardIteratorGetRefFcnPtr)(detail::ForwardIteratorImpl*, detail::RefCounted**);
                static const ForwardIteratorGetRefFcnPtr fcn = detail::resolveFunction<ForwardIteratorGetRefFcnPtr>
                    (detail::FunctionType::FORWARD_ITERATOR_GET_REF);
                fcn(pImpl.get(), &value);
                return detail::Access::createObj<reference>(value, fIndex);
            }

            /**
             * Get a pointer the element pointed to by this iterator
             *
             * @return pointer - the element
             * @throw none
             */
            pointer operator->() const MW_NOEXCEPT {
                detail::RefCounted* value = nullptr;
                typedef void(*ForwardIteratorGetRefFcnPtr)(detail::ForwardIteratorImpl*, detail::RefCounted**);
                static const ForwardIteratorGetRefFcnPtr fcn = detail::resolveFunction<ForwardIteratorGetRefFcnPtr>
                    (detail::FunctionType::FORWARD_ITERATOR_GET_REF);
                fcn(pImpl.get(), &value);
                fRef = detail::Access::createObj<reference>(value);
                return &fRef;
            }
            
            /**
             * Get a reference using a linear index
             *
             * @return reference - the element
             * @throw none
             */
            reference operator[](const size_t& rhs) const MW_NOEXCEPT {
                ForwardIterator<T> temp(*this);
                temp.fIndex += rhs;
                typedef void(*ForwardIteratorIncrementFcnPtr)(detail::ForwardIteratorImpl*, size_t);
                static const ForwardIteratorIncrementFcnPtr fcn = detail::resolveFunction<ForwardIteratorIncrementFcnPtr>
                    (detail::FunctionType::FORWARD_ITERATOR_INCREMENT);
                fcn(temp.pImpl.get(), rhs);
                return *temp;
            }

          private:
            friend class detail::Access;
            
            std::shared_ptr<detail::ForwardIteratorImpl> pImpl;   
            mutable std::remove_const<reference> fRef;
           
            size_t fIndex;

            /**
             * ForwardIterator constructor. 
             * @param *impl pointer to the implementation 
             * @throw None
             */
            ForwardIterator(detail::ForwardIteratorImpl *impl) :
                pImpl(std::shared_ptr<detail::ForwardIteratorImpl>
                      (impl, 
                       [](detail::ForwardIteratorImpl* ptr) {
                          typedef void(*ForwardIteratorDestroyFcnPtr)(detail::ForwardIteratorImpl*);
                          static const ForwardIteratorDestroyFcnPtr fcn2 = detail::resolveFunction<ForwardIteratorDestroyFcnPtr>
                              (detail::FunctionType::FORWARD_ITERATOR_DESTROY);
                          fcn2(ptr);
                      })),
                fIndex(0) {}
        };
    }
}


#endif
