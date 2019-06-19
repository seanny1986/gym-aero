/* Copyright 2018 The MathWorks, Inc. */

#ifndef MATLAB_DATA_TYPED_ITERATOR_VS2013_HPP
#define MATLAB_DATA_TYPED_ITERATOR_VS2013_HPP

namespace matlab {
    namespace data {
        namespace detail {
            class IteratorImpl;
            class ReferenceImpl;
            class Access;
        }

        template <typename T> class Reference;

        template <typename T> struct GetConstRefType    { using type = T; };
        template <> struct GetConstRefType<Enumeration> { using type = std::string; };

        template<typename T>
        class TypedIterator {
          public:
            
            typedef typename std::remove_const<T>::type impl_type;
            typedef typename GetConstRefType<impl_type>::type const_reference;
            typedef typename std::conditional<std::is_const<T>::value, const_reference, Reference<impl_type>>::type ref_type;
            typedef typename std::conditional<std::is_arithmetic<T>::value, T&, ref_type>::type reference;
            typedef typename std::conditional<std::is_arithmetic<T>::value, T*, ref_type*>::type pointer;

            typedef std::random_access_iterator_tag iterator_category;
            typedef T value_type;
            typedef std::ptrdiff_t difference_type;
            
            /**
             * TypedIterator copy constructor 
             * @param rhs TypedIterator object to assign to this instance.
             * @throw none
             */
            TypedIterator(const TypedIterator<T> &rhs) MW_NOEXCEPT :
                fRef(createTempRef<impl_type>()) {
                typedef detail::IteratorImpl*(*TypedIteratorCloneFcnPtr)(detail::IteratorImpl*);
                static const TypedIteratorCloneFcnPtr fcn = detail::resolveFunction<TypedIteratorCloneFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_CLONE);
                pImpl = std::shared_ptr<detail::IteratorImpl>
                    (fcn(rhs.pImpl.get()), 
                     [](detail::IteratorImpl* ptr) {
                        typedef void(*TypedIteratorDestroyFcnPtr)(detail::IteratorImpl*);
                        static const TypedIteratorDestroyFcnPtr fcn2 = detail::resolveFunction<TypedIteratorDestroyFcnPtr>
                            (detail::FunctionType::TYPED_ITERATOR_DESTROY);
                        fcn2(ptr);
                    });
            }
            
            /**
             * TypedIterator operator=. 
             * @param rhs TypedIterator object to assign to this instance.
             * @throw none
             */
            TypedIterator<T>& operator=(const TypedIterator<T> &rhs) MW_NOEXCEPT {
                typedef detail::IteratorImpl*(*TypedIteratorCloneFcnPtr)(detail::IteratorImpl*);
                static const TypedIteratorCloneFcnPtr fcn = detail::resolveFunction<TypedIteratorCloneFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_CLONE);
                pImpl = std::shared_ptr<detail::IteratorImpl>
                    (fcn(rhs.pImpl.get()), 
                     [](detail::IteratorImpl* ptr) {
                        typedef void(*TypedIteratorDestroyFcnPtr)(detail::IteratorImpl*);
                        static const TypedIteratorDestroyFcnPtr fcn2 = detail::resolveFunction<TypedIteratorDestroyFcnPtr>
                            (detail::FunctionType::TYPED_ITERATOR_DESTROY);
                        fcn2(ptr);
                    });
                fRef = rhs.fRef;
                return *this;
            }

            /**
             * TypedIterator move constructor 
             * @param rhs TypedIterator object to assign to this instance.
             * @throw none
             */
            TypedIterator(TypedIterator<T> &&rhs) MW_NOEXCEPT :
                pImpl(std::move(rhs.pImpl)),
                fRef(std::move(rhs.fRef)) {}
            
            /**
             * TypedIterator move assignment 
             * @param rhs TypedIterator object to assign to this instance.
             * @throw none
             */
            TypedIterator<T>& operator=(TypedIterator<T> &&rhs) MW_NOEXCEPT {
                pImpl = std::move(rhs.pImpl);
                fRef = std::move(rhs.fRef);
                return *this;
            }

            /**
             * pre-increment
             *
             * @return this
             * @throw none
             */
            TypedIterator<T>& operator++() MW_NOEXCEPT {
                typedef void(*TypedIteratorPlusPlusFcnPtr)(detail::IteratorImpl*);
                static const TypedIteratorPlusPlusFcnPtr fcn = detail::resolveFunction<TypedIteratorPlusPlusFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_PLUS_PLUS);
                fcn(pImpl.get());
                return *this;
            }

            /**
             * pre-decrement
             *
             * @return this
             * @throw none
             */
            TypedIterator<T>& operator--() MW_NOEXCEPT {
                typedef void(*TypedIteratorMinusMinusFcnPtr)(detail::IteratorImpl*);
                static const TypedIteratorMinusMinusFcnPtr fcn = detail::resolveFunction<TypedIteratorMinusMinusFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_MINUS_MINUS);
                fcn(pImpl.get());
                return *this;
            }

            /**
             * post-increment
             *
             * @return copy of original iterator
             * @throw none
             */
            TypedIterator<T> operator++(int) MW_NOEXCEPT {
                TypedIterator<T> temp(*this);
                typedef void(*TypedIteratorPlusPlusFcnPtr)(detail::IteratorImpl*);
                static const TypedIteratorPlusPlusFcnPtr fcn = detail::resolveFunction<TypedIteratorPlusPlusFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_PLUS_PLUS);
                fcn(pImpl.get());
                return temp;
            }

            /**
             * post-decrement
             *
             * @return copy of original iterator
             * @throw none
             */
            TypedIterator<T> operator--(int) MW_NOEXCEPT {
                TypedIterator<T> temp(*this);
                typedef void(*TypedIteratorMinusMinusFcnPtr)(detail::IteratorImpl*);
                static const TypedIteratorMinusMinusFcnPtr fcn = detail::resolveFunction<TypedIteratorMinusMinusFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_MINUS_MINUS);
                fcn(pImpl.get());
                return temp;
            }

            /**
             * Addition assignement
             *
             * @param d - amount to be added
             * @return this;
             * @throw none
             */
            TypedIterator<T>& operator+=(difference_type d) MW_NOEXCEPT {
                typedef void(*TypedIteratorIncrementFcnPtr)(detail::IteratorImpl*, difference_type);
                static const TypedIteratorIncrementFcnPtr fcn = detail::resolveFunction<TypedIteratorIncrementFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_INCREMENT);
                fcn(pImpl.get(), d);
                return *this;
            }
            
            /**
             * Subtraction assignement
             *
             * @param d - amount to be subtracted
             * @return this;
             * @throw none
             */
            TypedIterator<T>& operator-=(difference_type d) MW_NOEXCEPT {
                typedef void(*TypedIteratorDecrementFcnPtr)(detail::IteratorImpl*, ptrdiff_t);
                static const TypedIteratorDecrementFcnPtr fcn = detail::resolveFunction<TypedIteratorDecrementFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_DECREMENT);
                fcn(pImpl.get(), d);
                return *this;
            }
            
            /**
             * operator== 
             *
             * @param rhs - other iterator
             * @return true if both iterators are pointing to the same element
             * @throw none
             */
            bool operator==(const TypedIterator<T> &rhs) const MW_NOEXCEPT {
                typedef bool(*TypedIteratorEqualFcnPtr)(detail::IteratorImpl*, detail::IteratorImpl*);
                static const TypedIteratorEqualFcnPtr fcn = detail::resolveFunction<TypedIteratorEqualFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_EQUAL);
                return fcn(pImpl.get(), rhs.pImpl.get());
            }
            
            /**
             * operator!= 
             *
             * @param rhs - other iterator
             * @return true if both iterators are not pointing to the same element
             * @throw none
             */
            bool operator!=(const TypedIterator<T> &rhs) const MW_NOEXCEPT {
                typedef bool(*TypedIteratorEqualFcnPtr)(detail::IteratorImpl*, detail::IteratorImpl*);
                static const TypedIteratorEqualFcnPtr fcn = detail::resolveFunction<TypedIteratorEqualFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_EQUAL);
                return !fcn(pImpl.get(), rhs.pImpl.get());
            }

            /**
             * operator< 
             *
             * @param rhs - other iterator
             * @return true if lhs < rhs
             * @throw none
             */
            bool operator<(const TypedIterator<T> &rhs) const MW_NOEXCEPT {
                typedef difference_type(*TypedIteratorDistanceToFcnPtr)(detail::IteratorImpl*, detail::IteratorImpl*);
                static const TypedIteratorDistanceToFcnPtr fcn = detail::resolveFunction<TypedIteratorDistanceToFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_DISTANCE_TO);
                return (fcn(pImpl.get(), rhs.pImpl.get()) > 0);
            }
            
            /**
             * operator>
             *
             * @param rhs - other iterator
             * @return true if lhs > rhs
             * @throw none
             */
            bool operator>(const TypedIterator<T> &rhs) const MW_NOEXCEPT {
                typedef difference_type(*TypedIteratorDistanceToFcnPtr)(detail::IteratorImpl*, detail::IteratorImpl*);
                static const TypedIteratorDistanceToFcnPtr fcn = detail::resolveFunction<TypedIteratorDistanceToFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_DISTANCE_TO);
                return (fcn(pImpl.get(), rhs.pImpl.get()) < 0);
            }
            
            /**
             * operator<=
             *
             * @param rhs - other iterator
             * @return true if lhs <= rhs
             * @throw none
             */
            bool operator<=(const TypedIterator<T> &rhs) const MW_NOEXCEPT {
                typedef difference_type(*TypedIteratorDistanceToFcnPtr)(detail::IteratorImpl*, detail::IteratorImpl*);
                static const TypedIteratorDistanceToFcnPtr fcn = detail::resolveFunction<TypedIteratorDistanceToFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_DISTANCE_TO);
                return (fcn(pImpl.get(), rhs.pImpl.get()) >= 0);
            }
            
            /**
             * operator>=
             *
             * @param rhs - other iterator
             * @return true if lhs >= rhs
             * @throw none
             */
            bool operator>=(const TypedIterator<T> &rhs) const MW_NOEXCEPT {
                typedef difference_type(*TypedIteratorDistanceToFcnPtr)(detail::IteratorImpl*, detail::IteratorImpl*);
                static const TypedIteratorDistanceToFcnPtr fcn = detail::resolveFunction<TypedIteratorDistanceToFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_DISTANCE_TO);
                return (fcn(pImpl.get(), rhs.pImpl.get()) <= 0);
            }

            /**
             * operator+ - creates a new iterator that is added to this one by the amount passed in
             *
             * @param d - the amount to be added
             * @return newly created TypedIterator
             * @throw none
             */
            TypedIterator<T> operator+(difference_type d) const MW_NOEXCEPT {
                TypedIterator<T> temp(*this);
                typedef void(*TypedIteratorIncrementFcnPtr)(detail::IteratorImpl*, difference_type);
                static const TypedIteratorIncrementFcnPtr fcn = detail::resolveFunction<TypedIteratorIncrementFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_INCREMENT);
                fcn(temp.pImpl.get(), d);
                return temp;
            }

            /**
             * operator- - creates a new iterator that is decremented from this one by the amount passed in
             *
             * @param d - the amount to be decremented
             * @return newly created TypedIterator
             * @throw none
             */
            TypedIterator<T> operator-(difference_type d) const MW_NOEXCEPT {
                TypedIterator<T> temp(*this);
                typedef void(*TypedIteratorDecrementFcnPtr)(detail::IteratorImpl*, ptrdiff_t);
                static const TypedIteratorDecrementFcnPtr fcn = detail::resolveFunction<TypedIteratorDecrementFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_DECREMENT);
                fcn(temp.pImpl.get(), d);
                return temp;
            }

            /**
             * operator- - find the difference between 2 iterators
             *
             * @param rhs - other iterator
             * @return difference_type - the difference between the iterators
             * @throw none
             */
            difference_type operator-(const TypedIterator<T> &rhs) const MW_NOEXCEPT {
                typedef difference_type(*TypedIteratorDistanceToFcnPtr)(detail::IteratorImpl*, detail::IteratorImpl*);
                static const TypedIteratorDistanceToFcnPtr fcn = detail::resolveFunction<TypedIteratorDistanceToFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_DISTANCE_TO);
                return(fcn(rhs.pImpl.get(), pImpl.get()));
            }

            /**
             * Get a reference the element pointed to by this iterator
             *
             * @return reference - the element
             * @throw none
             */
            reference operator*() const MW_NOEXCEPT {
                return getValueOrRef<T>();
            }

            /**
             * Get a pointer the element pointed to by this iterator
             *
             * @return pointer - the element
             * @throw none
             */
            pointer operator->() const MW_NOEXCEPT {
                return getPointerOrRef<T>();
            }

            /**
             * Get a reference using a linear index
             *
             * @return reference - the element
             * @throw none
             */
            reference operator[](const size_t& rhs) const MW_NOEXCEPT {
                TypedIterator<T> temp(*this);
                typedef void(*TypedIteratorIncrementFcnPtr)(detail::IteratorImpl*, ptrdiff_t);
                static const TypedIteratorIncrementFcnPtr fcn = detail::resolveFunction<TypedIteratorIncrementFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_INCREMENT);
                fcn(temp.pImpl.get(), rhs);
                return *temp;
            }

          private:
            typedef typename std::conditional<std::is_arithmetic<T>::value, void*, ref_type>::type TempRef;
            friend class detail::Access;

            std::shared_ptr<detail::IteratorImpl> pImpl;
            mutable TempRef fRef;
            
            TypedIterator(detail::IteratorImpl *impl) MW_NOEXCEPT :
                pImpl(std::shared_ptr<detail::IteratorImpl>
                      (impl, 
                       [](detail::IteratorImpl* ptr) {
                          typedef void(*TypedIteratorDestroyFcnPtr)(detail::IteratorImpl*);
                          static const TypedIteratorDestroyFcnPtr fcn2 = detail::resolveFunction<TypedIteratorDestroyFcnPtr>
                              (detail::FunctionType::TYPED_ITERATOR_DESTROY);
                          fcn2(ptr);
                      })),
                fRef(createTempRef<impl_type>()) {}

            template <typename U>
            typename std::enable_if<std::is_arithmetic<U>::value, reference>::type getValueOrRef() const MW_NOEXCEPT {
                void* value = nullptr;
                typedef void(*TypedIteratorGetPodValueFcnPtr)(detail::IteratorImpl*, void**);
                static const TypedIteratorGetPodValueFcnPtr fcn = detail::resolveFunction<TypedIteratorGetPodValueFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_GET_POD_VALUE);
                fcn(pImpl.get(), &value);
                return *(static_cast<impl_type*>(value));
            }            

            template <typename U>
            typename std::enable_if<!std::is_arithmetic<U>::value, reference>::type getValueOrRef() const MW_NOEXCEPT {
                detail::ReferenceImpl* value = nullptr;
                typedef void(*TypedIteratorGetProxyFcnPtr)(detail::IteratorImpl*, detail::ReferenceImpl**);
                static const TypedIteratorGetProxyFcnPtr fcn = detail::resolveFunction<TypedIteratorGetProxyFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_GET_PROXY);
                fcn(pImpl.get(), &value);
                auto ref = detail::Access::createObj<Reference<impl_type>>(value);
                return ref;
            }

            template <typename U>
            typename std::enable_if<std::is_arithmetic<U>::value, pointer>::type getPointerOrRef() const MW_NOEXCEPT {
                void* value = nullptr;
                typedef void(*TypedIteratorGetPodValueFcnPtr)(detail::IteratorImpl*, void**);
                static const TypedIteratorGetPodValueFcnPtr fcn = detail::resolveFunction<TypedIteratorGetPodValueFcnPtr>
                    (detail::FunctionType::TYPED_ITERATOR_GET_POD_VALUE);
                fcn(pImpl.get(), &value);
                return (static_cast<impl_type*>(value));
            }            

            template <typename U>
            typename std::enable_if<!std::is_arithmetic<U>::value, pointer>::type getPointerOrRef() const MW_NOEXCEPT {
                fRef = getValueOrRef<U>();
                return &fRef;
            }

            template <typename U>
            typename std::enable_if<std::is_arithmetic<U>::value, void*>::type createTempRef() const MW_NOEXCEPT {
                return nullptr;
            }            

            template <typename U>
            typename std::enable_if<!std::is_arithmetic<U>::value && !std::is_const<T>::value, Reference<U>>::type createTempRef() const MW_NOEXCEPT {
                detail::ReferenceImpl* value = nullptr;
                return detail::Access::createObj<Reference<U>>(value);
            }
            
            template <typename U>
            typename std::enable_if<!std::is_arithmetic<U>::value && !std::is_same<Struct, U>::value && std::is_const<T>::value, ref_type>::type createTempRef() const MW_NOEXCEPT {
                return ref_type();
            }
            
            template <typename U>
            typename std::enable_if<!std::is_arithmetic<U>::value && std::is_same<Struct, U>::value && std::is_const<T>::value, ref_type>::type createTempRef() const MW_NOEXCEPT {
                return detail::Access::createObj<U>(std::shared_ptr<detail::ReferenceImpl>());
            }
        };
    }
}

#endif
