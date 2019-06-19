/* Copyright 2015-2017 The MathWorks, Inc. */

#ifndef MATLAB_EXTDATA_STRUCT_REF_HPP
#define MATLAB_EXTDATA_STRUCT_REF_HPP

#include "TypedArrayRef.hpp"
#include "Struct.hpp"
#include "MDArray.hpp"
#include "Exception.hpp"

#include "detail/publish_util.hpp"
#include "detail/FunctionType.hpp"

#include "detail/HelperFunctions.hpp"
#include "detail/ReferenceHelpers.hpp"

namespace matlab {
    namespace data {
        namespace detail {
            class ReferenceImpl;
            class Access;
        }

        
        template<>
        class Reference<Struct> : public ReferenceHolder {
          public:
            
            typedef TypedIterator<Array> iterator;
            typedef TypedIterator<Array> const_iterator;
            /**
             * operator[] index into the Struct with a field name
             *
             * @param idx - the field name
             * @return Reference<Array> - reference to the field value
             * @throw InvalidFieldNameException if the field does not exist in the struct
             *
             */
            Reference<Array> operator[](std::string idx) {
                typedef int(*StructReferenceGetRefFcnPtr)(detail::ReferenceImpl*,
                                                          const char*,
                                                          size_t,
                                                          bool,
                                                          detail::ReferenceImpl**);
                static const StructReferenceGetRefFcnPtr fcn = detail::resolveFunctionNoExcept<StructReferenceGetRefFcnPtr>
                    (detail::FunctionType::STRUCT_REFERENCE_GET_REF);
                if (fcn == nullptr) {
                    return detail::Access::createObj<Reference<Array>>(detail::getRef(pImpl.get(),
                                                                                      idx.c_str(),
                                                                                      idx.size(),
                                                                                      true));
                }
                detail::ReferenceImpl* impl = nullptr;
                detail::throwIfError(fcn(pImpl.get(), idx.c_str(), idx.size(), true, &impl));
                return detail::Access::createObj<Reference<Array>>(impl);
            }

            /**
             * return a shared copy of the Array found at the field specified
             *
             * @param idx - field name
             * @return Array - shared copy of the Array found at the field specified
             * @throw InvalidFieldNameException if the field does not exist in the struct
             */
            Array operator[](std::string idx) const {
                impl::ArrayImpl* arrayImpl = nullptr;
                typedef int(*StructGetFieldCopyFcnPtr)(detail::ReferenceImpl*,
                                                       const char*,
                                                       size_t,
                                                       impl::ArrayImpl**);
                static const StructGetFieldCopyFcnPtr fcn = detail::resolveFunction<StructGetFieldCopyFcnPtr>
                    (detail::FunctionType::STRUCT_GET_FIELD_COPY);
                detail::throwIfError(fcn(pImpl.get(), idx.c_str(), idx.size(), &arrayImpl));
                return detail::Access::createObj<Array>(arrayImpl);
            }
            
            /**
             * Return an iterator to the beginning of the list of fields
             *
             * @return iterator
             * @throw none
             */
            iterator begin() MW_NOEXCEPT {
                typedef detail::IteratorImpl*(*ArrayReferenceBeginFcnPtr)(detail::ReferenceImpl*, bool);
                static const ArrayReferenceBeginFcnPtr str_fcn = detail::resolveFunctionNoExcept<ArrayReferenceBeginFcnPtr>
                    (detail::FunctionType::STRUCT_REFERENCE_BEGIN);
                if (str_fcn == nullptr) {
                    static const ArrayReferenceBeginFcnPtr fcn = detail::resolveFunction<ArrayReferenceBeginFcnPtr>
                        (detail::FunctionType::ARRAY_REFERENCE_BEGIN);
                    return detail::Access::createObj<iterator>(fcn(pImpl.get(), true));
                }
                else {
                    return detail::Access::createObj<iterator>(str_fcn(pImpl.get(), true));
                }
            }

            /**
             * Return an iterator to the end of the list of fields
             *
             * @return iterator
             * @throw none
             */
            iterator end() MW_NOEXCEPT {
                typedef detail::IteratorImpl*(*ArrayReferenceEndFcnPtr)(detail::ReferenceImpl*, bool);
                static const ArrayReferenceEndFcnPtr str_fcn = detail::resolveFunctionNoExcept<ArrayReferenceEndFcnPtr>
                    (detail::FunctionType::STRUCT_REFERENCE_END);
                if (str_fcn == nullptr) {
                    static const ArrayReferenceEndFcnPtr fcn = detail::resolveFunction<ArrayReferenceEndFcnPtr>
                        (detail::FunctionType::ARRAY_REFERENCE_END);
                    return detail::Access::createObj<iterator>(fcn(pImpl.get(), true));
                }
                else {
                    return detail::Access::createObj<iterator>(str_fcn(pImpl.get(), true));
                }
            }

            /**
             * Return a const_iterator to the beginning of the list of fields
             *
             * @return const_iterator
             * @throw none
             */
            const_iterator begin() const MW_NOEXCEPT {
                return cbegin();
            }

            /**
             * Return a const_iterator to the beginning of the list of fields
             *
             * @return const_iterator
             * @throw none
             */
            const_iterator cbegin() const MW_NOEXCEPT {
                typedef detail::IteratorImpl*(*ArrayReferenceBeginFcnPtr)(detail::ReferenceImpl*, bool);
                static const ArrayReferenceBeginFcnPtr str_fcn = detail::resolveFunctionNoExcept<ArrayReferenceBeginFcnPtr>
                    (detail::FunctionType::STRUCT_REFERENCE_BEGIN);
                if (str_fcn == nullptr) {
                    static const ArrayReferenceBeginFcnPtr fcn = detail::resolveFunction<ArrayReferenceBeginFcnPtr>
                        (detail::FunctionType::ARRAY_REFERENCE_BEGIN);
                    return detail::Access::createObj<iterator>(fcn(pImpl.get(), false));
                }
                else {
                    return detail::Access::createObj<iterator>(str_fcn(pImpl.get(), false));
                }
            }

            /**
             * Return a const_iterator to the end of the list of fields
             *
             * @return const_iterator
             * @throw none
             */
            const_iterator end() const MW_NOEXCEPT {
                return cend();
            }

            /**
             * Return a const_iterator to the end of the list of fields
             *
             * @return const_iterator
             * @throw none
             */
            const_iterator cend() const MW_NOEXCEPT {
                typedef detail::IteratorImpl*(*ArrayReferenceEndFcnPtr)(detail::ReferenceImpl*, bool);
                static const ArrayReferenceEndFcnPtr str_fcn = detail::resolveFunctionNoExcept<ArrayReferenceEndFcnPtr>
                    (detail::FunctionType::STRUCT_REFERENCE_END);
                if (str_fcn == nullptr) {
                    static const ArrayReferenceEndFcnPtr fcn = detail::resolveFunction<ArrayReferenceEndFcnPtr>
                        (detail::FunctionType::ARRAY_REFERENCE_END);
                    return detail::Access::createObj<iterator>(fcn(pImpl.get(), false));
                }
                else {
                    return detail::Access::createObj<iterator>(str_fcn(pImpl.get(), false));
                }
            }

            /**
             * cast to a Struct
             *
             * @return Struct
             * @throw none
             */
            operator Struct() const MW_NOEXCEPT {
                return detail::Access::createObj<Struct>(pImpl);
            }
            
          private:

            friend class detail::Access;

            Reference(detail::ReferenceImpl* impl) MW_NOEXCEPT :
                ReferenceHolder(impl) {}

            Reference() = delete;

            Reference(std::shared_ptr<detail::ReferenceImpl> impl) MW_NOEXCEPT :
                ReferenceHolder(std::move(impl)) {}

        };
    }
}

#endif
