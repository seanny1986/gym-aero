/* Copyright 2014-2017 The MathWorks, Inc. */

#ifndef MATLAB_DATA_STRUCT_HPP
#define MATLAB_DATA_STRUCT_HPP

#include "MDArray.hpp"
#include "TypedIterator.hpp"

#include "detail/ExceptionHelpers.hpp"
#include "detail/HelperFunctions.hpp"
#include "detail/FunctionType.hpp"

namespace matlab {
    namespace data {        
        namespace detail {
            class ReferenceImpl;
            class Access;
        }
        namespace impl {
            class ArrayImpl;
        }
        /**
         * Struct Element - ElementType for StructArrays. Can be used to inspect the fields of the Struct, but not
         * modify the Struct.
         */
        class Struct {
          public:
            typedef TypedIterator<Array const> const_iterator;

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
                    return detail::Access::createObj<const_iterator>(fcn(pImpl.get(), true));
                }
                else {
                    return detail::Access::createObj<const_iterator>(str_fcn(pImpl.get(), true));
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
                    return detail::Access::createObj<const_iterator>(fcn(pImpl.get(), true));
                }
                else {
                    return detail::Access::createObj<const_iterator>(str_fcn(pImpl.get(), true));
                }
            }

          private:
            Struct() = delete;

            Struct(std::shared_ptr<detail::ReferenceImpl> impl) MW_NOEXCEPT :
                pImpl(impl) {}

            std::shared_ptr<detail::ReferenceImpl> pImpl;
            friend class detail::Access;
        };
    }
}

#endif
