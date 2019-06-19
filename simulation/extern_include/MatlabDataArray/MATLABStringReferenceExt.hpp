/* Copyright 2017 The MathWorks, Inc. */

#ifndef MATLAB_EXTDATA_MATLABSTRING_REF_EXT_HPP
#define MATLAB_EXTDATA_MATLABSTRING_REF_EXT_HPP

#include "String.hpp"
#include "ReferenceHolder.hpp"

namespace matlab {
    namespace data {

        /**
         * The MATLABStringReferenceExt class is a base class for all reference types that refer to MATLABString.
         * It provides APIs for MATLABString that are not supported by generic Reference class.
         */
        class MATLABStringReferenceExt : public ReferenceHolder {
          public:
            /*
             * Check if MATLABString contains a value.
             *
             * @return true if MATLABString contains value, false if it is missing
             * @throw NotEnoughIndicesProvidedException - if not enough indices were provided
             * @throw InvalidArrayIndexException - if an array index is out of range
             * @throw TooManyIndicesProvidedException if too many indices were provided
             */
            bool has_value() const {
                MATLABString opt = detail::getElement<MATLABString>(pImpl);
                return opt.has_value();
            }
            
            /*
             * Check if MATLABString contains a value.
             *
             * @return true if MATLABString contains value, false if it is missing
             *
             * @throw NotEnoughIndicesProvidedException - if not enough indices were provided
             * @throw InvalidArrayIndexException - if an array index is out of range
             * @throw TooManyIndicesProvidedException if too many indices were provided
             */
            explicit operator bool() const {
                return has_value();
            }
            
            /**
             * Cast to String
             *
             * @code
             *    String val = strArray[1][2];
             * @endcode
             *
             * @return String - the element of the array
             *
             * @throw NotEnoughIndicesProvidedException - if not enough indices were provided
             * @throw InvalidArrayIndexException - if an array index is out of range
             * @throw TooManyIndicesProvidedException if too many indices were provided
             * @throw std::runtime_error - if the MATLABString has no value
             */
            operator String() const {
                MATLABString opt = detail::getElement<MATLABString>(pImpl);
                if (!opt) {
                    throw std::runtime_error("Missing string value");
                }
                return *opt;
            }

          protected:
            friend class detail::Access;

            MATLABStringReferenceExt(detail::ReferenceImpl* impl) MW_NOEXCEPT :
                ReferenceHolder(impl) {}

            MATLABStringReferenceExt(std::shared_ptr<detail::ReferenceImpl> impl) MW_NOEXCEPT :
                ReferenceHolder(impl) {}

            MATLABStringReferenceExt(MATLABStringReferenceExt&& rhs) MW_NOEXCEPT :
                ReferenceHolder(std::move(rhs)) {}

            MATLABStringReferenceExt& operator= (MATLABStringReferenceExt&& rhs) MW_NOEXCEPT {
                ReferenceHolder::operator= (std::move(rhs));
                return *this;
            }

            MATLABStringReferenceExt(const MATLABStringReferenceExt& rhs) MW_NOEXCEPT :
                ReferenceHolder(rhs) {}

            MATLABStringReferenceExt& operator= (const MATLABStringReferenceExt& rhs) MW_NOEXCEPT {
                ReferenceHolder::operator= (rhs);
                return *this;
            }

          private:
            MATLABStringReferenceExt() = delete;
        };
    }
}

#endif
