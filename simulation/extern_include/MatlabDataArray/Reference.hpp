/* Copyright 2016-2017 The MathWorks, Inc. */

#ifndef MATLAB_EXTDATA_REFERENCE_HPP
#define MATLAB_EXTDATA_REFERENCE_HPP

#include "Enumeration.hpp"
#include "String.hpp"
#include "ReferenceHolder.hpp"
#include "ArrayReferenceExt.hpp"
#include "MATLABStringReferenceExt.hpp"
#include "Exception.hpp"
#include "GetArrayType.hpp"

#include "detail/publish_util.hpp"
#include "detail/FunctionType.hpp"

#include "detail/HelperFunctions.hpp"
#include "detail/ReferenceHelpers.hpp"

#include <memory>
#include <type_traits>

namespace matlab {
    namespace data {
        namespace detail {
            class Access;
        }

        template <typename T> struct GetReferenceExt { using type = ReferenceHolder; };
        template <> struct GetReferenceExt<Array> { using type = IndexableArrayRef<false>; };
        template <> struct GetReferenceExt<MATLABString> { using type = MATLABStringReferenceExt; };

        template <typename T>
        class Reference : public GetReferenceExt<T>::type {
        public:

            typedef typename std::remove_const<T>::type ref_type;

            using Parent = typename GetReferenceExt<T>::type;
            using Parent::pImpl;

            /**
             * Cast to element from the array. Makes a copy of the element from the array
             *
             * @code
             *    Reference<Array> ref = cell_arr[0];
             *    Array cpy(ref);
             * @endcode
             *
             * @return ref_type the element being referred to
             * @throw none
             */
            operator ref_type() const MW_NOEXCEPT {
                return detail::getElement<ref_type>(pImpl);
            }

            /**
             * Assign a value into an Array. The Array being indexed must be non-const.
             *
             * @code
             *    Reference<Array> ref = cell_arr[0];
             *    ref = factory.createScalar(false);
             * @endcode
             *
             * @param rhs - value to be assigned into the Array
             *
             * @return Reference<T>& - this
             * @throw none
             */
            Reference<T>& operator= (T rhs) MW_NOEXCEPT {
                static_assert(!std::is_const<T>::value, "Can't modify a Reference to const data");
                detail::setElement(pImpl.get(), std::move(rhs), static_cast<int>(GetArrayType<T>::type));
                return *this;
            }

            /**
             * Cast to std::string from the array. Makes a copy of the std::string. Only
             * valid for types that can be cast to a std::string
             *
             * @code
             *    Reference<String> ref = str_arr[0];
             *    std::string cpy(ref);
             * @endcode
             *
             * @return std::string - a string representation of the value
             *
             * @throw std::runtime_error - for a Reference<MATLABString> if the MATLABString is missing
             * @throw NonAsciiCharInInputDataException for a Reference<MATLABString> if the MATLABString contains non-ascii characters
             */
            operator std::string() const {
                static_assert(std::is_same<ref_type, Enumeration>::value ||
                    std::is_same<ref_type, MATLABString>::value,
                    "Invalid Reference to get a string");
                return detail::getString<ref_type>(pImpl.get());
            }

            /**
             * Assign a std::string into an Array. The Array being indexed must be non-const and allow strings to be assigned
             *
             * @code
             *    Reference<String> ref = str_arr[0];
             *    ref = "MyString";
             * @endcode
             *
             * @param rhs - std::string to be assigned into the Array
             *
             * @return Reference<T>& - this
             * @throw none
             */
            Reference<T>& operator=(std::string rhs) MW_NOEXCEPT {
                static_assert(!std::is_const<T>::value, "Can't modify a Reference to const data");
                static_assert(std::is_same<ref_type, Enumeration>::value ||
                              std::is_same<ref_type, MATLABString>::value,
                              "Invalid Reference to set a string");
                detail::setString<ref_type>(pImpl.get(), std::move(rhs));
                return *this;
            }

            /**
             * Assign a String value into a MATLABString Array. The Array being indexed must be non-const.
             *
             * @code
             *    Reference<MATLABString> ref = string_arr[0];
             *    ref = String(u"hello");
             * @endcode
             *
             * @param rhs - value to be assigned into the Array
             *
             * @return Reference<T>& - this
             *
             * @throw none
             */
            Reference<T>& operator= (String rhs) MW_NOEXCEPT {
                static_assert(!std::is_const<T>::value, "Can't modify a Reference to const data");
                static_assert(std::is_same<ref_type, MATLABString>::value,
                              "Invalid Reference to set a String");
                detail::setElement(pImpl.get(), std::move(rhs), static_cast<int>(ArrayType::MATLAB_STRING));
                return *this;
            }

            /**
             * Move constructor
             *
             * @param - rhs Reference value to be moved
             * @return - newly constructed Reference
             * @throw none
             */
            Reference(Reference<T>&& rhs) MW_NOEXCEPT :
                Parent(std::move(rhs)) {}

            /**
             * Move assignment
             *
             * @param - rhs Reference value to be moved
             * @return - updated Reference
             * @throw none
             */
            Reference<T>& operator= (Reference<T>&& rhs) MW_NOEXCEPT {
                Parent::operator= (std::move(rhs));
                return *this;
            }

            /**
             * Copy constructor
             *
             * @param - rhs Reference value to be copied
             * @return - newly constructed Reference
             * @throw none
             */
            Reference(const Reference<T>& rhs) MW_NOEXCEPT :
                Parent(rhs) {}

            /**
             * Copy assignment
             *
             * @param - rhs Reference value to be copied
             * @return - updated Reference
             * @throw none
             */
            Reference<T>& operator= (const Reference<T>& rhs) MW_NOEXCEPT {
                Parent::operator= (rhs);
                return *this;
            }

        protected:
            Reference(std::shared_ptr<detail::ReferenceImpl> impl) MW_NOEXCEPT :
                Parent(impl) {}

            Reference(detail::ReferenceImpl* impl) MW_NOEXCEPT :
                Parent(impl) {}

        private:
            friend class detail::Access;

            Reference() = delete;
        };

        inline bool operator ==(Reference<MATLABString> const& lhs, std::string const& rhs) MW_NOEXCEPT {
            typedef bool(*ReferenceMATLABStringEqualStringFcnPtr)(detail::ReferenceImpl*, const char*, size_t idx);
            static const ReferenceMATLABStringEqualStringFcnPtr fcn = detail::resolveFunction<ReferenceMATLABStringEqualStringFcnPtr>
                (detail::FunctionType::REFERENCE_MATLAB_STRING_EQUAL_STRING);
            return fcn(detail::Access::getImpl<detail::ReferenceImpl>(lhs), rhs.c_str(), rhs.size());
        }

        inline bool operator ==(std::string const& lhs, Reference<MATLABString> const& rhs) MW_NOEXCEPT {
            typedef bool(*ReferenceMATLABStringEqualStringFcnPtr)(detail::ReferenceImpl*, const char*, size_t idx);
            static const ReferenceMATLABStringEqualStringFcnPtr fcn = detail::resolveFunction<ReferenceMATLABStringEqualStringFcnPtr>
                (detail::FunctionType::REFERENCE_MATLAB_STRING_EQUAL_STRING);
            return fcn(detail::Access::getImpl<detail::ReferenceImpl>(rhs), lhs.c_str(), lhs.size());
        }

        inline bool operator ==(Reference<MATLABString> const& lhs, String const& rhs) MW_NOEXCEPT {
            typedef bool(*ReferenceMATLABStringEqualString16FcnPtr)(detail::ReferenceImpl*, const char16_t*, size_t idx);
            static const ReferenceMATLABStringEqualString16FcnPtr fcn = detail::resolveFunction<ReferenceMATLABStringEqualString16FcnPtr>
                (detail::FunctionType::REFERENCE_MATLAB_STRING_EQUAL_STRING16);
            return fcn(detail::Access::getImpl<detail::ReferenceImpl>(lhs), rhs.c_str(), rhs.size());
        }

        inline bool operator ==(String const& lhs, Reference<MATLABString> const& rhs) MW_NOEXCEPT {
            typedef bool(*ReferenceMATLABStringEqualString16FcnPtr)(detail::ReferenceImpl*, const char16_t*, size_t idx);
            static const ReferenceMATLABStringEqualString16FcnPtr fcn = detail::resolveFunction<ReferenceMATLABStringEqualString16FcnPtr>
                (detail::FunctionType::REFERENCE_MATLAB_STRING_EQUAL_STRING16);
            return fcn(detail::Access::getImpl<detail::ReferenceImpl>(rhs), lhs.c_str(), lhs.size());
        }

        inline bool operator ==(Reference<MATLABString> const& lhs, MATLABString const& rhs) MW_NOEXCEPT {
            if (rhs.has_value()) {
                const String& str = *rhs;
                return (lhs == str);
            }
            return false;
        }

        inline bool operator ==(MATLABString const& lhs, Reference<MATLABString> const& rhs) MW_NOEXCEPT {
            if (lhs.has_value()) {
                const String& str = *lhs;
                return (rhs == str);
            }
            return false;
        }

        inline bool operator ==(Reference<MATLABString> const& lhs, Reference<MATLABString> const& rhs) MW_NOEXCEPT {
            typedef bool(*ReferenceMATLABStringEqualReferenceFcnPtr)(detail::ReferenceImpl*, detail::ReferenceImpl*);
            static const ReferenceMATLABStringEqualReferenceFcnPtr fcn = detail::resolveFunction<ReferenceMATLABStringEqualReferenceFcnPtr>
                (detail::FunctionType::REFERENCE_MATLAB_STRING_EQUAL_REFERENCE);
            return fcn(detail::Access::getImpl<detail::ReferenceImpl>(lhs), detail::Access::getImpl<detail::ReferenceImpl>(rhs));
        }

        template<typename T1, typename T2>
        inline bool operator !=( T1 const& lhs, T2 const& rhs) MW_NOEXCEPT {
            return !(lhs == rhs);
        }

        template<typename T>
        bool operator ==(Reference<T> const& lhs, T const& rhs) MW_NOEXCEPT {
            using ref_type = typename Reference<T>::ref_type;
            return static_cast<ref_type>(lhs) == rhs;
        }

        template<typename T>
        bool operator ==(T const& lhs, Reference<T> const& rhs) MW_NOEXCEPT {
            using ref_type = typename Reference<T>::ref_type;
            return lhs == static_cast<ref_type>(rhs);
        }

        template<typename T>
        bool operator ==(Reference<T> const& lhs, Reference<T> const& rhs) MW_NOEXCEPT {
            using ref_type = typename Reference<T>::ref_type;
            return static_cast<ref_type>(lhs) == static_cast<ref_type>(rhs);
        }

        template<typename T>
        std::ostream& operator <<(std::ostream& os, Reference<T> const& rhs) MW_NOEXCEPT {
            using ref_type = typename Reference<T>::ref_type;
            return os << static_cast<ref_type>(rhs);
        }
  
        template<typename T>
        bool operator <(Reference<T> const& lhs, Reference<T> const& rhs)
        {
            using ref_type = typename Reference<T>::ref_type;
            return static_cast<ref_type>(lhs) < static_cast<ref_type>(rhs);
        }

        template<typename T>
        bool operator >(Reference<T> const& lhs, Reference<T> const& rhs)
        {
            using ref_type = typename Reference<T>::ref_type;
            return static_cast<ref_type>(lhs) > static_cast<ref_type>(rhs);
        }

        template<typename T>
        bool operator <=(Reference<T> const& lhs, Reference<T> const& rhs)
        {
            using ref_type = typename Reference<T>::ref_type;
            return static_cast<ref_type>(lhs) <= static_cast<ref_type>(rhs);
        }

        template<typename T>
        bool operator >=(Reference<T> const& lhs, Reference<T> const& rhs)
        {
            using ref_type = typename Reference<T>::ref_type;
            return static_cast<ref_type>(lhs) >= static_cast<ref_type>(rhs);
        }
    }
}

#endif

