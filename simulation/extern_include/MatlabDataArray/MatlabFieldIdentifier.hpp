/* Copyright 2014 - 2017 The MathWorks, Inc. */

#ifndef MATLAB_DATA_FIELDIDENTIFIER_HPP
#define MATLAB_DATA_FIELDIDENTIFIER_HPP

#include "detail/publish_util.hpp"
#include "detail/FunctionType.hpp"

#include <string>
#include <memory>

namespace matlab {
    namespace data {
        namespace detail {
            class RefCounted;
            class Access;
        }

        template <typename T> class ForwardIterator;

        /**
         * The MATLABFieldIdentifier is a class used to indentify field names
         */
        class MATLABFieldIdentifier {
          public:

            /**
             * Construct a MATLABFieldIdentifier
             *
             * @return newly constructed MATLABFieldIdentifer
             * @throw none
             */
            MATLABFieldIdentifier() MW_NOEXCEPT {}

            /**
             * Construct a MATLABFieldIdentifier from a std::string
             * @param str - the string that contains the field name
             * @return newly constructed MATLABFieldIdentifer
             * @throw none
             */
            MATLABFieldIdentifier(std::string str) MW_NOEXCEPT {
                typedef detail::RefCounted*(*FieldIdCreateFcnPtr)(const char*, size_t);
                static const FieldIdCreateFcnPtr fcn = detail::resolveFunction<FieldIdCreateFcnPtr>
                    (detail::FunctionType::FIELD_ID_CREATE);
                pImpl = std::shared_ptr<detail::RefCounted>
                    (fcn(str.c_str(), str.size()), [](detail::RefCounted* ptr) {
                        typedef void(*FieldIdDestroyFcnPtr)(detail::RefCounted*);
                        static const FieldIdDestroyFcnPtr fcn2 = detail::resolveFunction<FieldIdDestroyFcnPtr>
                            (detail::FunctionType::FIELD_ID_DESTROY);
                        fcn2(ptr);
                    });
            }
                      
            /**
             * Move assignment operator
             * @param rhs - the MATLABFieldIdentifier to be moved
             * @return updated MATLABFieldIdentifer
             * @throw none
             */
            MATLABFieldIdentifier& operator=(MATLABFieldIdentifier&& rhs) MW_NOEXCEPT {
                pImpl = std::move(rhs.pImpl);
                return *this;
            }

            /**
             * Move constructor
             * @param rhs - the MATLABFieldIdentifier to be moved
             * @return newly constructed MATLABFieldIdentifer
             * @throw none
             */
            MATLABFieldIdentifier(MATLABFieldIdentifier&& rhs) MW_NOEXCEPT :
                pImpl(std::move(rhs.pImpl)) {}

            /**
             * Assignment operator
             * @param rhs - the MATLABFieldIdentifier to be copied
             * @return updated MATLABFieldIdentifier which is a shared copy of the original MATLABFieldIdentifier
             * @throw none
             */
            MATLABFieldIdentifier& operator=(MATLABFieldIdentifier const& rhs) MW_NOEXCEPT {
                pImpl = rhs.pImpl;
                return *this;
            }

            /**
             * copy constructor
             * @param rhs - the MATLABFieldIdentifier to be copied
             * @return newly constructed MATLABFieldIdentifer
             * @throw none
             */
            MATLABFieldIdentifier(const MATLABFieldIdentifier &rhs) MW_NOEXCEPT :
                pImpl(rhs.pImpl) {}

            /**
             * Check to see if the 2 MATLABFieldIdentifiers are identical
             * @param rhs - MATLABFieldIdentifier to be compared
             * @return true if the MATLABFieldIdentifiers are identical
             * @throw none
             */
            bool operator==( const MATLABFieldIdentifier& rhs) const MW_NOEXCEPT {
                typedef bool(*FieldIdIsEqualFcnPtr)(detail::RefCounted*, detail::RefCounted*);
                static const FieldIdIsEqualFcnPtr fcn = detail::resolveFunction<FieldIdIsEqualFcnPtr>
                    (detail::FunctionType::FIELD_ID_IS_EQUAL);
                return fcn(pImpl.get(), rhs.pImpl.get());
            }

            /**
             * Get a std::string representation of the MATLABFieldIdentifier
             * @return std::string - std::string representation of the MATLABFieldIdentifier
             * @throw none
             */
            operator std::string() const MW_NOEXCEPT {
                const char* str = nullptr;
                size_t len = 0;
                typedef detail::RefCounted*(*FieldIdGetStringFcnPtr)(detail::RefCounted*, const char**, size_t*);
                static const FieldIdGetStringFcnPtr fcn = detail::resolveFunction<FieldIdGetStringFcnPtr>
                    (detail::FunctionType::FIELD_ID_GET_STRING);
                fcn(pImpl.get(), &str, &len);
                return std::string(str, len);
            }

          private:
            std::shared_ptr<detail::RefCounted> pImpl;

            friend class detail::Access;

            MATLABFieldIdentifier(detail::RefCounted* impl, size_t) MW_NOEXCEPT :
                pImpl(std::shared_ptr<detail::RefCounted>
                      (impl, [](detail::RefCounted* ptr) {
                          typedef void(*FieldIdDestroyFcnPtr)(detail::RefCounted*);
                          static const FieldIdDestroyFcnPtr fcn2 = detail::resolveFunction<FieldIdDestroyFcnPtr>
                              (detail::FunctionType::FIELD_ID_DESTROY);
                          fcn2(ptr);
                      })) {}
        };
    }
}
#endif
