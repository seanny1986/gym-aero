/* Copyright 2014-2017 The MathWorks, Inc. */

#ifndef MATLAB_DATA_OBJECT_HPP
#define MATLAB_DATA_OBJECT_HPP

#include "detail/object_interface.hpp"

#include <memory>

namespace matlab {
    namespace data {
        namespace impl {
            class ObjectImpl;
        }
        namespace detail {
            class Access;
        }
        
        /**
         * Object - the element type for ObjectArray
         */
        class Object {
          public:

            /**
             * Object destructor
             * @throw none
             */
            virtual ~Object() MW_NOEXCEPT {}

            /**
             * Object move constructor.
             * @param rhs Object object to move from
             * @return newly constructed object
             * @throw none
             */
            Object(Object&& rhs) MW_NOEXCEPT :
                pImpl(std::move(rhs.pImpl)) {}

            /**
             * Object move assignement.
             * @param rhs Object object to move from
             * @return updated Object
             * @throw none
             */
            Object& operator=(Object&& rhs) MW_NOEXCEPT {
                pImpl = std::move(rhs.pImpl);
                return *this;
            }

            /**
             * Object copy construct.
             * @param rhs Object object to be copied
             * @return newly constructed Object
             * @throw none
             */
            Object(const Object &rhs) MW_NOEXCEPT :
                pImpl(rhs.pImpl) {}

            /**
             * Object copy assignement.
             * @param rhs Object object to be copied
             * @return updated Object
             * @throw none
             */
            Object& operator=(Object const& rhs) MW_NOEXCEPT {
                pImpl = rhs.pImpl;
                return *this;
            }

          protected:
            friend class detail::Access;

            std::shared_ptr<matlab::data::impl::ObjectImpl> pImpl;

            Object() MW_NOEXCEPT {}

            Object(matlab::data::impl::ObjectImpl* impl) MW_NOEXCEPT :
                pImpl(std::shared_ptr<matlab::data::impl::ObjectImpl>
                      (impl, [](matlab::data::impl::ObjectImpl* ptr) {
                          object_destroy_impl(ptr);
                      })) {}

            Object(std::shared_ptr<matlab::data::impl::ObjectImpl> impl) MW_NOEXCEPT :
                pImpl(impl) {}

        };
    }
}

#endif
