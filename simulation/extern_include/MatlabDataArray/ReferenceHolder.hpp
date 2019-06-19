/* Copyright 2016-2017 The MathWorks, Inc. */

#ifndef MATLAB_EXTDATA_ARRAY_REFERENCE_HOLDER_HPP
#define MATLAB_EXTDATA_ARRAY_REFERENCE_HOLDER_HPP

#include "detail/ExceptionHelpers.hpp"
#include "detail/FunctionType.hpp"

#include <memory>

namespace matlab {
    namespace data {
        namespace detail {
            class ReferenceImpl;
        }

        class ReferenceHolder {
          protected:
            std::shared_ptr<detail::ReferenceImpl> pImpl;

            ReferenceHolder(detail::ReferenceImpl* impl) MW_NOEXCEPT :
                pImpl(std::shared_ptr<detail::ReferenceImpl>
                      (impl, [](detail::ReferenceImpl* ptr) {
                          typedef void(*ReferenceDestroyFcnPtr)(detail::ReferenceImpl*);
                          static const ReferenceDestroyFcnPtr fcn2 = detail::resolveFunction<ReferenceDestroyFcnPtr>
                              (detail::FunctionType::REFERENCE_DESTROY);
                          fcn2(ptr);
                      })) {}

            ReferenceHolder(std::shared_ptr<detail::ReferenceImpl> impl) MW_NOEXCEPT :
                pImpl(impl) {}

            ReferenceHolder(ReferenceHolder&& rhs) MW_NOEXCEPT :
                pImpl(std::move(rhs.pImpl)) {}

            ReferenceHolder& operator= (ReferenceHolder&& rhs) MW_NOEXCEPT {                            
                pImpl = std::move(rhs.pImpl);
                return *this;
            }
            
            ReferenceHolder(const ReferenceHolder& rhs) MW_NOEXCEPT :
                pImpl(rhs.pImpl) {}

            ReferenceHolder& operator= (const ReferenceHolder& rhs) MW_NOEXCEPT {                            
                pImpl = rhs.pImpl;
                return *this;
            }
        };
    }
}

#endif
