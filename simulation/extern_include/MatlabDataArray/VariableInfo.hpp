/* Copyright 2017 The MathWorks, Inc. */

#ifndef MATLAB_DATA_VARIABLE_INFO_HPP_
#define MATLAB_DATA_VARIABLE_INFO_HPP_

#include "SymbolStatus.hpp"
#include "MDArray.hpp"

#include "detail/publish_util.hpp"
#include "detail/FunctionType.hpp"
#include "detail/HelperFunctions.hpp"

#include <cassert>
#include <memory>

namespace matlab {
    namespace data {
        namespace impl {
            class ArrayImpl;
        }
        namespace detail {
            class Access;
            class RefCounted;
        }

        class VariableInfo {
          public:
            
            /**
             * get the name of this variable
             *
             * @return the name of the variable
             * @throw none
             */
            const char* getName() const MW_NOEXCEPT {
                typedef const char*(*VariableInfoGetNameFcnPtr)(detail::RefCounted*, size_t);
                static const VariableInfoGetNameFcnPtr fcn = detail::resolveFunction<VariableInfoGetNameFcnPtr>
                    (detail::FunctionType::VARIABLE_INFO_GET_NAME);
                return fcn(pImpl.get(), fIndex);
            }

            /**
             * get the value of this variable
             *
             * @return the value of the variable
             * @throw none
             */
            operator Array() const MW_NOEXCEPT {
                typedef impl::ArrayImpl*(*VariableInfoGetValueFcnPtr)(detail::RefCounted*, size_t);
                static const VariableInfoGetValueFcnPtr fcn = detail::resolveFunction<VariableInfoGetValueFcnPtr>
                    (detail::FunctionType::VARIABLE_INFO_GET_VALUE);
                return detail::Access::createObj<Array>(fcn(pImpl.get(), fIndex));
            }

            /**
             * copy constructor
             * @throw none
             */
            VariableInfo(const VariableInfo& rhs) MW_NOEXCEPT :
                pImpl(rhs.pImpl),
                fIndex(rhs.fIndex) {}

            /**
             * move constructor
             * @throw none
             */
            VariableInfo(VariableInfo&& rhs) MW_NOEXCEPT :
                pImpl(std::move(rhs.pImpl)),
                fIndex(std::move(rhs.fIndex)) {}

            /**
             * copy assignment
             * @throw none
             */
            VariableInfo& operator=(const VariableInfo& rhs) MW_NOEXCEPT {
                pImpl = rhs.pImpl;
                fIndex = rhs.fIndex;
                return *this;
            }

            /**
             * move assignment
             * @throw none
             */
            VariableInfo& operator=(VariableInfo&& rhs) MW_NOEXCEPT {
                pImpl = std::move(rhs.pImpl);
                fIndex = rhs.fIndex;
                return *this;
            }
            
            
          private:
            friend class detail::Access;
            std::shared_ptr<detail::RefCounted> pImpl;
            size_t fIndex;
            
            VariableInfo(detail::RefCounted* impl, size_t index) MW_NOEXCEPT :
                pImpl(std::shared_ptr<detail::RefCounted>
                      (impl, [](detail::RefCounted* ptr) {
                          typedef void(*VariableInfoDestroyFcnPtr)(detail::RefCounted*);
                          static const VariableInfoDestroyFcnPtr fcn2 = detail::resolveFunction<VariableInfoDestroyFcnPtr>
                              (detail::FunctionType::VARIABLE_INFO_DESTROY);
                          fcn2(ptr);
                      })),
                fIndex(index) {}
            
            VariableInfo() = delete;
        };
        
        class SymbolInfo {
          public:

            /**
             * get the name of this symbol
             *
             * @return the name of the symbol
             * @throw none
             */
            const char* getName() const MW_NOEXCEPT {
                typedef const char*(*SymbolInfoGetNameFcnPtr)(detail::RefCounted*, size_t);
                static const SymbolInfoGetNameFcnPtr fcn = detail::resolveFunction<SymbolInfoGetNameFcnPtr>
                    (detail::FunctionType::SYMBOL_INFO_GET_NAME);
                return fcn(pImpl.get(), fIndex);
            }

            /**
             * Assign a status for this symbol. If the symbol represents a variable and the value can
             * be returned, assign a value for the symbol. If the symbol represents a variable and no 
             * value is assigned, the getVariable() method on the CustomWorkspace will be called to
             * retrieve the value when it is needed.
             *
             * @param status - the status for this name
             * @return SymbolInfo&
             * @throw none
             */
            SymbolInfo& operator=( SymbolStatus status ) MW_NOEXCEPT {
                typedef const void(*SymbolInfoSetStatusFcnPtr)(detail::RefCounted*, size_t, int);
                static const SymbolInfoSetStatusFcnPtr fcn = detail::resolveFunction<SymbolInfoSetStatusFcnPtr>
                    (detail::FunctionType::SYMBOL_INFO_SET_STATUS);
                fcn(pImpl.get(), fIndex, static_cast<int>(status));
                return *this;
            }
            
            /**
             * Set the value for a variable. It is not necessary to also assign a status if a 
             * value is being returned.
             *
             * @param value - Array containing the variable value
             * @return SymbolInfo&
             * @throw none
             */
            SymbolInfo& operator=( Array val ) MW_NOEXCEPT {
                typedef const void(*SymbolInfoSetValueFcnPtr)(detail::RefCounted*, size_t, impl::ArrayImpl*);
                static const SymbolInfoSetValueFcnPtr fcn = detail::resolveFunction<SymbolInfoSetValueFcnPtr>
                    (detail::FunctionType::SYMBOL_INFO_SET_VALUE);
                fcn(pImpl.get(), fIndex, detail::Access::getImpl<impl::ArrayImpl>(val));
                return *this;
            }

            /**
             * copy constructor
             * @throw none
             */
            SymbolInfo(const SymbolInfo& rhs) MW_NOEXCEPT :
                pImpl(rhs.pImpl),
                fIndex(rhs.fIndex) {}

            /**
             * move constructor
             * @throw none
             */
            SymbolInfo(SymbolInfo&& rhs) MW_NOEXCEPT :
                pImpl(std::move(rhs.pImpl)),
                fIndex(std::move(rhs.fIndex)) {}

            /**
             * copy assignment
             * @throw none
             */
            SymbolInfo& operator=(const SymbolInfo& rhs) MW_NOEXCEPT {
                pImpl = rhs.pImpl;
                fIndex = rhs.fIndex;
                return *this;
            }

            /**
             * move assignment
             * @throw none
             */
            SymbolInfo& operator=(SymbolInfo&& rhs) MW_NOEXCEPT {
                pImpl = std::move(rhs.pImpl);
                fIndex = rhs.fIndex;
                return *this;
            }
            

          private:
            friend class detail::Access;
            std::shared_ptr<detail::RefCounted> pImpl;
            size_t fIndex;
            
            SymbolInfo(detail::RefCounted* impl, size_t index) MW_NOEXCEPT :
                pImpl(std::shared_ptr<detail::RefCounted>
                      (impl, [](detail::RefCounted* ptr) {
                          typedef void(*SymbolInfoDestroyFcnPtr)(detail::RefCounted*);
                          static const SymbolInfoDestroyFcnPtr fcn2 = detail::resolveFunction<SymbolInfoDestroyFcnPtr>
                              (detail::FunctionType::SYMBOL_INFO_DESTROY);
                          fcn2(ptr);
                      })),
                fIndex(index) {}
            
            SymbolInfo() = delete;
        };
    }
}

#endif
