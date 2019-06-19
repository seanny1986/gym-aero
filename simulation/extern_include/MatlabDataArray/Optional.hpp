/* Copyright 2017 The MathWorks, Inc. */

#ifndef MATLAB_DATA_OPTIONAL_HPP_
#define MATLAB_DATA_OPTIONAL_HPP_

#include "detail/publish_util.hpp"

#include <stdexcept>

namespace matlab {
    namespace data {
        typedef struct nullopt {
            explicit nullopt(int) {}
        } nullopt_t;

        /**
         *  The class template optional manages an optional contained value, i.e. a value that may or may not be present.
         */
        template<typename T>
        class optional {
          public:
            
            typedef T value_type;
            
            /**
             * Default constructor
             *
             * Set missing flag to true to indicate that the object
             * does not contain a value
             * @throw none
             */
            optional() MW_NOEXCEPT {
                fMissing = true;
            }
            
            /**
             * Default constructor
             *
             * Set missing flag to true to indicate that the object
             * does not contain a value
             * @throw None
             */
            optional(nullopt_t) MW_NOEXCEPT {
                fMissing = true;
            }
            
            /**
             * Default constructor
             *
             * Set missing flag to true to indicate that the object
             * does not contain a value
             * @throw None
             */
            optional(std::nullptr_t) MW_NOEXCEPT{
                fMissing = true;
            }
            
            /**
             * Copy constructor
             *
             * @param - rhs Reference value to be copied
             * @return - newly constructed optional object
             * @throw none
             */
            optional(const optional& other) MW_NOEXCEPT {
                fMissing = other.fMissing;
                if (!fMissing) {
                    fData = other.fData;
                }
            }
            
            /**
             * Move constructor
             *
             * @param - rhs optional value to be moved
             * @return - newly constructed optional
             * @throw none
             */
            optional(optional&& other) MW_NOEXCEPT {
                fMissing = std::move(other.fMissing);
                if (!fMissing) {
                    fData = std::move(other.fData);
                }
            }
            
            /**
             * Move constructor
             *
             * @param - rhs value of type T to be moved
             * @return - newly constructed optional
             * @throw none
             */
            optional(T&& value) MW_NOEXCEPT
                : fData(std::move(value)) {
                fMissing = false;
            }
            
            
            /**
             * Copy assignment
             *
             * @param - rhs optional value to be copied
             * @return - updated optional
             * @throw none
             */
            optional<T>& operator=(const optional<T>& other) MW_NOEXCEPT {
                if (fMissing && other.fMissing) {
                    return *this;
                }
                if (!fMissing && other.fMissing) {
                    fMissing = true;
                }
                if (!other.fMissing) {
                    fData = other.fData;
                    fMissing = false;
                }
                return *this;
            }
            
            /**
             * Move assignment
             *
             * @param - rhs optional value to be moved
             * @return - updated optional
             * @throw none
             */
            optional<T>& operator=(optional<T>&& other) MW_NOEXCEPT {
                if (fMissing && other.fMissing) {
                    return *this;
                }
                if (!fMissing && other.fMissing) {
                    fMissing = true;
                }
                if (!other.fMissing) {
                    fData = std::move(other.fData);
                    fMissing = false;
                }
                return *this;
            }
            
            /**
             * Move assignment
             *
             * @param - rhs value of type value_type to be moved
             * @return - updated optional
             * @throw none
             */
            optional<T>& operator=(T&& value)  MW_NOEXCEPT {
                fMissing = false;
                fData = value;
                return *this;
            }
            
            /**
             * Copy assignment
             *
             * @param - rhs value of type value_type to be copied
             * @return - updated optional
             * @throw none
             */
            optional<T>& operator=(const T& value) MW_NOEXCEPT {
                fMissing = false;
                fData = value;
                return *this;
            }
            
            
            /**
             * Operator->
             *
             * @return - return const value contained in optional
             * @throw - std::runtime_error if there is no value
             */
            const T* operator->() const {
                if (fMissing) {
                    throw std::runtime_error("Missing string value");
                }
                return fData;
            }
            
            /**
             * Operator->
             *
             * @return - return value contained in optional
             * @throw - std::runtime_error if there is no value
             */
            T * operator->() {
                if (fMissing) {
                    throw std::runtime_error("Missing string value");
                }
                return &fData;
            }
            
            /**
             * Operator*
             *
             * @return - return const value contained in optional
             * @throw - std::runtime_error if there is no value
             */
            const T& operator*() const {
                if (fMissing) {
                    throw std::runtime_error("Missing string value");
                }
                return const_cast<T&>(fData);
            }
            
            /**
             * Operator*
             *
             * @return - value contained in optional
             * @throw - std::runtime_error if there is no value
             */
            T& operator*() {
                if (fMissing) {
                    throw std::runtime_error("Missing string value");
                }
                return fData;
            }
            
            /**
             * Check if this instance of optional contains a value
             *
             * @return - true if contains a value, false otherwise
             * @throw none
             */
            explicit operator bool() const MW_NOEXCEPT {
                return !fMissing;
            }
            
            /**
             * Cast optional<T> value to T
             *
             * @return value contained in optional<T> if such exists
             * &throw none
             */
            operator T() const {
                if (fMissing) {
                    throw std::runtime_error("Missing string value");
                }
                return fData;
            }
            
            /**
             * Check if this instance of optional contains a value
             *
             * @return - true if contains a value, false otherwise
             * @throw none
             */
            bool has_value() const MW_NOEXCEPT {
                return !fMissing;
            }

            /**
             * Swap value of this optional instance with value
             * contained in the parameter
             *
             * @param - optional value to swap with
             * @throw none
             */
            void swap(optional &other) MW_NOEXCEPT {
                if (fMissing && other.fMissing) {
                    return;
                }
                if (!fMissing && !other) {
                    other = std::move(fData);
                    fMissing = true;
                }
                else if (fMissing && other) {
                    fData = std::move(*other);
                    other.reset();
                }  else {
                    auto tmp = fData;
                    fData = *other;
                    other = std::move(tmp);
                }
            }

            /**
             * Reset optional value to missing
             *
             * @throw none
             */
            void reset() MW_NOEXCEPT {
                fMissing = true;
            }

            ~optional() {}

          private:
            bool fMissing;
            T fData;
        };


        // non-member functions
        // To be consistent with MATLAB string behavior, missing values do not compare equal to anything

        template<class T>
        bool operator==(const optional<T>& lhs, const optional<T>& rhs)  MW_NOEXCEPT {
            if (lhs.has_value() && rhs.has_value()) {
                return (*lhs == *rhs);
            } else {
                return false;
            }
        }
    
        template< class T >
        bool operator==(const optional<T>& opt, nullopt_t) MW_NOEXCEPT {
            return false;
        }
    
        template<class T>
        bool operator!=(const optional<T>& lhs, const optional<T>& rhs)  MW_NOEXCEPT {
            return (!(lhs == rhs));
        }
    
        template<class T>
        bool operator==(const optional<T>& opt, const T& value) MW_NOEXCEPT {
            if (!opt) {
                return false;
            }
            
            return (*opt == value);
        }

        template< class T >
        bool operator!=(const optional<T>& opt, nullopt_t) MW_NOEXCEPT {
            return true;
        }

        template<class T>
        bool operator!=(const optional<T>& opt, const T& value) MW_NOEXCEPT {
            return (!(opt == value));
        }
    }  // data
} // matlab

#endif
