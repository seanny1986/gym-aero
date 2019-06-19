/* Copyright 2015 The MathWorks, Inc. */

#ifndef MATLAB_DATA_RANGE_HPP_
#define MATLAB_DATA_RANGE_HPP_

#include "detail/publish_util.hpp"

namespace matlab {
    namespace data {

        template<template<class> class IteratorType, class ElementType>
        class Range {
          public:

            /**
             * Get the begin of the Range
             *
             * @return the first element in the range
             * @throw none
             */
            IteratorType<ElementType>& begin() MW_NOEXCEPT {
                return fBegin;
            }
            
            /**
             * Get the end of the Range
             *
             * @return the end of the range
             * @throw none
             */
            IteratorType<ElementType>& end() MW_NOEXCEPT {
                return fEnd;
            }

            /**
             * Range constructor
             *
             * @param begin - begin for the Range
             * @param end - the end of the Range
             *
             * @return the newly constructed Range
             *
             * @throw none
             */
            Range(IteratorType<ElementType> begin, IteratorType<ElementType> end) MW_NOEXCEPT :
                fBegin(std::move(begin)),
                fEnd(std::move(end)) {}

            /**
             * Range move constructor
             *
             * @param rhs - Range to be moved
             *
             * @return the newly constructed Range
             *
             * @throw none
             */
            Range(Range&& rhs) MW_NOEXCEPT :
                fBegin(std::move(rhs.fBegin)),
                fEnd(std::move(rhs.fEnd)) {}

            /**
             * Range move assignment operator
             *
             * @param rhs - Range to be moved
             *
             * @return the updated Range
             *
             * @throw none
             */
            Range& operator=(Range&& rhs) MW_NOEXCEPT {
                fBegin = std::move(rhs.fBegin);
                fEnd = std::move(rhs.fEnd);
                return *this;
            }

          private:
            IteratorType<ElementType> fBegin;
            IteratorType<ElementType> fEnd;    

            Range& operator=(Range const& rhs) = delete;
            Range(const Range &rhs) = delete;
        };
    }
}

#endif
