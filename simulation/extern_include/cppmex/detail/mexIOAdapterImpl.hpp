/* Copyright 2017 The MathWorks, Inc. */

#ifndef __MEX_IO_ADAPTER_HPP__
#define __MEX_IO_ADAPTER_HPP__

#include "MatlabDataArray.hpp"

#include <vector>
#include "assert.h"

namespace matlab {
    namespace mex {
        template <typename iterator_type>
        class MexIORange {

            iterator_type begin_;

            iterator_type end_;

            size_t size_;
          public:

            MexIORange(iterator_type b, iterator_type e, size_t size) : begin_(b), end_(e), size_(size) {}

            typename std::iterator_traits<iterator_type>::difference_type size() {
                return size_;
            }

            typename std::iterator_traits<iterator_type>::difference_type internal_size() {
                return std::distance(begin_, end_);
            }

            iterator_type begin() {
                return begin_;
            }

            iterator_type end() {
                return end_;
            }

            bool empty() {
                return size() == 0;
            }

            typename std::iterator_traits<iterator_type>::reference operator[](size_t i) {
                if (static_cast<int>(i) + 1 > internal_size())
                    throw matlab::engine::Exception("ArgumentList index out of range.");

                return *(begin_ + i);
            }
        };
    }
}



matlab::data::Array getArray(void* v) {
    matlab::data::impl::ArrayImpl* impl = reinterpret_cast<matlab::data::impl::ArrayImpl*>(v);
    if (impl == nullptr)
        return matlab::data::Array();
    return matlab::data::detail::Access::createObj<matlab::data::Array>(impl);
}

void implToArray(int na, void* va[], std::vector<matlab::data::Array>& pa) {
    assert(na == static_cast<int>(pa.capacity()));

    for(int i = 0; i < na; i++) {
        matlab::data::impl::ArrayImpl* impl = reinterpret_cast<matlab::data::impl::ArrayImpl*>(va[i]);
        pa.push_back(matlab::data::detail::Access::createObj<matlab::data::Array>(impl));
    }
}

void arrayToImpl(int na, void* va[], const std::vector<matlab::data::Array>& pa) {
    for(int i = 0; i < na; i++) {
        va[i] = matlab::data::detail::Access::getImpl<matlab::data::impl::ArrayImpl>(pa[i]);
    }
}

void arrayToImplOutput(int nlhs, std::vector<matlab::data::Array>& edi_plhs, void (*callbackOutput)(int, void**)) {
    assert(nlhs == static_cast<int>(edi_plhs.size()));
    std::unique_ptr<matlab::data::impl::ArrayImpl*, void(*)(matlab::data::impl::ArrayImpl**)> vlhsPtr(new matlab::data::impl::ArrayImpl*[nlhs], [](matlab::data::impl::ArrayImpl** ptr) {
        delete[] ptr;
    });
    void** vlhs = (void**)vlhsPtr.get();
    arrayToImpl(nlhs, vlhs, edi_plhs);
    callbackOutput(nlhs, vlhs);
}

#endif
