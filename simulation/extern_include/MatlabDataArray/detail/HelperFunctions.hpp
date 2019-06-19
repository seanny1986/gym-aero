/* Copyright 2016-2017 The MathWorks, Inc. */

#ifndef MATLAB_EXTDATA_HELPER_FUNCTIONS_HPP
#define MATLAB_EXTDATA_HELPER_FUNCTIONS_HPP

#include "publish_util.hpp"
#include "FunctionType.hpp"

#include <string>
#include <algorithm>
#include <memory>

MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C void* get_function_ptr(int fcn);

namespace matlab {
    namespace data {
        namespace impl {
            class ArrayImpl;
        }
    }
}

namespace matlab {
    namespace data {
        namespace detail {
            class Access {
            public:
                template<typename ImplT, typename ObjT>
                static ImplT* getImpl(const ObjT& a) {
                    return static_cast<ImplT *>(a.pImpl.get());
                }
                template<typename ImplT, typename ObjT>
                static ImplT* getImplPtr(const ObjT& a) {
                    return static_cast<ImplT *>(a.pImpl);
                }
                template<typename ObjT>
                static matlab::data::impl::ArrayImpl* getImplForWorkspace(const ObjT& a) {
                    typedef matlab::data::impl::ArrayImpl*(*ArrayGetImplForWorkspaceFcnPtr)(matlab::data::impl::ArrayImpl*);
                    static const ArrayGetImplForWorkspaceFcnPtr fcn = (ArrayGetImplForWorkspaceFcnPtr)get_function_ptr(static_cast<int>(detail::FunctionType::ARRAY_GET_IMPL_FOR_WORKSPACE));
                    return fcn(a.pImpl.get());
                }
                template <typename Obj, typename Impl>
                static Obj createObj(Impl* impl) {
                    return Obj(impl);
                }
                template <typename Obj, typename Impl>
                static Obj createObj(std::shared_ptr<Impl> impl) {
                    return Obj(impl);
                }
                template <typename Obj, typename Impl, typename T>
                static Obj createObj(Impl* impl, T arg1) {
                    return Obj(impl, arg1);
                }
                template<typename ImplT>
                static void releaseImplFromWorkspace(ImplT impl) {
                    typedef void(*ArrayReleaseImplFromWorkspaceFcnPtr)(matlab::data::impl::ArrayImpl*);
                    static const ArrayReleaseImplFromWorkspaceFcnPtr fcn = (ArrayReleaseImplFromWorkspaceFcnPtr)
                        get_function_ptr(static_cast<int>(detail::FunctionType::ARRAY_RELEASE_IMPL_FROM_WORKSPACE));
                    return fcn(impl);
                }
            };
        }
    }
}

#endif
