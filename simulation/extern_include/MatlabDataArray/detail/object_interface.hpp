/* Copyright 2015 The MathWorks, Inc. */

#ifndef OBJECT_INTERFACE_HPP_
#define OBJECT_INTERFACE_HPP_

#include "publish_util.hpp"

namespace matlab {
    namespace data {
        class ValueObject;
      
        namespace detail {
            class ReferenceImpl;
            class ForwardIteratorImpl;
            class RefCounted;
        }
        namespace impl {
            class ObjectImpl;
            class MatlabIdentifierImpl;
            class ArrayImpl;
        }
    }
}

MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C matlab::data::detail::ForwardIteratorImpl* object_begin(matlab::data::impl::ObjectImpl* impl);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C matlab::data::detail::ForwardIteratorImpl* object_end(matlab::data::impl::ObjectImpl* impl);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C matlab::data::detail::ForwardIteratorImpl* object_reference_begin(matlab::data::detail::ReferenceImpl* impl);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C matlab::data::detail::ForwardIteratorImpl* object_reference_end(matlab::data::detail::ReferenceImpl* impl);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C matlab::data::detail::ReferenceImpl* object_get_prop_ref(matlab::data::impl::ObjectImpl* impl, matlab::data::detail::RefCounted*, bool is_const_ref);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C matlab::data::detail::RefCounted* object_get_class_info(matlab::data::impl::ObjectImpl* impl);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C const char* object_get_enumeration_name(matlab::data::impl::ObjectImpl* impl);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C bool object_unshare(matlab::data::impl::ObjectImpl* impl, bool isUserObjectUnique, matlab::data::impl::ObjectImpl** newImpl); 
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C void object_set_value(matlab::data::detail::ReferenceImpl* impl, matlab::data::impl::ObjectImpl* rhs);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C void object_reference_shared_copy(matlab::data::detail::ReferenceImpl* impl, matlab::data::impl::ObjectImpl** cpy);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C void object_destroy_impl(matlab::data::impl::ObjectImpl* impl);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C void object_reference_get_reference_value(matlab::data::detail::ReferenceImpl* impl, const char* prop, size_t len, bool unshare, matlab::data::detail::ReferenceImpl** retVal);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C void object_reference_get_reference_value_with_id(matlab::data::detail::ReferenceImpl* impl, matlab::data::detail::RefCounted* prop, bool unshare, matlab::data::detail::ReferenceImpl** retVal);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C matlab::data::detail::RefCounted* object_array_get_class_info(matlab::data::impl::ArrayImpl* impl);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C matlab::data::detail::RefCounted* object_reference_get_class_info(matlab::data::detail::ReferenceImpl* impl);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C const char* object_reference_get_enumeration_name(matlab::data::detail::ReferenceImpl* impl);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C bool object_array_is_heterogeneous(matlab::data::impl::ArrayImpl* impl);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C bool handle_object_ref_is_equal(matlab::data::impl::ObjectImpl* lhs, matlab::data::impl::ObjectImpl* rhs);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C bool handle_object_ref_get_id(matlab::data::impl::ObjectImpl* impl, int64_t* retVal);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C bool handle_object_ref_get_mvm_id(matlab::data::impl::ObjectImpl* impl, int8_t* retVal);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C bool handle_ref_get_id(matlab::data::detail::ReferenceImpl* impl, int64_t* retVal);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C bool handle_ref_get_mvm_id(matlab::data::detail::ReferenceImpl* impl, int8_t* retVal);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C void handle_ref_set_value(matlab::data::detail::ReferenceImpl* impl, matlab::data::impl::ObjectImpl* rhs);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C bool handle_ref_is_equal(matlab::data::detail::ReferenceImpl* lhs, matlab::data::detail::ReferenceImpl* rhs);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C bool handle_ref_is_equal_to_object(matlab::data::detail::ReferenceImpl* lhs, matlab::data::impl::ObjectImpl* rhs);
MATLAB_EXTDATA_PUBLISHED_API_EXTERN_C void handle_object_ref_reference_shared_copy(matlab::data::detail::ReferenceImpl* impl, matlab::data::impl::ObjectImpl** cpy);

#endif
