// Copyright 2007-2012 The MathWorks, Inc.

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SFUN_REFERENCE_C_API_HPP
#define SFUN_REFERENCE_C_API_HPP

#include "../libmwcgir_construct.hpp"
#include "../fwd.hpp"
#include <cstddef>

namespace RTW
{
    extern "C"
    {
        CGIR_CONSTRUCT_API SFun_Reference_Impl* SFun_Reference_cctor(
            const SFun_Reference_Impl* pOther);

        CGIR_CONSTRUCT_API int SFun_Reference_op_assign_V(
            const SFun_Reference_Impl* pThis,
            const SFun_Value_Impl* pNewValue);

        CGIR_CONSTRUCT_API int SFun_Reference_op_assign_C(
            const SFun_Reference_Impl* pThis,
            const SFun_Constant_Impl* pNewValue);

        CGIR_CONSTRUCT_API int SFun_Reference_op_assign(
            const SFun_Reference_Impl* pThis,
            const SFun_Reference_Impl* pNewValue);

        CGIR_CONSTRUCT_API void SFun_Reference_dtor(
            SFun_Reference_Impl* pThis);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Reference_dynamicSize(
            const SFun_Reference_Impl* pThis);

        CGIR_CONSTRUCT_API int SFun_Reference_setDynamicSize(
            const SFun_Reference_Impl* pThis,
            const SFun_Value_Impl* newSize);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Reference_dynamicWidth(
            const SFun_Reference_Impl* pThis);

        CGIR_CONSTRUCT_API SFun_Type_Impl* SFun_Reference_type(
            const SFun_Reference_Impl* pThis);
        CGIR_CONSTRUCT_API SFun_Reference_Impl* SFun_Reference_op_subscript_V(
            const SFun_Reference_Impl* pThis,
            const SFun_Value_Impl* index);

        CGIR_CONSTRUCT_API SFun_Reference_Impl* SFun_Reference_op_subscript_C(
            const SFun_Reference_Impl* pThis,
            const SFun_Constant_Impl* index);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_addressOf_R(
            const SFun_Reference_Impl* var);
    }
}

#endif
