// Copyright 2007-2012 The MathWorks, Inc.

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SFUN_FUNCTION_C_API_HPP
#define SFUN_FUNCTION_C_API_HPP

#include "../libmwcgir_construct.hpp"
#include "../fwd.hpp"
#include <cstddef>

namespace RTW
{
    extern "C"
    {
       
        CGIR_CONSTRUCT_API SFun_Function_Impl* SFun_Function_ctor_c_c_T_T_sz(
            const char* name,
            const char* headerFile,
            const SFun_Type_Impl* pOutputType,
            const SFun_Type_Impl* inputTypes[],
            std::size_t numInputTypes);

        CGIR_CONSTRUCT_API SFun_Function_Impl* SFun_Function_cctor(
            const SFun_Function_Impl* pOther);
        CGIR_CONSTRUCT_API int SFun_Function_op_assign(
            SFun_Function_Impl* pThis,
            const SFun_Function_Impl* pOther);

        CGIR_CONSTRUCT_API void SFun_Function_dtor(
            SFun_Function_Impl* pThis);

        CGIR_CONSTRUCT_API int SFun_Function_setStoresAddr_b(
            SFun_Function_Impl* pThis,
            int storesAddress);

        CGIR_CONSTRUCT_API int SFun_Function_setPure_b(
            SFun_Function_Impl* pThis,
            int isPure);
        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Function_op_fnCall_V_sz(
            const SFun_Function_Impl* pThis,
            const SFun_Value_Impl* args[],
            std::size_t numArgs);
    }
}

#endif
