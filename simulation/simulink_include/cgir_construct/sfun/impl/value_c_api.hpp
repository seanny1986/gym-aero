// Copyright 2007-2017 The MathWorks, Inc.

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SFUN_VALUE_C_API_HPP
#define SFUN_VALUE_C_API_HPP

#include "../libmwcgir_construct.hpp"
#include "../fwd.hpp"

#if defined(EXPORT_CGIR_CONSTRUCT_API) || defined(DLL_IMPORT_SYM)

# include "simstruct/simstruc.h" 

#include "fixpoint/fixpoint_spec.hpp"
#include "fixpoint/Published/fxpPublishedIntro.hpp"
#include "fixedpointcore/published/fxpLimits.hpp"
#include "fixedpointcore/published/fxpChunkArray.hpp"
#include "fixpoint/Published/fxpPublishedScalingIntro.hpp"
#include "sl_types/FxpModeOverflow.hpp"
#include "sl_types/FxpModeRounding.hpp"
#include "fixedpointcore/published/fxpOverflowLogs.hpp"
#include "fixpoint/Published/fixPublishedSFunAPI.hpp"
#include "fixpoint/Published/fixPublishedDeprecated.hpp"

#else

# include "simstruc.h" 
#include "fixedpoint.h"
#endif

#include <cstddef>

namespace RTW
{
   
    extern "C"
    {
       
        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_ctor_R(
            const SFun_Reference_Impl* pReference);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_cctor(
            const SFun_Value_Impl* pOther);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_ctor_T_b_c(
            const SFun_Type_Impl* type,
            int val,
            const char* name);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_ctor_T_c_c(
            const SFun_Type_Impl* type,
            char val,
            const char* name);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_ctor_T_sc_c(
            const SFun_Type_Impl* type,
            signed char val,
            const char* name);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_ctor_T_s_c(
            const SFun_Type_Impl* type,
            short val,
            const char* name);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_ctor_T_i_c(
            const SFun_Type_Impl* type,
            int val,
            const char* name);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_ctor_T_l_c(
            const SFun_Type_Impl* type,
            long val,
            const char* name);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_ctor_T_ll_c(
            const SFun_Type_Impl* type,
            long long val,
            const char* name);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_ctor_T_uc_c(
            const SFun_Type_Impl* type,
            unsigned char val,
            const char* name);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_ctor_T_us_c(
            const SFun_Type_Impl* type,
            unsigned short val,
            const char* name);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_ctor_T_ui_c(
            const SFun_Type_Impl* type,
            unsigned int val,
            const char* name);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_ctor_T_ul_c(
            const SFun_Type_Impl* type,
            unsigned long val,
            const char* name);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_ctor_T_ull_c(
            const SFun_Type_Impl* type,
            unsigned long long val,
            const char* name);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_ctor_T_f_c(
            const SFun_Type_Impl* type,
            float val,
            const char* name);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_ctor_T_d_c(
            const SFun_Type_Impl* type,
            double val,
            const char* name);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_ctor_T_C_d_c(
            const SFun_Type_Impl* type,
            const SFun_Constant_Impl* val,
            const char* name);
        CGIR_CONSTRUCT_API void SFun_Value_dtor(SFun_Value_Impl* pThis);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_dynamicSize(
            const SFun_Value_Impl* pThis);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_dynamicWidth(
            const SFun_Value_Impl* pThis);

        CGIR_CONSTRUCT_API SFun_Type_Impl* SFun_Value_type(
            const SFun_Value_Impl* pThis);

        CGIR_CONSTRUCT_API int SFun_Value_getCodeCtor(
            CodeConstructor** result,
            const SFun_Value_Impl* pThis);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_op_subscript_V(
            const SFun_Value_Impl* pThis,
            const SFun_Value_Impl* pIndex);

        CGIR_CONSTRUCT_API SFun_Value_Impl* SFun_Value_op_subscript_C(
            const SFun_Value_Impl* pThis,
            const SFun_Constant_Impl* pIndex);
    }
}
#endif 
