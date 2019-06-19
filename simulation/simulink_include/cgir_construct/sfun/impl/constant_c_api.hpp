// Copyright 2007-2012 The MathWorks, Inc.

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SFUN_CONSTANT_C_CAPI_HPP
#define SFUN_CONSTANT_C_CAPI_HPP

#include "../libmwcgir_construct.hpp"
#include "../fwd.hpp"

namespace RTW
{
   
    extern "C"
    {
       
        CGIR_CONSTRUCT_API SFun_Constant_Impl* SFun_Constant_ctor_b(int val);
        CGIR_CONSTRUCT_API SFun_Constant_Impl* SFun_Constant_ctor_c(char val);
        CGIR_CONSTRUCT_API SFun_Constant_Impl* SFun_Constant_ctor_sc(signed char val);
        CGIR_CONSTRUCT_API SFun_Constant_Impl* SFun_Constant_ctor_s(short val);
        CGIR_CONSTRUCT_API SFun_Constant_Impl* SFun_Constant_ctor_i(int val);
        CGIR_CONSTRUCT_API SFun_Constant_Impl* SFun_Constant_ctor_l(long val);
        CGIR_CONSTRUCT_API SFun_Constant_Impl* SFun_Constant_ctor_ll(long long val);
        CGIR_CONSTRUCT_API SFun_Constant_Impl* SFun_Constant_ctor_uc(unsigned char val);
        CGIR_CONSTRUCT_API SFun_Constant_Impl* SFun_Constant_ctor_us(unsigned short val);
        CGIR_CONSTRUCT_API SFun_Constant_Impl* SFun_Constant_ctor_ui(unsigned int val);
        CGIR_CONSTRUCT_API SFun_Constant_Impl* SFun_Constant_ctor_ul(unsigned long val);
        CGIR_CONSTRUCT_API SFun_Constant_Impl* SFun_Constant_ctor_ull(unsigned long long val);
        CGIR_CONSTRUCT_API SFun_Constant_Impl* SFun_Constant_ctor_f(float val);
        CGIR_CONSTRUCT_API SFun_Constant_Impl* SFun_Constant_ctor_d(double val);

        CGIR_CONSTRUCT_API SFun_Constant_Impl* SFun_Constant_cctor(
            const SFun_Constant_Impl* pOther);
        CGIR_CONSTRUCT_API void SFun_Constant_dtor(SFun_Constant_Impl* pThis);

    }
}

#endif
