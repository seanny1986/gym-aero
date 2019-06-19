// Copyright 2007-2012 The MathWorks, Inc.

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SFUN_CONSTANT_HPP
#define SFUN_CONSTANT_HPP

#include "fwd.hpp"

#include "impl/rtw_api_lint_begin.hpp"

namespace RTW
{
   
    class Constant
    {
      public:
       
        Constant(bool val);

        Constant(char val);

        Constant(signed char val);

        Constant(short val);

        Constant(int val);

        Constant(long val);

        Constant(long long val);

        Constant(unsigned char val);

        Constant(unsigned short val);

        Constant(unsigned int val);

        Constant(unsigned long val);

        Constant(unsigned long long val);

        Constant(float val);

        Constant(double val);

        Constant(const Constant& other);
        ~Constant();

        explicit Constant(SFun_Constant_Impl* pImpl);
        const SFun_Constant_Impl* getImpl() const;

      private:
       
        Constant(const void* val);

        Constant& operator=(const Constant& rhs);

        SFun_Constant_Impl* fpImpl;
    };
    
}

#include "impl/rtw_api_lint_end.hpp"

#include "impl/constant_c_api.hpp"
#include "type.hpp"
#include "impl/api_util.hpp"
#include "impl/rtw_api_lint_begin.hpp"

namespace RTW
{
   
    inline Constant::Constant(SFun_Constant_Impl* pImpl)
        : fpImpl(pImpl)
    {
        verify(fpImpl);
    }

    inline const SFun_Constant_Impl* Constant::getImpl() const
    {
        return fpImpl;
    }

    inline Constant::Constant(bool val)
        : fpImpl(SFun_Constant_ctor_b(val))
    {
        verify(fpImpl);
    }

    inline Constant::Constant(char val)
        : fpImpl(SFun_Constant_ctor_c(val))
    {
        verify(fpImpl);
    }

    inline Constant::Constant(signed char val)
        : fpImpl(SFun_Constant_ctor_sc(val))
    {
        verify(fpImpl);
    }

    inline Constant::Constant(short val)
        : fpImpl(SFun_Constant_ctor_s(val))
    {
        verify(fpImpl);
    }

    inline Constant::Constant(int val)
        : fpImpl(SFun_Constant_ctor_i(val))
    {
        verify(fpImpl);
    }

    inline Constant::Constant(long val)
        : fpImpl(SFun_Constant_ctor_l(val))
    {
        verify(fpImpl);
    }

    inline Constant::Constant(long long val)
        : fpImpl(SFun_Constant_ctor_ll(val))
    {
        verify(fpImpl);
    }

    inline Constant::Constant(unsigned char val)
        : fpImpl(SFun_Constant_ctor_uc(val))
    {
        verify(fpImpl);
    }

    inline Constant::Constant(unsigned short val)
        : fpImpl(SFun_Constant_ctor_us(val))
    {
        verify(fpImpl);
    }

    inline Constant::Constant(unsigned int val)
        : fpImpl(SFun_Constant_ctor_ui(val))
    {
        verify(fpImpl);
    }

    inline Constant::Constant(unsigned long val)
        : fpImpl(SFun_Constant_ctor_ul(val))
    {
        verify(fpImpl);
    }

    inline Constant::Constant(unsigned long long val)
        : fpImpl(SFun_Constant_ctor_ull(val))
    {
        verify(fpImpl);
    }

    inline Constant::Constant(float val)
        : fpImpl(SFun_Constant_ctor_f(val))
    {
        verify(fpImpl);
    }

    inline Constant::Constant(double val)
        : fpImpl(SFun_Constant_ctor_d(val))
    {
        verify(fpImpl);
    }

    inline Constant::Constant(const Constant& other)
        : fpImpl(SFun_Constant_cctor(other.fpImpl))
    {
        verify(fpImpl);
    }
    inline Constant::~Constant()
    {
        SFun_Constant_dtor(fpImpl);
    }

}

#endif
