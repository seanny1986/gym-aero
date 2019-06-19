// Copyright 2007-2017 The MathWorks, Inc.

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SFUN_VALUE_HPP
#define SFUN_VALUE_HPP

#include "fwd.hpp"
#include "impl/value_c_api.hpp"
#include "constant.hpp"
#include <list>
#include <utility>
#include "impl/rtw_api_lint_begin.hpp"

namespace RTW
{
   
    class Value
    {
      public:
       
        Value(const Reference& ref); 

        Value(const Value& other);

        Value(const Type& type, bool val,               const char* name = NULL);

        Value(const Type& type, char val,               const char* name = NULL);

        Value(const Type& type, signed char val,        const char* name = NULL);

        Value(const Type& type, short val,              const char* name = NULL);

        Value(const Type& type, int val,                const char* name = NULL);

        Value(const Type& type, long val,               const char* name = NULL);

        Value(const Type& type, long long val,          const char* name = NULL);

        Value(const Type& type, unsigned char val,      const char* name = NULL);

        Value(const Type& type, unsigned short val,     const char* name = NULL);

        Value(const Type& type, unsigned int val,       const char* name = NULL);

        Value(const Type& type, unsigned long val,      const char* name = NULL);

        Value(const Type& type, unsigned long long val, const char* name = NULL);

        Value(const Type& type, float val,              const char* name = NULL);

        Value(const Type& type, double val,             const char* name = NULL);

        Value(const Type& type, const Constant& val,    const char* name = NULL);
        ~Value();

        Value operator[](const Value& index) const;

        Value operator[](const Constant& index) const;
       
        Value dynamicSize() const;

        Value dynamicWidth() const;

        Type type() const;
       
        explicit Value(SFun_Value_Impl* pImpl);
        SFun_Value_Impl* getImpl();
        const SFun_Value_Impl* getImpl() const;

      private:
       
        Value& operator=(const Value& rhs);

        friend class Reference;

        SFun_Value_Impl* fpImpl;
    };
    
}

#include "impl/rtw_api_lint_end.hpp"

#include "impl/value_c_api.hpp"
#include "fieldaccessproxy.hpp"
#include "type.hpp"
#include "reference.hpp"
#include "impl/api_util.hpp"
#include <vector>
#include "impl/rtw_api_lint_begin.hpp"

namespace RTW
{
   
    inline Value::Value(SFun_Value_Impl* pImpl)
        : fpImpl(pImpl)
    {
        verify(fpImpl);
    }

    inline SFun_Value_Impl* Value::getImpl()
    {
        return fpImpl;
    }

    inline const SFun_Value_Impl* Value::getImpl() const
    {
        return fpImpl;
    }

    inline Value::Value(const Reference& ref)
        : fpImpl(SFun_Value_ctor_R(ref.getImpl()))
    {
        verify(fpImpl);
    }

    inline Value::Value(const Value& other)
        : fpImpl(SFun_Value_cctor(other.fpImpl))
    {
        verify(fpImpl);
    }

    inline Value::Value(const Type& aType, bool val,               const char* name)
        : fpImpl(SFun_Value_ctor_T_b_c(aType.getImpl(), val, name))
    {
        verify(fpImpl);
    }

    inline Value::Value(const Type& aType, char val,               const char* name)
        : fpImpl(SFun_Value_ctor_T_c_c(aType.getImpl(), val, name))
    {
        verify(fpImpl);
    }

    inline Value::Value(const Type& aType, signed char val,        const char* name)
        : fpImpl(SFun_Value_ctor_T_sc_c(aType.getImpl(), val, name))
    {
        verify(fpImpl);
    }

    inline Value::Value(const Type& aType, short val,              const char* name)
        : fpImpl(SFun_Value_ctor_T_s_c(aType.getImpl(), val, name))
    {
        verify(fpImpl);
    }

    inline Value::Value(const Type& aType, int val,                const char* name)
        : fpImpl(SFun_Value_ctor_T_i_c(aType.getImpl(), val, name))
    {
        verify(fpImpl);
    }

    inline Value::Value(const Type& aType, long val,               const char* name)
        : fpImpl(SFun_Value_ctor_T_l_c(aType.getImpl(), val, name))
    {
        verify(fpImpl);
    }

    inline Value::Value(const Type& aType, long long val,          const char* name)
        : fpImpl(SFun_Value_ctor_T_ll_c(aType.getImpl(), val, name))
    {
        verify(fpImpl);
    }

    inline Value::Value(const Type& aType, unsigned char val,      const char* name)
        : fpImpl(SFun_Value_ctor_T_uc_c(aType.getImpl(), val, name))
    {
        verify(fpImpl);
    }

    inline Value::Value(const Type& aType, unsigned short val,     const char* name)
        : fpImpl(SFun_Value_ctor_T_us_c(aType.getImpl(), val, name))
    {
        verify(fpImpl);
    }

    inline Value::Value(const Type& aType, unsigned int val,       const char* name)
        : fpImpl(SFun_Value_ctor_T_ui_c(aType.getImpl(), val, name))
    {
        verify(fpImpl);
    }

    inline Value::Value(const Type& aType, unsigned long val,      const char* name)
        : fpImpl(SFun_Value_ctor_T_ul_c(aType.getImpl(), val, name))
    {
        verify(fpImpl);
    }

    inline Value::Value(const Type& aType, unsigned long long val, const char* name)
        : fpImpl(SFun_Value_ctor_T_ull_c(aType.getImpl(), val, name))
    {
        verify(fpImpl);
    }

    inline Value::Value(const Type& aType, float val,              const char* name)
        : fpImpl(SFun_Value_ctor_T_f_c(aType.getImpl(), val, name))
    {
        verify(fpImpl);
    }

    inline Value::Value(const Type& aType, double val,             const char* name)
        : fpImpl(SFun_Value_ctor_T_d_c(aType.getImpl(), val, name))
    {
        verify(fpImpl);
    }

    inline Value::Value(const Type& aType, const Constant& val,    const char* name)
        : fpImpl(SFun_Value_ctor_T_C_d_c(aType.getImpl(), val.getImpl(), name))
    {
        verify(fpImpl);
    }
    inline Value::~Value()
    {
        SFun_Value_dtor(fpImpl);
    }

    inline Value Value::dynamicSize() const
    {
        SFun_Value_Impl* pResult = SFun_Value_dynamicSize(fpImpl);
        return Value(pResult);
    }

    inline Value Value::dynamicWidth() const
    {
        SFun_Value_Impl* pResult = SFun_Value_dynamicWidth(fpImpl);
        return Value(pResult);
    }

    inline Type Value::type() const
    {
        SFun_Type_Impl* pResult = SFun_Value_type(fpImpl);
        return Type(pResult);
    }

    inline Value Value::operator[](const Value& index) const
    {
        const SFun_Value_Impl* pThis = getImpl();
        const SFun_Value_Impl* index_impl = index.getImpl();

        SFun_Value_Impl* pResult = SFun_Value_op_subscript_V(pThis, index_impl);
        return Value(pResult);
    }

    inline Value Value::operator[](const Constant& index) const
    {
        const SFun_Value_Impl* pThis = getImpl();
        const SFun_Constant_Impl* index_impl = index.getImpl();

        SFun_Value_Impl* pResult = SFun_Value_op_subscript_C(pThis, index_impl);
        return Value(pResult);
    }
}

#include "impl/rtw_api_lint_end.hpp"

#endif
