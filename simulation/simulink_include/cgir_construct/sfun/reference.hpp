// Copyright 2007-2012 The MathWorks, Inc.

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SFUN_REFERENCE_HPP
#define SFUN_REFERENCE_HPP

#include "fwd.hpp"
#include <list>
#include "impl/rtw_api_lint_begin.hpp"

namespace RTW
{
    extern "C" typedef int (*PfnAssign)(const SFun_Reference_Impl*,
                                        const SFun_Value_Impl*);

    class Reference
    {
      public:
       
        Reference(const Reference& other);

        const Reference& operator=(const Value& newValue) const;
       
        const Reference& operator=(const Constant& newValue) const;
       
        const Reference& operator=(const Reference& newValue) const;

        ~Reference();

        Value dynamicSize() const;

        void setDynamicSize(const Value& newSize) const;

        Value dynamicWidth() const;

        Type type() const;
       
        Reference operator[](const Value& index) const;
       
        Reference operator[](const Constant& index) const;

        explicit Reference(SFun_Reference_Impl* pImpl);
        const SFun_Reference_Impl* getImpl() const;
        SFun_Reference_Impl* getImpl();

      private:
        SFun_Reference_Impl* fpImpl;
    };

    Value addressOf(const Reference& var);

}

#include "impl/rtw_api_lint_end.hpp"

#include "impl/reference_c_api.hpp"
#include "impl/api_util.hpp"
#include "fieldaccessproxy.hpp"
#include "value.hpp"
#include "type.hpp"
#include <vector>
#include "impl/rtw_api_lint_begin.hpp"

namespace RTW
{
   
    inline Reference::Reference(SFun_Reference_Impl* pImpl)
        : fpImpl(pImpl)
    {
        verify(fpImpl);
    }

    inline const SFun_Reference_Impl* Reference::getImpl() const
    {
        return fpImpl;
    }

    inline SFun_Reference_Impl* Reference::getImpl()
    {
        return fpImpl;
    }

    inline Reference::Reference(const Reference& other)
        : fpImpl(SFun_Reference_cctor(other.fpImpl))
    {
        verify(fpImpl);
    }

    inline const Reference& Reference::operator=(const Value& newValue) const
    {
        int success = SFun_Reference_op_assign_V(fpImpl, newValue.fpImpl);
        verify(success);
        return *this;
    }

    inline const Reference& Reference::operator=(const Constant& newValue) const
    {
        int success = SFun_Reference_op_assign_C(fpImpl, newValue.getImpl());
        verify(success);
        return *this;
    }

    inline const Reference& Reference::operator=(const Reference& newValue) const
    {
        int success = SFun_Reference_op_assign(fpImpl, newValue.fpImpl);
        verify(success);
        return *this;
    }

    inline Reference::~Reference()
    {
        SFun_Reference_dtor(fpImpl);
    }

    inline Value Reference::dynamicSize() const
    {
        SFun_Value_Impl* pResult = SFun_Reference_dynamicSize(fpImpl);
        return Value(pResult);
    }

    inline void Reference::setDynamicSize(const Value& newSize) const
    {
        int success = SFun_Reference_setDynamicSize(fpImpl, newSize.getImpl());
        verify(success);
    }

    inline Value Reference::dynamicWidth() const
    {
        SFun_Value_Impl* pResult = SFun_Reference_dynamicWidth(fpImpl);
        return Value(pResult);
    }

    inline Type Reference::type() const
    {
        SFun_Type_Impl* pResult = SFun_Reference_type(fpImpl);
        return Type(pResult);
    }
    inline Reference Reference::operator[](const Value& index) const
    {
        SFun_Reference_Impl* pResult = SFun_Reference_op_subscript_V(
            fpImpl,
            index.getImpl());
        return Reference(pResult);
    }

    inline Reference Reference::operator[](const Constant& index) const
    {
        SFun_Reference_Impl* pResult = SFun_Reference_op_subscript_C(
            fpImpl,
            index.getImpl());
        return Reference(pResult);
    }

    inline Value addressOf(const Reference& var)
    {
        SFun_Value_Impl* pResult = SFun_addressOf_R(var.getImpl());
        return Value(pResult);
    }
}

#include "impl/rtw_api_lint_end.hpp"

#endif
