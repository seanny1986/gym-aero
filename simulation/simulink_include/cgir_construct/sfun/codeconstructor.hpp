// Copyright 2007-2012 The MathWorks, Inc.

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SFUN_CODECONSTRUCTOR_HPP
#define SFUN_CODECONSTRUCTOR_HPP

#include "impl/codeconstructor_c_api.hpp"
#include "fwd.hpp"
#include "impl/rtw_api_lint_begin.hpp"

#define RTWCG_S_FUNCTION_API_REV  1

#if RTWCG_S_FUNCTION_API_REV != 1
#error You must #define RTWCG_S_FUNCTION_API_REV equal to 1 in your S-function source code file.
#endif

#define RTWCG_S_FUNCTION_BINARY_REV  2 

#if defined(_MSC_VER) && _MSC_VER < 1300
#error When using Microsoft Visual C++, this API requires version 7.0, also known as Visual Studio.NET 2002, or later.
#endif

#ifdef __WATCOMC__
#error This API does not support the Watcom C++ compiler.
#endif

const PreDefinedDTypeId SS_INDEX = SS_INTEGER;

namespace RTW
{
   
    class CodeConstructor
    {
      public:
        explicit CodeConstructor(SFun_CodeCtor_Impl* pImpl);
        virtual ~CodeConstructor();

        Reference createLocal(const Type& dataType, const char* name = NULL);

        Value input(int index);

        Reference output(int index);
       
        const SFun_CodeCtor_Impl* getImpl() const;
        SFun_CodeCtor_Impl* getImpl();
      private:
       
        CodeConstructor(const CodeConstructor& other);
        CodeConstructor& operator=(const CodeConstructor& other);

        SFun_CodeCtor_Impl* fpImpl;
    };

}

#include "impl/rtw_api_lint_end.hpp"

#include "reference.hpp"
#include "value.hpp"
#include "impl/rtw_api_lint_begin.hpp"

namespace RTW
{
    inline const SFun_CodeCtor_Impl* CodeConstructor::getImpl() const
    {
        return fpImpl;
    }

    inline SFun_CodeCtor_Impl* CodeConstructor::getImpl()
    {
        return fpImpl;
    }

    inline CodeConstructor::CodeConstructor(SFun_CodeCtor_Impl* pImpl)
        : fpImpl(pImpl)
    {
        verify(fpImpl);
    }

    inline CodeConstructor::~CodeConstructor()
    {
        SFun_Block_dtor(fpImpl);
    }

    inline Reference CodeConstructor::createLocal(const Type& dataType,
                                                  const char* name)
    {
        SFun_Reference_Impl* pResult =
            SFun_Block_createLocal_T_c(
                fpImpl, dataType.getImpl(), name);
        return Reference(pResult);
    }

    inline Value CodeConstructor::input(int index)
    {
        SFun_Value_Impl* pResult = SFun_Block_input_i(fpImpl, index);
        return Value(pResult);
    }

    inline Reference CodeConstructor::output(int index)
    {
        SFun_Reference_Impl* pResult = SFun_Block_output_i(fpImpl, index);
        return Reference(pResult);
    }
}

#include "impl/rtw_api_lint_end.hpp"

#endif
