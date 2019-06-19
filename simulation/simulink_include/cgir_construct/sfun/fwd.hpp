// Copyright 2007-2018 The MathWorks, Inc.

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef CGIR_CONSTRUCT_RTW_API_FWD_HPP
#define CGIR_CONSTRUCT_RTW_API_FWD_HPP
namespace RTW
{
    extern "C"
    {
        class CodeConstructor;
        class CodeGenContext;
        class Constant;
        class FieldAccessProxy;
        class FragmentConstructor;
        class Function;
        class FunctionFamily;
        class GeneratedFunction;
        class Reference;
        class Type;
        class Value;

        struct EnumConstant;

        class SFun_CodeCtor_Impl;
        class SFun_Constant_Impl;
        class SFun_FieldAccessProxy_Impl;
        class SFun_FragmentCtor_Impl;
        class SFun_Function_Impl;
        class SFun_FunctionFamily_Impl;
        class SFun_GeneratedFunction_Impl;
        class SFun_Reference_Impl;
        class SFun_Type_Impl;
        class SFun_Value_Impl;
        class RTWCGBlock_CodeCtor_Impl;

        struct SFun_GeneratedFunction_VTable;
    }
}

namespace SFun = RTW;

#endif
