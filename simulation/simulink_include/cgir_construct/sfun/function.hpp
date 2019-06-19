// Copyright 2007-2012 The MathWorks, Inc.

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SFUN_FUNCTION_HPP
#define SFUN_FUNCTION_HPP

#include "fwd.hpp"
#include "value.hpp"
#include "impl/rtw_api_lint_begin.hpp"
#include <cstdlib>
#include <list>

namespace RTW
{
   
    class Function
    {
      public:
       
        Function(const char* name, const char* headerFile, const Type& outputType,
                 const Type inputTypes[], std::size_t numInputTypes);
       
        Function(const Function& other);

        Function& operator=(const Function& other);

        ~Function();

        void setStoresAddress(bool storesAddress);

        void setPure(bool isPure);
       
        Value operator()() const;

        Value operator()(const Value& arg) const;

        Value operator()(const Value& arg0, const Value& arg1) const;

        Value operator()(const Value& arg0, const Value& arg1, const Value& arg2) const;

        Value operator()(const Value args[], std::size_t numArgs) const;

        Value operator()(const std::list<Value>& args) const;
       
        explicit Function(SFun_Function_Impl* pImpl);

      private:
        void initialize(
            const char* name,
            const char* headerFile,
            const Type& outputType,
            const Type inputTypes[],
            std::size_t numInputTypes);

        Value doCall(const SFun_Value_Impl*[], std::size_t numArgs) const;

        SFun_Function_Impl* fpImpl;
    };

}

#include "impl/rtw_api_lint_end.hpp"

#include "impl/function_c_api.hpp"
#include "impl/api_util.hpp"
#include "type.hpp"
#include "value.hpp"
#include <vector>
#include "impl/rtw_api_lint_begin.hpp"

namespace RTW
{
    inline Function::Function(const char* name,
                              const char* headerFile,
                              const Type& outputType,
                              const Type inputTypes[],
                              std::size_t numInputTypes)
    {
        initialize(name, headerFile, outputType, inputTypes, numInputTypes);
    }
   
    inline void Function::initialize(const char* name,
                                     const char* headerFile,
                                     const Type& outputType,
                                     const Type inputTypes[],
                                     std::size_t numInputTypes)
    {
        std::vector<const SFun_Type_Impl*> inputTypeImpls(numInputTypes);
        for (std::size_t i = 0 ; i != numInputTypes ; ++i)
            inputTypeImpls[i] = inputTypes[i].getImpl() ;

        fpImpl = SFun_Function_ctor_c_c_T_T_sz(
            name,
            headerFile,
            outputType.getImpl(),
            &inputTypeImpls[0],
            numInputTypes);
        verify(fpImpl);
    }

    inline Function::Function(SFun_Function_Impl* pImpl)
        : fpImpl(pImpl)
    {
        verify(fpImpl);
    }

    inline Function::Function(const Function& other)
        : fpImpl(SFun_Function_cctor(other.fpImpl))
    {
        verify(fpImpl);
    }

    inline Function& Function::operator=(const Function& other)
    {
        verify(SFun_Function_op_assign(fpImpl, other.fpImpl));
        return *this;
    }

    inline Function::~Function()
    {
        SFun_Function_dtor(fpImpl);
    }

    inline void Function::setStoresAddress(bool storesAddress)
    {
        verify(SFun_Function_setStoresAddr_b(fpImpl, storesAddress));
    }

    inline void Function::setPure(bool isPure)
    {
        verify(SFun_Function_setPure_b(fpImpl, isPure));
    }
    inline Value Function::operator()() const
    {
        return doCall(NULL, 0);
    }

    inline Value Function::operator()(const Value& arg0) const
    {
        const SFun_Value_Impl* args[] = { arg0.getImpl() };
        return doCall(args, sizeof(args)/sizeof(args[0]));
    }

    inline Value Function::operator()(const Value& arg0,
                                      const Value& arg1) const
    {
        const SFun_Value_Impl* args[] = { arg0.getImpl(), arg1.getImpl() };
        return doCall(args, sizeof(args)/sizeof(args[0]));
    }

    inline Value Function::operator()(const Value& arg0,
                                      const Value& arg1,
                                      const Value& arg2) const
    {
        const SFun_Value_Impl* args[] = { arg0.getImpl(), arg1.getImpl(), arg2.getImpl() };
        return doCall(args, sizeof(args)/sizeof(args[0]));
    }

    inline Value Function::operator()(const Value args[],
                                      std::size_t numArgs) const
    {
        std::vector<const SFun_Value_Impl*> argImpls(numArgs);
        for (std::size_t i = 0 ; i != numArgs ; ++i)
            argImpls[i] = args[i].getImpl() ;

        return doCall(
            numArgs == 0 ? NULL : &argImpls[0],
            numArgs);
    }

    inline Value Function::operator()(const std::list<Value>& args) const
    {
        std::vector<const SFun_Value_Impl*> argImpls;
        for (std::list<Value>::const_iterator pArg = args.begin() ;
             pArg != args.end() ;
             ++pArg) {
            argImpls.push_back(pArg->getImpl());
        }

        return doCall(
            argImpls.empty() ? NULL : &argImpls[0],
            argImpls.size());
    }

    inline Value Function::doCall(const SFun_Value_Impl* args[],
                                  std::size_t numArgs) const
    {
        SFun_Value_Impl* pResult = SFun_Function_op_fnCall_V_sz(
            fpImpl,
            args,
            numArgs);
        return Value(pResult);
    }
}

#include "impl/rtw_api_lint_end.hpp"

#endif
