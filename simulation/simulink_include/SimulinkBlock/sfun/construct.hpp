// Copyright 2007-2012 The MathWorks, Inc.

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SFUN_CONSTRUCT_HPP
#define SFUN_CONSTRUCT_HPP
#include "../sl_core_block_spec.hpp"

#include "block.hpp"
#include "blockfunction.hpp"

namespace RTW
{
   
    template<typename DerivedBlock>
    static void construct_code_for_user_block(SimStruct *S, void *rtwBlk);

    template<typename DerivedBlock>
    static void do_construct_code(SimStruct *S, void *rtwBlk, SFun_BlockFunctionTable* pFcnTable)
    {
        SFun_Block_Impl* pImpl = SFun_create_Block_Impl(
                                        S,
                                        rtwBlk,
                                        pFcnTable,
                                        RTWCG_S_FUNCTION_API_REV,
                                        RTWCG_S_FUNCTION_BINARY_REV);
        DerivedBlock block(pImpl);
        block.constructCode();
    }

    template<typename DerivedBlock>
    static void construct_code_for_user_block(SimStruct *S, void *rtwBlk)
    {
        try
        {
            static SFun_BlockFunctionTable_SmartPtr pFcnTable(build_BlockFunctionTable<DerivedBlock>());
            do_construct_code<DerivedBlock>(S, rtwBlk, pFcnTable.get());
            SFun_setLastError_c(NULL);
        }
        catch(...)
        {
            setErrorInfoFromActiveException();
        }
    }
}

#endif
