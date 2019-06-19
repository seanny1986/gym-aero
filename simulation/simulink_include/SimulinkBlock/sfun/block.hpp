// Copyright 2007-2018 The MathWorks, Inc.

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SFUN_BLOCK_HPP
#define SFUN_BLOCK_HPP
#include "../sl_core_block_spec.hpp"

#include "block_c_api.hpp"
#include "cgir_construct_from_SimulinkBlock.hpp"

namespace RTW
{
   
    class Block : public CodeConstructor
    {
    public:
        explicit Block(SFun_Block_Impl* pImpl);
        ~Block() override;

        void constructCode();

        virtual void cgStart() {}

        virtual void cgSetupRuntimeResources() {}

        virtual void cgInitialize() {}

        virtual void cgSystemInitialize() {}

        virtual void cgOutput() = 0;

        virtual void cgUpdate() {}

        virtual void cgTerminate() {}

        virtual void cgCleanupRuntimeResources() {}

        virtual void cgFcnCallPortInitialize(int , int ) {}
        virtual void cgFcnCallPortStart     (int , int ) {}
        virtual void cgFcnCallPortRun       (int , int ) {}
        virtual void cgFcnCallPortEnable    (int , int ) {}
        virtual void cgFcnCallPortDisable   (int , int ) {}

        SimStruct* getSimStruct();

        SLBlock* getSlBlock();

        Value param(int index);

        Reference dWork(int index = 0);
    private:
       
        Block(const Block& other);
        Block& operator=(const Block& other);
    };

}

namespace RTW
{
   
    inline Block::Block(SFun_Block_Impl* pImpl)
        : CodeConstructor(reinterpret_cast<SFun_CodeCtor_Impl*>(pImpl))
    {
        verify( SFun_Block_setOuter_B(getImpl(), this) );
    }

    inline Block::~Block()
    {
       
    }

    inline void Block::constructCode()
    {
        int result = SFun_Block_constructCode(getImpl());
        verify(result);
    }

    inline SimStruct* Block::getSimStruct()
    {
        SimStruct* pS = SFun_Block_getSimStruct(getImpl());
        verify(pS);
        return pS;
    }

    inline SLBlock* Block::getSlBlock()
    {
        SLBlock* block = SFun_Block_getSlBlock(getImpl());
        verify(block);
        return block;
    }

    inline Value Block::param(int index)
    {
        SFun_Value_Impl* pResult = SFun_Block_param_i(getImpl(), index);
        return Value(pResult);
    }

    inline Reference Block::dWork(int index)
    {
        SFun_Reference_Impl* pResult = SFun_Block_dWork_i(getImpl(), index);
        return Reference(pResult);
    }
}

#endif
