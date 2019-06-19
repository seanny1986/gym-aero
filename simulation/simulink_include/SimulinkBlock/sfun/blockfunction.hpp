// Copyright 2007-2013 The MathWorks, Inc.

#ifdef SUPPORTS_PRAGMA_ONCE
#pragma once
#endif

#ifndef SFUN_BLOCKFUNCTION_HPP
#define SFUN_BLOCKFUNCTION_HPP

#include "../sl_core_block_spec.hpp"
#include "cgir_construct_from_SimulinkBlock.hpp"
#include "block.hpp"

namespace RTW
{
    inline int call_Block_method( Block* pBlock,
                                  void (Block::* pMemFun)())
    {
        try
        {
           
            (pBlock->*pMemFun)();
            return true;
        }
        catch(...)
        {
            setErrorInfoFromActiveException();
            return false;
        }
    }

    inline int call_Block_method( Block* pBlock,
                                  void (Block::* pMemFun)(int, int),
                                  int i,
                                  int j)
    {
        try
        {
           
            (pBlock->*pMemFun)(i, j);
            return true;
        }
        catch(...)
        {
            setErrorInfoFromActiveException();
            return false;
        }
    }

    extern "C"
    {
       
        typedef int (*SFun_BlockFunction)(CodeConstructor*);

        typedef int (*SFun_PortFunction)(CodeConstructor*, int, int);

        static inline int call_cgStart(CodeConstructor* pCodeCtor)
        {
            Block* pBlock = static_cast<Block*>(pCodeCtor);
            return call_Block_method(pBlock, &Block::cgStart);
        }

        static inline int call_cgSetupRuntimeResources(CodeConstructor* pCodeCtor)
        {
            Block* pBlock = static_cast<Block*>(pCodeCtor);
            return call_Block_method(pBlock, &Block::cgSetupRuntimeResources);
        }

        static inline int call_cgInitialize(CodeConstructor* pCodeCtor)
        {
            Block* pBlock = static_cast<Block*>(pCodeCtor);
            return call_Block_method(pBlock, &Block::cgInitialize);
        }

        static inline int call_cgSystemInitialize(CodeConstructor* pCodeCtor)
        {
            Block* pBlock = static_cast<Block*>(pCodeCtor);
            return call_Block_method(pBlock, &Block::cgSystemInitialize);
        }
       
        static inline int call_cgOutput(CodeConstructor* pCodeCtor)
        {
            Block* pBlock = static_cast<Block*>(pCodeCtor);
            return call_Block_method(pBlock, &Block::cgOutput);
        }

        static inline int call_cgUpdate(CodeConstructor* pCodeCtor)
        {
            Block* pBlock = static_cast<Block*>(pCodeCtor);
            return call_Block_method(pBlock, &Block::cgUpdate);
        }
        static inline int call_cgCleanupRuntimeResources(CodeConstructor* pCodeCtor)
        {
            Block* pBlock = static_cast<Block*>(pCodeCtor);
            return call_Block_method(pBlock, &Block::cgCleanupRuntimeResources);
        }

        static inline int call_cgTerminate(CodeConstructor* pCodeCtor)
        {
            Block* pBlock = static_cast<Block*>(pCodeCtor);
            return call_Block_method(pBlock, &Block::cgTerminate);
        }
        SL_CORE_BLOCK_EXPORT_CLASS SFun_BlockFunctionTable* SFun_new_BlockFcnTable();
        SL_CORE_BLOCK_EXPORT_CLASS void SFun_delete_BlockFcnTable(SFun_BlockFunctionTable* pFcnTable);

        SL_CORE_BLOCK_EXPORT_CLASS void SFun_set_cgStart          (SFun_BlockFunctionTable* pFcnTable, SFun_BlockFunction pfn_call_cgStart          );
        SL_CORE_BLOCK_EXPORT_CLASS void SFun_set_cgSetupRuntimeResources (SFun_BlockFunctionTable* pFcnTable, SFun_BlockFunction pfn_call_cgSetupRuntimeResources );
        SL_CORE_BLOCK_EXPORT_CLASS void SFun_set_cgInitialize     (SFun_BlockFunctionTable* pFcnTable, SFun_BlockFunction pfn_call_cgInitialize     );
        SL_CORE_BLOCK_EXPORT_CLASS void SFun_set_cgSystemInitialize     (SFun_BlockFunctionTable* pFcnTable, SFun_BlockFunction pfn_call_cgSystemInitialize     );       
        SL_CORE_BLOCK_EXPORT_CLASS void SFun_set_cgOutput         (SFun_BlockFunctionTable* pFcnTable, SFun_BlockFunction pfn_call_cgOutput         );
        SL_CORE_BLOCK_EXPORT_CLASS void SFun_set_cgUpdate         (SFun_BlockFunctionTable* pFcnTable, SFun_BlockFunction pfn_call_cgUpdate         );
        SL_CORE_BLOCK_EXPORT_CLASS void SFun_set_cgCleanupRuntimeResources       (SFun_BlockFunctionTable* pFcnTable, SFun_BlockFunction pfn_call_cgCleanupRuntimeResources       );
        SL_CORE_BLOCK_EXPORT_CLASS void SFun_set_cgTerminate      (SFun_BlockFunctionTable* pFcnTable, SFun_BlockFunction pfn_call_cgTerminate      );
    }

    class SFun_BlockFunctionTable_SmartPtr {
      public:
       
        explicit SFun_BlockFunctionTable_SmartPtr(SFun_BlockFunctionTable* aFcnTable)
            : fFcnTable(aFcnTable) {}
       
        ~SFun_BlockFunctionTable_SmartPtr() {
           
            deallocate();
        }

        SFun_BlockFunctionTable* get() const {
            return fFcnTable;
        }

      private:
       
        SFun_BlockFunctionTable_SmartPtr(SFun_BlockFunctionTable_SmartPtr& lvalue)
             : fFcnTable(lvalue.release())
        {}
      public:
       
        friend SFun_BlockFunctionTable_SmartPtr move(SFun_BlockFunctionTable_SmartPtr& lvalue)
        {
            return SFun_BlockFunctionTable_SmartPtr(lvalue);
        }

        class rvalue_proxy {
          private:
           
            SFun_BlockFunctionTable_SmartPtr& underlying;

            explicit rvalue_proxy(SFun_BlockFunctionTable_SmartPtr& p)
                : underlying(p)
            {}
           
            friend class SFun_BlockFunctionTable_SmartPtr;
          public:
           
            ~rvalue_proxy() {}
           
            SFun_BlockFunctionTable_SmartPtr& operator*() { return underlying; }
           
            SFun_BlockFunctionTable_SmartPtr* operator->() { return &underlying; }
        };

        operator rvalue_proxy() { return rvalue_proxy(*this); }

        SFun_BlockFunctionTable_SmartPtr(rvalue_proxy r)
            : fFcnTable(r->release())
        {
        }

      private:

        SFun_BlockFunctionTable* release() {
            SFun_BlockFunctionTable* tmp = fFcnTable;
            fFcnTable = nullptr;
            return tmp;
        }

        void deallocate() {
            if (fFcnTable) {
                SFun_delete_BlockFcnTable(fFcnTable);
                fFcnTable = nullptr;
            }
        }

        SFun_BlockFunctionTable& operator=(const SFun_BlockFunctionTable_SmartPtr&);

        SFun_BlockFunctionTable* fFcnTable;
    };

    template<typename DerivedBlock>
    SFun_BlockFunctionTable_SmartPtr build_BlockFunctionTable(DerivedBlock* = NULL)
    {
       
        SFun_BlockFunctionTable_SmartPtr pFcnTable(SFun_new_BlockFcnTable());
       
        SFun_set_cgStart          ( pFcnTable.get(), &call_cgStart           );
        SFun_set_cgSetupRuntimeResources ( pFcnTable.get(), &call_cgSetupRuntimeResources  );
        SFun_set_cgInitialize     ( pFcnTable.get(), &call_cgInitialize      );
        SFun_set_cgSystemInitialize     ( pFcnTable.get(), &call_cgSystemInitialize      );       
        SFun_set_cgOutput         ( pFcnTable.get(), &call_cgOutput          );
        SFun_set_cgUpdate         ( pFcnTable.get(), &call_cgUpdate          );
        SFun_set_cgCleanupRuntimeResources     ( pFcnTable.get(), &call_cgCleanupRuntimeResources      );
        SFun_set_cgTerminate      ( pFcnTable.get(), &call_cgTerminate       );
        return move(pFcnTable);
    }
}

#endif
