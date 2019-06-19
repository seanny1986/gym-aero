/* Copyright 2008-2017 The MathWorks, Inc. */

#ifdef SUPPORTS_PRAGMA_ONCE
# pragma once
#endif

#ifndef __slMessagesSfcnCGBridge_hpp__
#define __slMessagesSfcnCGBridge_hpp__

#include "simulink_spec.h"
#include "simstruct/simstruc.h"

namespace RTWCG
{
    class CGIR_Block_IF;
}
namespace CG
{
    class CoreConstructionFacade;
    class Var;
    class Expr;
    class Region;
}

namespace slmsgcg
{
   
    SIMULINK_EXPORT_EXTERN_C
        CG::Region* cgCreateAndSendMessage(SimStruct *S,
                                           CG::CoreConstructionFacade *f,
                                           CG::Var *u,
                                           int queueType,
                                           int queueId,
                                           CG::Var* localQId);

    SIMULINK_EXPORT_EXTERN_C
        CG::Expr* cgCreateMessage(SimStruct* S,
                                  CG::CoreConstructionFacade* f,
                                  CG::Var* u,
                                  int queueType,
                                  int id,
                                  CG::Var* localQId);

    SIMULINK_EXPORT_EXTERN_C
        CG::Region* cgSendMessage(SimStruct* S,
                                  CG::CoreConstructionFacade* f,
                                  CG::Var* msgVar,
                                  int queueType,
                                  int prtId,
                                  CG::Var* localQId);

    SIMULINK_EXPORT_EXTERN_C
        CG::Expr* cgPopMessage(SimStruct* S,
                               CG::CoreConstructionFacade* f,
                               int queueType,
                               int portId,
                               CG::Var* localQId);

    SIMULINK_EXPORT_EXTERN_C
        CG::Expr* cgGetMessageDataPtr(SimStruct* S,
                                      CG::CoreConstructionFacade* f,
                                      int queueType,
                                      int portId,
                                      CG::Var* msgVar);

    SIMULINK_EXPORT_EXTERN_C
        CG::Expr* cgGetNumMessages(SimStruct* S,
                                   CG::CoreConstructionFacade* f,
                                   int queueType,
                                   int portId,
                                   CG::Var* localQId);

    SIMULINK_EXPORT_EXTERN_C
        CG::Expr* cgDestroyMessage(SimStruct *S,
                                   CG::CoreConstructionFacade* f,
                                   int queueType, int portId,
                                   CG::Var* msgVar,
                                   bool isFromInput = true);

    SIMULINK_EXPORT_EXTERN_C
        CG::Region* cgTargetQueueSend(CG::CoreConstructionFacade& ccf,
                                      SimStruct* s, CG::Var* msgDataVar,
                                      int queueType, int portId, CG::Var* localQVar, CG::Var* msgQueueOverflowVar);

    SIMULINK_EXPORT_EXTERN_C
        CG::Region* cgTargetQueueReceive(CG::CoreConstructionFacade& ccf,
                                         CG::Var* successVar,
                                         SimStruct* s, CG::Var* msgHandleVar, CG::Var* msgDataVar,
                                         CG::Var* payloadVar,
                                         int queueType, int portId, CG::Var* localQVar, CG::Var* msgQueueOverflowVar);

    SIMULINK_EXPORT_EXTERN_C
        CG::Region* cgTargetQueueDiscard(CG::CoreConstructionFacade& ccf,
                                         SimStruct* s, CG::Var* msgHandleVar,
                                         int queueType, int portId);

    SIMULINK_EXPORT_EXTERN_C
        bool cgIsTargetInputQueueExternal(SimStruct* S, int queueType, int portId);

    SIMULINK_EXPORT_EXTERN_C
        bool cgIsTargetOutputQueueExternal(SimStruct* S, int queueType, int portId);

}

#endif
