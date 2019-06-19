

/* this ALWAYS GENERATED file contains the IIDs and CLSIDs */

/* link this file in with the server and any clients */


 /* File created by MIDL compiler version 8.00.0603 */
/* at Wed Jan 23 23:07:31 2019
 */
/* Compiler settings for win64\mwcomutil.idl:
    Oicf, W1, Zp8, env=Win64 (32b run), target_arch=IA64 8.00.0603 
    protocol : dce , ms_ext, c_ext, robust
    error checks: allocation ref bounds_check enum stub_data 
    VC __declspec() decoration level: 
         __declspec(uuid()), __declspec(selectany), __declspec(novtable)
         DECLSPEC_UUID(), MIDL_INTERFACE()
*/
/* @@MIDL_FILE_HEADING(  ) */

#pragma warning( disable: 4049 )  /* more than 64k source lines */


#ifdef __cplusplus
extern "C"{
#endif 


#include <rpc.h>
#include <rpcndr.h>

#ifdef _MIDL_USE_GUIDDEF_

#ifndef INITGUID
#define INITGUID
#include <guiddef.h>
#undef INITGUID
#else
#include <guiddef.h>
#endif

#define MIDL_DEFINE_GUID(type,name,l,w1,w2,b1,b2,b3,b4,b5,b6,b7,b8) \
        DEFINE_GUID(name,l,w1,w2,b1,b2,b3,b4,b5,b6,b7,b8)

#else // !_MIDL_USE_GUIDDEF_

#ifndef __IID_DEFINED__
#define __IID_DEFINED__

typedef struct _IID
{
    unsigned long x;
    unsigned short s1;
    unsigned short s2;
    unsigned char  c[8];
} IID;

#endif // __IID_DEFINED__

#ifndef CLSID_DEFINED
#define CLSID_DEFINED
typedef IID CLSID;
#endif // CLSID_DEFINED

#define MIDL_DEFINE_GUID(type,name,l,w1,w2,b1,b2,b3,b4,b5,b6,b7,b8) \
        const type name = {l,w1,w2,{b1,b2,b3,b4,b5,b6,b7,b8}}

#endif !_MIDL_USE_GUIDDEF_

MIDL_DEFINE_GUID(IID, IID_IMWUtil,0xC47EA90E,0x56D1,0x11d5,0xB1,0x59,0x00,0xD0,0xB7,0xBA,0x75,0x44);


MIDL_DEFINE_GUID(IID, LIBID_MWComUtil,0xEBAE4826,0x552A,0x4CFA,0x91,0x9C,0xE9,0xD2,0x01,0x55,0x1C,0xE2);


MIDL_DEFINE_GUID(CLSID, CLSID_MWField,0x2CA8734F,0x907D,0x4C5E,0xA3,0xBA,0x31,0x92,0x5E,0xE0,0xD0,0x2B);


MIDL_DEFINE_GUID(CLSID, CLSID_MWStruct,0x453AB52D,0x43A9,0x4D5E,0x91,0x44,0xC8,0xFF,0xB3,0xEF,0xB8,0x8F);


MIDL_DEFINE_GUID(CLSID, CLSID_MWComplex,0x355A0CC6,0x539E,0x45C8,0x89,0xE0,0xA4,0x99,0x25,0x4E,0xD6,0xC9);


MIDL_DEFINE_GUID(CLSID, CLSID_MWSparse,0x61089CB7,0x9EF4,0x4E27,0x9F,0x5E,0x07,0x50,0x8A,0xC0,0x0A,0x86);


MIDL_DEFINE_GUID(CLSID, CLSID_MWArg,0xD538E18E,0x1516,0x4545,0x9F,0xB5,0x1F,0x2D,0x54,0x5B,0x93,0xBB);


MIDL_DEFINE_GUID(CLSID, CLSID_MWArrayFormatFlags,0x47713823,0x1C52,0x4393,0x9C,0x48,0x16,0xA9,0x99,0x63,0x7A,0xBA);


MIDL_DEFINE_GUID(CLSID, CLSID_MWDataConversionFlags,0xD02CCE18,0x42A5,0x451F,0x94,0x49,0x4C,0xCD,0x47,0xCC,0xB9,0xD7);


MIDL_DEFINE_GUID(CLSID, CLSID_MWUtil,0xD7E8733F,0x9C83,0x4616,0x87,0x02,0x4E,0x09,0x81,0x2E,0x46,0xBB);


MIDL_DEFINE_GUID(CLSID, CLSID_MWFlags,0x13F340ED,0xE414,0x45EC,0xA1,0xE2,0xEB,0x18,0x09,0x81,0xAB,0x45);

#undef MIDL_DEFINE_GUID

#ifdef __cplusplus
}
#endif



