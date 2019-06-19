// Copyright 2007-2010 The MathWorks, Inc.

// master include file for S-function API in rtwcg.dll

#if defined(_MSC_VER) || __GNUC__ > 3 || (__GNUC__ == 3 && __GNUC_MINOR__ > 3)
#pragma once
#endif

#ifndef SIMULINK_SFUN_RTWCG_HPP
#define SIMULINK_SFUN_RTWCG_HPP

#if defined(_MSC_VER) && _MSC_VER < 1300
#error When using Visual C++, this API requires version 7.0, also known as Visual Studio.NET 2002, or later.
#endif

#include "cgir_construct/sfun/rtw_api.hpp"
#include "SimulinkBlock/sfun/block.hpp"
#include "SimulinkBlock/sfun/block_c_api.hpp"
#include "SimulinkBlock/sfun/blockfunction.hpp"
#include "SimulinkBlock/sfun/construct.hpp"

#endif //SIMULINK_SFUN_RTWCG_HPP
