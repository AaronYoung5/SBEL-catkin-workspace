// =============================================================================
// PROJECT SYNRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#ifndef SYNPLATFORM_H
#define SYNPLATFORM_H

// Functionality for API import/export symbols,
// in a platform independent way.
//
// When building the DLL, the SYN_EXPORTS symbol must be defined.
//
// Each exported function / class in Chrono::Engine
// will use the 'SynApi' macro in headers (for non-MS compilers,
// this has no effect because all symbols will be exported).

#if ((defined _WIN32) || (defined(__MINGW32__) || defined(__CYGWIN__))) && defined(_DLL)
#if !defined(SYN_DLL) && !defined(SYN_STATIC)
#define SYN_DLL
#endif
#endif

#if ((defined _WIN32) || (defined(__MINGW32__) || defined(__CYGWIN__))) && defined(SYN_DLL)
#define SynApiEXPORT __declspec(dllexport)
#define SynApiIMPORT __declspec(dllimport)
#else
#define SynApiEXPORT
#define SynApiIMPORT
#endif

// Define a deprecated macro which generates a warning at compile time.
// Usage:
//   For typedef:         typedef SYN_DEPRECATED int test1;
//   For classes/structs: class SYN_DEPRECATED test2 { ... };
//   For methods:         class test3 { SYN_DEPRECATED virtual void foo() {} };
//   For functions:       template<class T> SYN_DEPRECATED void test4() {}
//
// When building the Chrono libraries, define SYN_IGNORE_DEPRECATED to stop issuing these warnings.

#if defined(SYN_IGNORE_DEPRECATED)
#define SYN_DEPRECATED(msg)
#else
#if __cplusplus >= 201402L
#define SYN_DEPRECATED(msg) [[deprecated(msg)]]
#elif defined(__GNUC__)
#define SYN_DEPRECATED(msg) __attribute__((deprecated(msg)))
#elif defined(_MSC_VER)
#define SYN_DEPRECATED(msg) __declspec(deprecated(msg))
#else
#define SYN_DEPRECATED(msg)
#endif
#endif



// Disable the C4251 warning under MSVC, that happens when using
// templated classes in data members of other exported classes.

#pragma warning(disable : 4251)

#endif
