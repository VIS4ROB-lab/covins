/*
 Copyright (C) 2011  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger, Simon Lynen and Margarita Chli.

 Copyright (C) 2013  The Autonomous Systems Lab, ETH Zurich,
 Stefan Leutenegger and Simon Lynen.

 BRISK - Binary Robust Invariant Scalable Keypoints
 Reference implementation of
 [1] Stefan Leutenegger,Margarita Chli and Roland Siegwart, BRISK:
 Binary Robust Invariant Scalable Keypoints, in Proceedings of
 the IEEE International Conference on Computer Vision (ICCV2011).

 This file is part of BRISK.

 All rights reserved.

 Redistribution and use in source and binary forms, with or without
 modification, are permitted provided that the following conditions are met:
 * Redistributions of source code must retain the above copyright
 notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
 notice, this list of conditions and the following disclaimer in the
 documentation and/or other materials provided with the distribution.
 * Neither the name of the <organization> nor the
 names of its contributors may be used to endorse or promote products
 derived from this software without specific prior written permission.

 THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef INTERNAL_MACROS_H_
#define INTERNAL_MACROS_H_

#ifndef CV_EXPORTS
#define CV_EXPORTS
#endif  // CV_EXPORTS

#ifndef M_PI
#define M_PI 3.141592653589793
#endif

#define USE_SIMPLE_POINT_WITH_SCORE

#include <stdint.h>

namespace rdtsc {
namespace timing {
class DummyTimer;
}  // namespace timing
}  // namespace rdtsc

namespace brisk {
typedef rdtsc::timing::DummyTimer DebugTimer;
// This is needed to avoid aliasing issues with the __m128i data type:
#ifdef __GNUC__
typedef unsigned char __attribute__ ((__may_alias__)) UCHAR_ALIAS;
typedef uint16_t __attribute__ ((__may_alias__)) UINT16_ALIAS;
typedef uint32_t __attribute__ ((__may_alias__)) UINT32_ALIAS;
typedef uint64_t __attribute__ ((__may_alias__)) UINT64_ALIAS;
typedef int __attribute__ ((__may_alias__)) INT32_ALIAS;
typedef uint8_t __attribute__ ((__may_alias__)) U_INT8T_ALIAS;
#endif
#ifdef _MSC_VER
// TODO(lestefan): Find the equivalent to may_alias.
#define UCHAR_ALIAS unsigned char  // __declspec(noalias)
#define UINT32_ALIAS unsigned int  // __declspec(noalias)
#define __inline__ __forceinline
#endif
}  // namespace brisk
#endif  // INTERNAL_MACROS_H_
