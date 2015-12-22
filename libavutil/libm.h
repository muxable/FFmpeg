/*
 * erf function: Copyright (c) 2006 John Maddock
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */

/**
 * @file
 * Replacements for frequently missing libm functions
 */

#ifndef AVUTIL_LIBM_H
#define AVUTIL_LIBM_H

#include <math.h>
#include "config.h"
#include "attributes.h"
#include "intfloat.h"
#include "mathematics.h"

#if HAVE_MIPSFPU && HAVE_INLINE_ASM
#include "libavutil/mips/libm_mips.h"
#endif /* HAVE_MIPSFPU && HAVE_INLINE_ASM*/

#if !HAVE_ATANF
#undef atanf
#define atanf(x) ((float)atan(x))
#endif

#if !HAVE_ATAN2F
#undef atan2f
#define atan2f(y, x) ((float)atan2(y, x))
#endif

#if !HAVE_POWF
#undef powf
#define powf(x, y) ((float)pow(x, y))
#endif

#if !HAVE_CBRT
static av_always_inline double cbrt(double x)
{
    return x < 0 ? -pow(-x, 1.0 / 3.0) : pow(x, 1.0 / 3.0);
}
#endif

#if !HAVE_CBRTF
static av_always_inline float cbrtf(float x)
{
    return x < 0 ? -powf(-x, 1.0 / 3.0) : powf(x, 1.0 / 3.0);
}
#endif

#if !HAVE_COPYSIGN
static av_always_inline double copysign(double x, double y)
{
    uint64_t vx = av_double2int(x);
    uint64_t vy = av_double2int(y);
    return av_int2double((vx & UINT64_C(0x7fffffffffffffff)) | (vy & UINT64_C(0x8000000000000000)));
}
#endif

#if !HAVE_COSF
#undef cosf
#define cosf(x) ((float)cos(x))
#endif

#if !HAVE_ERF
static inline double ff_eval_poly(const double *coeff, int size, double x) {
    double sum = coeff[size-1];
    int i;
    for (i = size-2; i >= 0; --i) {
        sum *= x;
        sum += coeff[i];
    }
    return sum;
}

/**
 * erf function
 * Algorithm taken from the Boost project, source:
 * http://www.boost.org/doc/libs/1_46_1/boost/math/special_functions/erf.hpp
 * Use, modification and distribution are subject to the
 * Boost Software License, Version 1.0 (see notice below).
 * Boost Software License - Version 1.0 - August 17th, 2003
Permission is hereby granted, free of charge, to any person or organization
obtaining a copy of the software and accompanying documentation covered by
this license (the "Software") to use, reproduce, display, distribute,
execute, and transmit the Software, and to prepare derivative works of the
Software, and to permit third-parties to whom the Software is furnished to
do so, all subject to the following:

The copyright notices in the Software and this entire statement, including
the above license grant, this restriction and the following disclaimer,
must be included in all copies of the Software, in whole or in part, and
all derivative works of the Software, unless such copies or derivative
works are solely in the form of machine-executable object code generated by
a source language processor.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT
SHALL THE COPYRIGHT HOLDERS OR ANYONE DISTRIBUTING THE SOFTWARE BE LIABLE
FOR ANY DAMAGES OR OTHER LIABILITY, WHETHER IN CONTRACT, TORT OR OTHERWISE,
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
DEALINGS IN THE SOFTWARE.
 */
static inline double erf(double z)
{
#ifndef FF_ARRAY_ELEMS
#define FF_ARRAY_ELEMS(a) (sizeof(a) / sizeof((a)[0]))
#endif
    double result;

    /* handle the symmetry: erf(-x) = -erf(x) */
    if (z < 0)
        return -erf(-z);

    /* branch based on range of z, and pick appropriate approximation */
    if (z == 0)
        return 0;
    else if (z < 1e-10)
        return z * 1.125 + z * 0.003379167095512573896158903121545171688;
    else if (z < 0.5) {
        // Maximum Deviation Found:                     1.561e-17
        // Expected Error Term:                         1.561e-17
        // Maximum Relative Change in Control Points:   1.155e-04
        // Max Error found at double precision =        2.961182e-17

        static const double y = 1.044948577880859375;
        static const double p[] = {
            0.0834305892146531832907,
            -0.338165134459360935041,
            -0.0509990735146777432841,
            -0.00772758345802133288487,
            -0.000322780120964605683831,
        };
        static const double q[] = {
            1,
            0.455004033050794024546,
            0.0875222600142252549554,
            0.00858571925074406212772,
            0.000370900071787748000569,
        };
        double zz = z * z;
        return z * (y + ff_eval_poly(p, FF_ARRAY_ELEMS(p), zz) / ff_eval_poly(q, FF_ARRAY_ELEMS(q), zz));
    }
    /* here onwards compute erfc */
    else if (z < 1.5) {
        // Maximum Deviation Found:                     3.702e-17
        // Expected Error Term:                         3.702e-17
        // Maximum Relative Change in Control Points:   2.845e-04
        // Max Error found at double precision =        4.841816e-17
        static const double y = 0.405935764312744140625;
        static const double p[] = {
            -0.098090592216281240205,
            0.178114665841120341155,
            0.191003695796775433986,
            0.0888900368967884466578,
            0.0195049001251218801359,
            0.00180424538297014223957,
        };
        static const double q[] = {
            1,
            1.84759070983002217845,
            1.42628004845511324508,
            0.578052804889902404909,
            0.12385097467900864233,
            0.0113385233577001411017,
            0.337511472483094676155e-5,
        };
        result = y + ff_eval_poly(p, FF_ARRAY_ELEMS(p), z - 0.5) / ff_eval_poly(q, FF_ARRAY_ELEMS(q), z - 0.5);
        result *= exp(-z * z) / z;
        return 1 - result;
    }
    else if (z < 2.5) {
        // Max Error found at double precision =        6.599585e-18
        // Maximum Deviation Found:                     3.909e-18
        // Expected Error Term:                         3.909e-18
        // Maximum Relative Change in Control Points:   9.886e-05
        static const double y = 0.50672817230224609375;
        static const double p[] = {
            -0.0243500476207698441272,
            0.0386540375035707201728,
            0.04394818964209516296,
            0.0175679436311802092299,
            0.00323962406290842133584,
            0.000235839115596880717416,
        };
        static const double q[] = {
            1,
            1.53991494948552447182,
            0.982403709157920235114,
            0.325732924782444448493,
            0.0563921837420478160373,
            0.00410369723978904575884,
        };
        result = y + ff_eval_poly(p, FF_ARRAY_ELEMS(p), z - 1.5) / ff_eval_poly(q, FF_ARRAY_ELEMS(q), z - 1.5);
        result *= exp(-z * z) / z;
        return 1 - result;
    }
    else if (z < 4.5) {
        // Maximum Deviation Found:                     1.512e-17
        // Expected Error Term:                         1.512e-17
        // Maximum Relative Change in Control Points:   2.222e-04
        // Max Error found at double precision =        2.062515e-17
        static const double y = 0.5405750274658203125;
        static const double p[] = {
            0.00295276716530971662634,
            0.0137384425896355332126,
            0.00840807615555585383007,
            0.00212825620914618649141,
            0.000250269961544794627958,
            0.113212406648847561139e-4,
        };
        static const double q[] = {
            1,
            1.04217814166938418171,
            0.442597659481563127003,
            0.0958492726301061423444,
            0.0105982906484876531489,
            0.000479411269521714493907,
        };
        result = y + ff_eval_poly(p, FF_ARRAY_ELEMS(p), z - 3.5) / ff_eval_poly(q, FF_ARRAY_ELEMS(q), z - 3.5);
        result *= exp(-z * z) / z;
        return 1 - result;
    }
    /* differ from Boost here, the claim of underflow of erfc(x) past 5.8 is
     * slightly incorrect, change to 5.92
     * (really somewhere between 5.9125 and 5.925 is when it saturates) */
    else if (z < 5.92) {
        // Max Error found at double precision =        2.997958e-17
        // Maximum Deviation Found:                     2.860e-17
        // Expected Error Term:                         2.859e-17
        // Maximum Relative Change in Control Points:   1.357e-05
        static const double y = 0.5579090118408203125;
        static const double p[] = {
            0.00628057170626964891937,
            0.0175389834052493308818,
            -0.212652252872804219852,
            -0.687717681153649930619,
            -2.5518551727311523996,
            -3.22729451764143718517,
            -2.8175401114513378771,
        };
        static const double q[] = {
            1,
            2.79257750980575282228,
            11.0567237927800161565,
            15.930646027911794143,
            22.9367376522880577224,
            13.5064170191802889145,
            5.48409182238641741584,
        };
        result = y + ff_eval_poly(p, FF_ARRAY_ELEMS(p), 1 / z) / ff_eval_poly(q, FF_ARRAY_ELEMS(q), 1 / z);
        result *= exp(-z * z) / z;
        return 1 - result;
    }
    /* handle the nan case, but don't use isnan for max portability */
    else if (z != z)
        return z;
    /* finally return saturated result */
    else
        return 1;
}
#endif

#if !HAVE_EXPF
#undef expf
#define expf(x) ((float)exp(x))
#endif

#if !HAVE_EXP2
#undef exp2
#define exp2(x) exp((x) * 0.693147180559945)
#endif /* HAVE_EXP2 */

#if !HAVE_EXP2F
#undef exp2f
#define exp2f(x) ((float)exp2(x))
#endif /* HAVE_EXP2F */

/* Somewhat inaccurate fallbacks, relative error ~ 1e-13 concentrated on very
small and very large values. For perfection accuracy-wise, should use pow.
Speed benefits (>2x average, with no super slow paths) deemed to be worth the
accuracy tradeoff */
#if !HAVE_EXP10
static av_always_inline double exp10(double x)
{
    return exp2(M_LOG2_10 * x);
}
#endif /* HAVE_EXP10 */

#if !HAVE_EXP10F
static av_always_inline float exp10f(float x)
{
    return exp2f(M_LOG2_10 * x);
}
#endif /* HAVE_EXP10F */

#if !HAVE_ISINF
#undef isinf
/* Note: these do not follow the BSD/Apple/GNU convention of returning -1 for
-Inf, +1 for Inf, 0 otherwise, but merely follow the POSIX/ISO mandated spec of
returning a non-zero value for +/-Inf, 0 otherwise. */
static av_always_inline av_const int avpriv_isinff(float x)
{
    uint32_t v = av_float2int(x);
    if ((v & 0x7f800000) != 0x7f800000)
        return 0;
    return !(v & 0x007fffff);
}

static av_always_inline av_const int avpriv_isinf(double x)
{
    uint64_t v = av_double2int(x);
    if ((v & 0x7ff0000000000000) != 0x7ff0000000000000)
        return 0;
    return !(v & 0x000fffffffffffff);
}

#define isinf(x)                  \
    (sizeof(x) == sizeof(float)   \
        ? avpriv_isinff(x)        \
        : avpriv_isinf(x))
#endif /* HAVE_ISINF */

#if !HAVE_ISNAN
static av_always_inline av_const int avpriv_isnanf(float x)
{
    uint32_t v = av_float2int(x);
    if ((v & 0x7f800000) != 0x7f800000)
        return 0;
    return v & 0x007fffff;
}

static av_always_inline av_const int avpriv_isnan(double x)
{
    uint64_t v = av_double2int(x);
    if ((v & 0x7ff0000000000000) != 0x7ff0000000000000)
        return 0;
    return (v & 0x000fffffffffffff) && 1;
}

#define isnan(x)                  \
    (sizeof(x) == sizeof(float)   \
        ? avpriv_isnanf(x)        \
        : avpriv_isnan(x))
#endif /* HAVE_ISNAN */

#if !HAVE_HYPOT
#undef hypot
static inline av_const double hypot(double x, double y)
{
    double ret, temp;
    x = fabs(x);
    y = fabs(y);

    if (isinf(x) || isinf(y))
        return av_int2double(0x7ff0000000000000);
    if (x == 0 || y == 0)
        return x + y;
    if (x < y) {
        temp = x;
        x = y;
        y = temp;
    }

    y = y/x;
    return x*sqrt(1 + y*y);
}
#endif /* HAVE_HYPOT */

#if !HAVE_LDEXPF
#undef ldexpf
#define ldexpf(x, exp) ((float)ldexp(x, exp))
#endif

#if !HAVE_LLRINT
#undef llrint
#define llrint(x) ((long long)rint(x))
#endif /* HAVE_LLRINT */

#if !HAVE_LLRINTF
#undef llrintf
#define llrintf(x) ((long long)rint(x))
#endif /* HAVE_LLRINT */

#if !HAVE_LOG2
#undef log2
#define log2(x) (log(x) * 1.44269504088896340736)
#endif /* HAVE_LOG2 */

#if !HAVE_LOG2F
#undef log2f
#define log2f(x) ((float)log2(x))
#endif /* HAVE_LOG2F */

#if !HAVE_LOG10F
#undef log10f
#define log10f(x) ((float)log10(x))
#endif

#if !HAVE_SINF
#undef sinf
#define sinf(x) ((float)sin(x))
#endif

#if !HAVE_RINT
static inline double rint(double x)
{
    return x >= 0 ? floor(x + 0.5) : ceil(x - 0.5);
}
#endif /* HAVE_RINT */

#if !HAVE_LRINT
static av_always_inline av_const long int lrint(double x)
{
    return rint(x);
}
#endif /* HAVE_LRINT */

#if !HAVE_LRINTF
static av_always_inline av_const long int lrintf(float x)
{
    return (int)(rint(x));
}
#endif /* HAVE_LRINTF */

#if !HAVE_ROUND
static av_always_inline av_const double round(double x)
{
    return (x > 0) ? floor(x + 0.5) : ceil(x - 0.5);
}
#endif /* HAVE_ROUND */

#if !HAVE_ROUNDF
static av_always_inline av_const float roundf(float x)
{
    return (x > 0) ? floor(x + 0.5) : ceil(x - 0.5);
}
#endif /* HAVE_ROUNDF */

#if !HAVE_TRUNC
static av_always_inline av_const double trunc(double x)
{
    return (x > 0) ? floor(x) : ceil(x);
}
#endif /* HAVE_TRUNC */

#if !HAVE_TRUNCF
static av_always_inline av_const float truncf(float x)
{
    return (x > 0) ? floor(x) : ceil(x);
}
#endif /* HAVE_TRUNCF */

#endif /* AVUTIL_LIBM_H */
