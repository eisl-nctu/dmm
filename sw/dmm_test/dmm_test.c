// =============================================================================
//  Program : dmm_test.c
//  Author  : Chun-Jen Tsai
//  Date    : Dec/09/2019
// -----------------------------------------------------------------------------
//  Description:
//  This is the minimal time library for aquila.
// -----------------------------------------------------------------------------
//  Revision information:
//
//  None.
// -----------------------------------------------------------------------------
//  License information:
//
//  This software is released under the BSD-3-Clause Licence,
//  see https://opensource.org/licenses/BSD-3-Clause for details.
//  In the following license statements, "software" refers to the
//  "source code" of the complete hardware/software system.
//
//  Copyright 2019,
//                    Embedded Intelligent Systems Lab (EISL)
//                    Deparment of Computer Science
//                    National Chiao Tung Uniersity
//                    Hsinchu, Taiwan.
//
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions are met:
//
//  1. Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//  2. Redistributions in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//  3. Neither the name of the copyright holder nor the names of its contributors
//     may be used to endorse or promote products derived from this software
//     without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
//  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
// =============================================================================

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <mallocr2.h>
#include "uart.h"
#include <wctype.h>
#include <stddef.h>
#include "workload.h"

void *dmm_test_malloc(int nwords);
void dmm_test_free(int pointer);
void my_sleep(int stime);

#define __ASM_STR(x)  #x
#define csr_read(csr) ({ \
            register unsigned long __v; \
            __asm__ __volatile__ ("csrr %0, " __ASM_STR(csr): "=r" (__v) :: "memory"); \
            __v; })

#define SW_DMM 1

clock_t cycles(void)
{
    size_t ticks;
    size_t cycles, cyclesh;
    long long cycles64;

    asm volatile ("csrrs %0, mcycle, x0":"=r" (cycles));
    asm volatile ("csrrs %0, mcycleh, x0":"=r" (cyclesh));
    cycles64 = ((long long) cyclesh << 32) + cycles;
    ticks = (size_t) ((cycles64) & 0xFFFFFFFFL);

    return ticks;
}

int main(void)
{
    clock_t clk1, clk2, clk3, clk4, clk_overhead;
    clock_t malloc_time = 0, free_time = 0;
#if SW_DMM
    for (int i = 0; i < seq_num; i++)
    {
        clk1 = cycles();
        clk2 = cycles();
        clk_overhead = clk2 - clk1;
        if (mem_ops[i] == 1)
        {
            clk3 = cycles();
            allocate_array[alloc_idx[i]] = (long) mALLOc(alloc_size[i]);
            clk4 = cycles();
            malloc_time += ((clk4 - clk3) - clk_overhead);
        }
        else
        {
            clk3 = cycles();
            fREe((void *) allocate_array[alloc_idx[i]]);
            clk4 = cycles();
            free_time += ((clk4 - clk3) - clk_overhead);
        }
        my_sleep(1); // Busy loop used to simulate data processing workload.
    }
    printf("SW - malloc: %uld, free: %uld\n\n", malloc_time, free_time);
#else
    malloc_time = 0, free_time = 0;
    clk1 = cycles();
    clk2 = cycles();
    clk_overhead = clk2 - clk1;
    for (int i = 0; i < seq_num; i++)
    {
        clk1 = cycles();
        clk2 = cycles();
        clk_overhead = clk2 - clk1;
        if (mem_ops[i] == 1)
        {
            clk3 = cycles();
            allocate_array[alloc_idx[i]] = (long) dmm_test_malloc(alloc_size[i]);
            clk4 = cycles();
            malloc_time += ((clk4 - clk3) - clk_overhead);
        }
        else
        {
            clk3 = cycles();
            dmm_test_free(allocate_array[alloc_idx[i]]);
            clk4 = cycles();
            free_time += ((clk4 - clk3) - clk_overhead);
        }
        my_sleep(1); // Busy loop used to simulate data processing workload.
    }
    printf("HW - malloc: %uld, free: %uld\n\n", malloc_time, free_time);

    // Now, read the user CSR registers that shows HW background
    // execution time.
    my_sleep(100);
    int a = csr_read(0xE09);
    int b = csr_read(0xE0A);
    printf("HW bkg - malloc: %uld, free(): %uld\n\n", a, b);
#endif

    return 0;
}

void *dmm_test_malloc(int nwords)
{
    int a, b;

    asm volatile ("add %0, t1, zero" : "=r" (b)); // saving t1.
    asm volatile ("add t2, %1, zero":"=r" (nwords):"r"(nwords));
    //0000 0000 0000 00111 000 00110 1101011 
    //     immi       rs1  fun  rd     op
    //0000 0000 0000 0011 1000 0011 0110 1011
    asm volatile (".byte 0x6b, 0x83, 0x03, 0x00");  //malloc t1(x6) t2+0
    asm volatile ("addi %0, t1, 0":"=r" (a));
    asm volatile ("add t1, %1, zero" : "=r"(b) : "r"(b)); // restore t1.

    return (void *) a;
}

void dmm_test_free(int pointer)
{
    int p;
    asm volatile ("add t2, %1, zero":"=r" (p):"r"(pointer));
    //0000 0000 0000 00111 010 00000 1101011
    //     immi       rs1  fun  rd     op
    //0000 0000 0000 0011 1010 0000 0110 1011
    asm volatile (".byte 0x6b, 0xa0, 0x03, 0x00");  //free
}

void my_sleep(int stime)
{
    int clk1, clk2;

    clk1 = cycles();
    do
    {
        clk2 = cycles();
    }
    while (clk2 - clk1 < stime);
}

