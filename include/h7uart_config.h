/*********************************************************************************************/
/* STM32H7 UART bare-metal driver                                                            */
/* Copyright (c) 2022-2023, Luigi Calligaris, Carlos R. Dell'Aquila                          */
/* All rights reserved.                                                                      */
/*                                                                                           */
/* This software is distributed under the BSD (3-clause) license, which is reproduced below: */
/* ----------------------------------------------------------------------------------------- */
/*                                                                                           */
/* Redistribution and use in source and binary forms, with or without modification,          */
/* are permitted provided that the following conditions are met:                             */
/*                                                                                           */
/* * Redistributions of source code must retain the above copyright notice, this             */
/*   list of conditions and the following disclaimer.                                        */
/*                                                                                           */
/* * Redistributions in binary form must reproduce the above copyright notice, this          */
/*   list of conditions and the following disclaimer in the documentation and/or             */
/*   other materials provided with the distribution.                                         */
/*                                                                                           */
/* * Neither the name of SPRACE nor the one of UNESP nor the names of its                    */
/*   contributors may be used to endorse or promote products derived from                    */
/*   this software without specific prior written permission.                                */
/*                                                                                           */
/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND           */
/* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED             */
/* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE                    */
/* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR          */
/* ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES            */
/* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;              */
/* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON            */
/* ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT                   */
/* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS             */
/* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                              */
/*                                                                                           */
/*********************************************************************************************/

#ifndef INC_H7UART_CONFIG_H_
#define INC_H7UART_CONFIG_H_

#include "main.h"


// Declare the following macros in main.h file to put the peripheral under the responsibility of this driver.
// You shall mind about collisions with the STM32Cube driver (if you use this driver,
// the peripheral should be unconfigured in the IOC file).


#ifndef H7UART_PERIPH_ENABLE_USART1
#define H7UART_PERIPH_ENABLE_USART1  1
#endif

#ifndef H7UART_PERIPH_ENABLE_USART2
#define H7UART_PERIPH_ENABLE_USART2  0
#endif

#ifndef H7UART_PERIPH_ENABLE_USART3
#define H7UART_PERIPH_ENABLE_USART3  0
#endif

#ifndef H7UART_PERIPH_ENABLE_UART4
#define H7UART_PERIPH_ENABLE_UART4   0
#endif

#ifndef H7UART_PERIPH_ENABLE_UART5
#define H7UART_PERIPH_ENABLE_UART5   0
#endif

#ifndef H7UART_PERIPH_ENABLE_USART6
#define H7UART_PERIPH_ENABLE_USART6  0
#endif

#ifndef H7UART_PERIPH_ENABLE_UART7
#define H7UART_PERIPH_ENABLE_UART7   0
#endif

#ifndef H7UART_PERIPH_ENABLE_UART8
#define H7UART_PERIPH_ENABLE_UART8   0
#endif

#ifndef H7UART_PERIPH_ENABLE_LPUART1
#define H7UART_PERIPH_ENABLE_LPUART1 0
#endif

// Peripheral clock settings important to calculate the Baud-Rate
#ifndef USART_CLOCK_KER_USART_1
#define USART_CLOCK_KER_USART_1 100000000UL // 100 MHz
#endif

#ifndef USART_CLOCK_KER_USART_2345678
#define USART_CLOCK_KER_USART_2345678 100000000UL // 100 MHz
#endif

#ifndef USART_CLOCK_KER_LPUART_1
#define USART_CLOCK_KER_LPUART_1 100000000UL // 100 MHz
#endif

// Do you want to use the FreeRTOS-compatible function implementations?
#define H7UART_USE_FREERTOS_IMPL 0

// Specify the IRQ priorities for the various UART devices
#define H7UART_IRQ_UART4_PRI   15
#define H7UART_IRQ_UART5_PRI   15
#define H7UART_IRQ_UART7_PRI   15
#define H7UART_IRQ_UART8_PRI   15
#define H7UART_IRQ_USART1_PRI  7
#define H7UART_IRQ_USART2_PRI  15
#define H7UART_IRQ_USART3_PRI  15
#define H7UART_IRQ_USART6_PRI  7
#define H7UART_IRQ_LPUART1_PRI 7

#define H7UART_IRQ_UART4_SUBPRI   0
#define H7UART_IRQ_UART5_SUBPRI   1
#define H7UART_IRQ_UART7_SUBPRI   2
#define H7UART_IRQ_UART8_SUBPRI   3
#define H7UART_IRQ_USART1_SUBPRI  4
#define H7UART_IRQ_USART2_SUBPRI  5
#define H7UART_IRQ_USART3_SUBPRI  6
#define H7UART_IRQ_USART6_SUBPRI  7
#define H7UART_IRQ_LPUART1_SUBPRI 8

// Do not edit this logic if you don't understand it
#if H7UART_PERIPH_ENABLE_UART4 == 1 || H7UART_PERIPH_ENABLE_UART5 == 1 || H7UART_PERIPH_ENABLE_UART7 == 1 || H7UART_PERIPH_ENABLE_UART8 == 1 || H7UART_PERIPH_ENABLE_USART1 == 1 || H7UART_PERIPH_ENABLE_USART2 == 1 || H7UART_PERIPH_ENABLE_USART3 == 1 || H7UART_PERIPH_ENABLE_USART6 == 1 || H7UART_PERIPH_ENABLE_LPUART1 == 1
#define H7UART_PERIPH_ENABLE_ANY 1
#else
#define H7UART_PERIPH_ENABLE_ANY 0
#endif


#endif // INC_H7UART_CONFIG_H_
