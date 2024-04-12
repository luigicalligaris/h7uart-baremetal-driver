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

#ifndef INC_H7UART_BARE_PRIV_H_
#define INC_H7UART_BARE_PRIV_H_

#include "h7uart_bare.h"
#include "h7uart_config.h"


enum {H7UART_UART_MUTEX_UNLOCKED = 0, H7UART_UART_MUTEX_LOCKED = 1};

typedef struct
{
  h7uart_uart_fsm_state_t fsm_state;

  void*    uart_base;

  uint32_t cr1_value;
  uint32_t cr2_value;
  uint32_t cr3_value;
  uint32_t brr_value;
  uint32_t gtpr_value;
  uint32_t rtor_value;
  uint32_t rqr_value;

  uint8_t  data_rx[H7UART_FIFO_DEPH];
  uint8_t  cont_rx;

  uint32_t  len_tx;
  uint32_t  cont_tx;
  uint8_t*  data_tx;

  uint32_t timestart;
  uint32_t timeout;

  void (*irq_callback)(uint32_t);
  void (*rx_callback) (uint8_t*,uint32_t);

} h7uart_driver_instance_state_t;


extern h7uart_uart_ret_code_t h7uart_uart_mutex_lock(h7uart_periph_t peripheral, uint32_t timeout);
extern h7uart_uart_ret_code_t h7uart_uart_mutex_lock_impl(h7uart_periph_t peripheral);
extern h7uart_uart_ret_code_t h7uart_uart_mutex_release(h7uart_periph_t peripheral);
extern h7uart_uart_ret_code_t h7uart_uart_mutex_release_fromISR(h7uart_periph_t peripheral);

#endif // INC_H7I2C_BARE_PRIV_H_
