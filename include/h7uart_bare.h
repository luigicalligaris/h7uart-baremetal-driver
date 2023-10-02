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

#ifndef INC_H7UART_BARE_H_
#define INC_H7UART_BARE_H_

#include <stdint.h>

#include "h7uart_config.h"

// UART FIFO FEATURE
#define H7UART_FIFO_DEPH 16

// H7UART Driver data type definitions

typedef enum
{
  H7UART_UART4,
  H7UART_UART5,
  H7UART_UART7,
  H7UART_UART8,
  H7UART_USART1,
  H7UART_USART2,
  H7UART_USART3,
  H7UART_USART6,
  H7UART_LPUART1
} h7uart_periph_t;


typedef enum
{
  H7UART_FSM_STATE_UNINITIALIZED,
  H7UART_FSM_STATE_IDLE,
  H7UART_FSM_STATE_SETUP_TRANSFER,
  H7UART_FSM_STATE_TRASFERING,
  H7UART_FSM_STATE_DEINITIALIZING,
  H7UART_FSM_STATE_ERROR_ABRE,   // Auto baud-rate error
  H7UART_FSM_STATE_ERROR_UDR,    // SPI slave underrun error flag
  H7UART_FSM_STATE_ERROR_ORE,    // Overrun error
  H7UART_FSM_STATE_ERROR_NE,     // Noise detection flag
  H7UART_FSM_STATE_ERROR_FE,     // Framing error
  H7UART_FSM_STATE_ERROR_PE,     // Parity error
  H7UART_FSM_STATE_UNMANAGED_BY_DRIVER
} h7uart_uart_fsm_state_t;

typedef enum
{
  H7UART_RX_FSM_STATE_UNINITIALIZED,
  H7UART_RX_FSM_STATE_READY,
  H7UART_RX_FSM_STATE_RECEIVING,
  H7UART_RX_FSM_STATE_ERROR
} h7uart_rx_fsm_state_t;

typedef enum
{
  H7UART_TX_FSM_STATE_UNINITIALIZED,
  H7UART_TX_FSM_STATE_READY,
  H7UART_TX_FSM_STATE_TRANSMITING,
  H7UART_TX_FSM_STATE_ERROR
} h7uart_tx_fsm_state_t;

typedef enum
{
  H7UART_RET_CODE_OK,
  H7UART_RET_CODE_BUSY,
  H7UART_RET_CODE_TIMEOUT,
  H7UART_RET_CODE_PERIPH_IN_ERR_STATE,
  H7UART_RET_CODE_INVALID_ARGS,
  H7UART_RET_CODE_ERROR,
  H7UART_RET_CODE_CLEARED_A_NONERROR_STATE,
  H7UART_RET_CODE_UNMANAGED_BY_DRIVER
} h7uart_uart_ret_code_t;

typedef enum
{
  H7UART_PIN_USART1_RX_PA10,
  H7UART_PIN_USART1_RX_PB7,
  H7UART_PIN_USART1_RX_PB15,

  H7UART_PIN_USART2_RX_PA3,
  H7UART_PIN_USART2_RX_PD6,

  H7UART_PIN_USART3_RX_PB11,
  H7UART_PIN_USART3_RX_PC11,
  H7UART_PIN_USART3_RX_PD9,

  H7UART_PIN_UART4_RX_PA1,
  H7UART_PIN_UART4_RX_PA11,
  H7UART_PIN_UART4_RX_PB8,
  H7UART_PIN_UART4_RX_PC11,
  H7UART_PIN_UART4_RX_PD0,
  H7UART_PIN_UART4_RX_PH14,
  H7UART_PIN_UART4_RX_PI9,

  H7UART_PIN_UART5_RX_PD2,

  H7UART_PIN_USART6_RX_PC7,
  H7UART_PIN_USART6_RX_PG9,

  H7UART_PIN_UART7_RX_PA8,
  H7UART_PIN_UART7_RX_PB3,
  H7UART_PIN_UART7_RX_PE7,
  H7UART_PIN_UART7_RX_PF6,

  H7UART_PIN_UART8_RX_PE0,
  H7UART_PIN_UART8_RX_PJ9,

  H7UART_PIN_LPUART1_RX_PA10,
  H7UART_PIN_LPUART1_RX_PB7

} h7uart_pin_rx_t;

typedef enum
{
  H7UART_PIN_USART1_TX_PA9,
  H7UART_PIN_USART1_TX_PB6,
  H7UART_PIN_USART1_TX_PB14,

  H7UART_PIN_USART2_TX_PA2,
  H7UART_PIN_USART2_TX_PD5,

  H7UART_PIN_USART3_TX_PB10,
  H7UART_PIN_USART3_TX_PC10,
  H7UART_PIN_USART3_TX_PD8,

  H7UART_PIN_UART4_TX_PA0,
  H7UART_PIN_UART4_TX_PA12,
  H7UART_PIN_UART4_TX_PB9,
  H7UART_PIN_UART4_TX_PC10,
  H7UART_PIN_UART4_TX_PD1,
  H7UART_PIN_UART4_TX_PH13,

  H7UART_PIN_UART5_TX_PC12,

  H7UART_PIN_USART6_TX_PC6,
  H7UART_PIN_USART6_TX_PG14,

  H7UART_PIN_UART7_TX_PA15,
  H7UART_PIN_UART7_TX_PB4,
  H7UART_PIN_UART7_TX_PE8,
  H7UART_PIN_UART7_TX_PF7,

  H7UART_PIN_UART8_TX_PE1,
  H7UART_PIN_UART8_TX_PJ8,

  H7UART_PIN_LPUART1_TX_PA9,
  H7UART_PIN_LPUART1_TX_PB6

} h7uart_pin_tx_t;

typedef enum
{
  PERIPH_TX_ONLY,
  PERIPH_RX_ONLY,
  PERIPH_TX_RX
} h7uart_periph_func_t;

typedef enum
{
  DATA_WORD_LENGTH_8_NO_PARITY,    // 1 start, 8 Data bits, no parity, 1 Stop bit
  DATA_WORD_LENGTH_8_EVEN_PARITY,  // 1 start, 8 Data bits, even parity, 1 Stop bit
  DATA_WORD_LENGTH_8_ODD_PARITY,   // 1 start, 8 Data bits, odd parity, 1 Stop bit
} h7uart_data_config_t;


typedef enum
{
  H7UART_PRESC_DIV_1   = 0UL,
  H7UART_PRESC_DIV_2   = 1UL,
  H7UART_PRESC_DIV_4   = 2UL,
  H7UART_PRESC_DIV_6   = 3UL,
  H7UART_PRESC_DIV_8   = 4UL,
  H7UART_PRESC_DIV_10  = 5UL,
  H7UART_PRESC_DIV_12  = 6UL,
  H7UART_PRESC_DIV_16  = 7UL,
  H7UART_PRESC_DIV_32  = 8UL,
  H7UART_PRESC_DIV_64  = 9UL,
  H7UART_PRESC_DIV_128 = 10UL,
  H7UART_PRESC_DIV_256 = 11UL
} h7uart_presc_t;

typedef enum
{
  FIFO_MODE_DISABLE,
  FIFO_MODE_ENABLE
} h7uart_fifo_mode_en_t;

typedef enum
{
  FIFO_TH_1_8      = 0UL, // FIFO reaches 1/8 of its depth
  FIFO_TH_1_4      = 1UL, // FIFO reaches 1/4 of its depth
  FIFO_TH_1_2      = 2UL, // FIFO reaches 1/2 of its depth
  FIFO_TH_3_4      = 3UL, // FIFO reaches 3/4 of its depth
  FIFO_TH_7_8      = 4UL, // FIFO reaches 7/8 of its depth
  FIFO_TH_COMPLETE = 5UL  // TX FIFO is empty or RX FIFO is full.
} h7uart_fifo_thres_t;


typedef struct
{
  uint8_t               irq_pri;
  uint8_t               irq_subpri;
  h7uart_pin_rx_t       pin_rx;
  h7uart_pin_tx_t       pin_tx;
  uint32_t              rcc_clksource;
  h7uart_periph_func_t  function;
  h7uart_data_config_t  data_config;
  h7uart_fifo_mode_en_t fifo_enable;
  h7uart_fifo_thres_t   fifo_rx_thres;
  h7uart_presc_t        presc;
  uint32_t              baud_rate;

  void (*irq_callback)(uint32_t);
  void (*rx_callback)(uint8_t*,uint32_t);

}h7uart_periph_init_config_t;


h7uart_uart_ret_code_t h7uart_uart_init(h7uart_periph_t peripheral);
h7uart_uart_ret_code_t h7uart_uart_init_by_config(h7uart_periph_t peripheral, h7uart_periph_init_config_t* init_config);
void h7uart_deinit(h7uart_periph_t peripheral);

h7uart_uart_ret_code_t h7uart_uart_reset_peripheral_full(h7uart_periph_t peripheral);
h7uart_uart_ret_code_t h7uart_uart_reset_peripheral_soft(h7uart_periph_t peripheral);
h7uart_uart_ret_code_t h7uart_uart_reset_driver(h7uart_periph_t peripheral);

int h7uart_uart_is_managed_by_this_driver(h7uart_periph_t peripheral);

int h7uart_is_ready(h7uart_periph_t peripheral);
h7uart_uart_ret_code_t h7uart_wait_until_ready(h7uart_periph_t peripheral, uint32_t timeout);

int h7uart_is_in_error(h7uart_periph_t peripheral);
h7uart_uart_fsm_state_t h7uart_get_state(h7uart_periph_t peripheral);
h7uart_uart_ret_code_t h7uart_clear_error_state(h7uart_periph_t peripheral);

h7uart_uart_ret_code_t h7uart_tx_nonblocking(h7uart_periph_t peripheral, uint16_t len,  uint8_t* data, uint32_t timeout);
h7uart_uart_ret_code_t h7uart_tx_blocking(h7uart_periph_t peripheral, uint16_t len,  uint8_t* data, uint32_t timeout);

h7uart_uart_ret_code_t h7uart_rx_from_isr(h7uart_periph_t peripheral, uint16_t len, uint8_t* data);
h7uart_uart_ret_code_t h7uart_rx_blocking(h7uart_periph_t peripheral, uint16_t len, uint8_t* data);

#endif // INC_H7UART_BARE_H_
