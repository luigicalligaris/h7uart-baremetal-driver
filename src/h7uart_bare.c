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

#include "h7uart_bare.h"

#include "main.h"

#include <string.h>

#include "stm32h7xx_hal_gpio.h"
#include "stm32h7xx_hal_rcc.h"
#include "stm32h745xx.h"

#include "h7uart_bare_priv.h"
#include "h7uart_config.h"

// For an explanation of the CR1 and CR2 bitmaps, see RM0399

#if H7UART_PERIPH_ENABLE_USART1 == 1
uint8_t h7uart_mutex_usart1 = H7UART_UART_MUTEX_UNLOCKED;

h7uart_driver_instance_state_t h7uart_state_usart1 =
{
  .fsm_state       = H7UART_FSM_STATE_UNINITIALIZED,
  .uart_base       = USART1,

  .cr1_value       = 0UL,
  .cr2_value       = 0UL,
  .cr3_value       = 0UL,
  .brr_value       = 0UL,
  .gtpr_value      = 0UL,
  .rtor_value      = 0UL,
  .rqr_value       = 0UL,

  .data_rx         = {0},

  .len_tx          = 0UL,
  .cont_tx         = 0UL,
  .data_tx         = NULL,

  .irq_callback    = NULL,
  .rx_callback     = NULL,

  .timestart       = 0UL,
  .timeout         = 0UL
};

h7uart_periph_init_config_t current_periph_init_config_usart1 =
{
  .pin_rx        = H7UART_PIN_USART1_RX_PA10,
  .pin_tx        = H7UART_PIN_USART1_TX_PB14,
  .rcc_clksource = RCC_USART1CLKSOURCE_D2PCLK2,
  .data_config   = DATA_WORD_LENGTH_8_NO_PARITY,
  .fifo_enable   = FIFO_MODE_DISABLE,
  .fifo_rx_thres = FIFO_TH_1_8,
  .presc         = H7UART_PRESC_DIV_1,
  .baud_rate     = 115200,
  .irq_callback  = NULL,
  .rx_callback   = NULL
};

#endif

#if H7UART_PERIPH_ENABLE_USART2 == 1
uint8_t h7uart_mutex_usart2 = H7UART_UART_MUTEX_UNLOCKED;

h7uart_driver_instance_state_t h7uart_state_usart2 =
{
  .fsm_state       = H7UART_FSM_STATE_UNINITIALIZED,
  .uart_base       = USART2,

  .cr1_value       = 0UL,
  .cr2_value       = 0UL,
  .cr3_value       = 0UL,
  .brr_value       = 0UL,
  .gtpr_value      = 0UL,
  .rtor_value      = 0UL,
  .rqr_value       = 0UL,

  .data_rx         = {0},

  .len_tx          = 0UL,
  .cont_tx         = 0UL,
  .data_tx         = NULL,

  .irq_callback    = NULL,
  .rx_callback     = NULL,

  .timestart       = 0UL,
  .timeout         = 0UL
};

h7uart_periph_init_config_t current_periph_init_config_usart2 =
{
  .pin_rx        = H7UART_PIN_USART1_RX_PA10,
  .pin_tx        = H7UART_PIN_USART1_TX_PB14,
  .rcc_clksource = RCC_USART1CLKSOURCE_D2PCLK2,
  .data_config   = DATA_WORD_LENGTH_8_NO_PARITY,
  .fifo_enable   = FIFO_MODE_DISABLE,
  .fifo_rx_thres = FIFO_TH_1_8,
  .presc         = H7UART_PRESC_DIV_1,
  .baud_rate     = 115200,
  .irq_callback  = NULL,
  .rx_callback   = NULL
};
#endif

#if H7UART_PERIPH_ENABLE_USART3 == 1
uint8_t h7uart_mutex_usart3 = H7UART_UART_MUTEX_UNLOCKED;

h7uart_driver_instance_state_t h7uart_state_usart3 =
{
  .fsm_state       = H7UART_FSM_STATE_UNINITIALIZED,
  .uart_base       = USART3,

  .cr1_value       = 0UL,
  .cr2_value       = 0UL,
  .cr3_value       = 0UL,
  .brr_value       = 0UL,
  .gtpr_value      = 0UL,
  .rtor_value      = 0UL,
  .rqr_value       = 0UL,

  .data_rx         = {0},

  .len_tx          = 0UL,
  .cont_tx         = 0UL,
  .data_tx         = NULL,

  .irq_callback    = NULL,
  .rx_callback     = NULL,

  .timestart       = 0UL,
  .timeout         = 0UL
};

h7uart_periph_init_config_t current_periph_init_config_usart3 =
{
  .pin_rx        = H7UART_PIN_USART1_RX_PA10,
  .pin_tx        = H7UART_PIN_USART1_TX_PB14,
  .rcc_clksource = RCC_USART1CLKSOURCE_D2PCLK2,
  .data_config   = DATA_WORD_LENGTH_8_NO_PARITY,
  .fifo_enable   = FIFO_MODE_DISABLE,
  .fifo_rx_thres = FIFO_TH_1_8,
  .presc         = H7UART_PRESC_DIV_1,
  .baud_rate     = 115200,
  .irq_callback  = NULL,
  .rx_callback   = NULL
};
#endif

#if H7UART_PERIPH_ENABLE_UART4 == 1
uint8_t h7uart_mutex_uart4 = H7UART_UART_MUTEX_UNLOCKED;

h7uart_driver_instance_state_t h7uart_state_uart4 =
{
  .fsm_state       = H7UART_FSM_STATE_UNINITIALIZED,
  .uart_base       = UART4,

  .cr1_value       = 0UL,
  .cr2_value       = 0UL,
  .cr3_value       = 0UL,
  .brr_value       = 0UL,
  .gtpr_value      = 0UL,
  .rtor_value      = 0UL,
  .rqr_value       = 0UL,

  .data_rx         = {0},

  .len_tx          = 0UL,
  .cont_tx         = 0UL,
  .data_tx         = NULL,

  .irq_callback    = NULL,
  .rx_callback     = NULL,

  .timestart       = 0UL,
  .timeout         = 0UL
};

h7uart_periph_init_config_t current_periph_init_config_uart4 =
{
  .pin_rx        = H7UART_PIN_USART1_RX_PA10,
  .pin_tx        = H7UART_PIN_USART1_TX_PB14,
  .rcc_clksource = RCC_USART1CLKSOURCE_D2PCLK2,
  .data_config   = DATA_WORD_LENGTH_8_NO_PARITY,
  .fifo_enable   = FIFO_MODE_DISABLE,
  .fifo_rx_thres = FIFO_TH_1_8,
  .presc         = H7UART_PRESC_DIV_1,
  .baud_rate     = 115200,
  .irq_callback  = NULL,
  .rx_callback   = NULL
};
#endif

#if H7UART_PERIPH_ENABLE_UART5 == 1
uint8_t h7uart_mutex_uart5 = H7UART_UART_MUTEX_UNLOCKED;

h7uart_driver_instance_state_t h7uart_state_uart5 =
{
  .fsm_state       = H7UART_FSM_STATE_UNINITIALIZED,
  .uart_base       = UART5,

  .cr1_value       = 0UL,
  .cr2_value       = 0UL,
  .cr3_value       = 0UL,
  .brr_value       = 0UL,
  .gtpr_value      = 0UL,
  .rtor_value      = 0UL,
  .rqr_value       = 0UL,

  .data_rx         = {0},

  .len_tx          = 0UL,
  .cont_tx         = 0UL,
  .data_tx         = NULL,

  .irq_callback    = NULL,
  .rx_callback     = NULL,

  .timestart       = 0UL,
  .timeout         = 0UL
};

h7uart_periph_init_config_t current_periph_init_config_uart5 =
{
  .pin_rx        = H7UART_PIN_USART1_RX_PA10,
  .pin_tx        = H7UART_PIN_USART1_TX_PB14,
  .rcc_clksource = RCC_USART1CLKSOURCE_D2PCLK2,
  .data_config   = DATA_WORD_LENGTH_8_NO_PARITY,
  .fifo_enable   = FIFO_MODE_DISABLE,
  .fifo_rx_thres = FIFO_TH_1_8,
  .presc         = H7UART_PRESC_DIV_1,
  .baud_rate     = 115200,
  .irq_callback  = NULL,
  .rx_callback   = NULL
};
#endif

#if H7UART_PERIPH_ENABLE_USART6 == 1
uint8_t h7uart_mutex_usart6 = H7UART_UART_MUTEX_UNLOCKED;

h7uart_driver_instance_state_t h7uart_state_usart6 =
{
  .fsm_state       = H7UART_FSM_STATE_UNINITIALIZED,
  .uart_base       = USART6,

  .cr1_value       = 0UL,
  .cr2_value       = 0UL,
  .cr3_value       = 0UL,
  .brr_value       = 0UL,
  .gtpr_value      = 0UL,
  .rtor_value      = 0UL,
  .rqr_value       = 0UL,

  .data_rx         = {0},

  .len_tx          = 0UL,
  .cont_tx         = 0UL,
  .data_tx         = NULL,

  .irq_callback    = NULL,
  .rx_callback     = NULL,

  .timestart       = 0UL,
  .timeout         = 0UL
};

h7uart_periph_init_config_t current_periph_init_config_usart6 =
{
  .pin_rx        = H7UART_PIN_USART1_RX_PA10,
  .pin_tx        = H7UART_PIN_USART1_TX_PB14,
  .rcc_clksource = RCC_USART1CLKSOURCE_D2PCLK2,
  .data_config   = DATA_WORD_LENGTH_8_NO_PARITY,
  .fifo_enable   = FIFO_MODE_DISABLE,
  .fifo_rx_thres = FIFO_TH_1_8,
  .presc         = H7UART_PRESC_DIV_1,
  .baud_rate     = 115200,
  .irq_callback  = NULL,
  .rx_callback   = NULL
};
#endif

#if H7UART_PERIPH_ENABLE_UART7 == 1
uint8_t h7uart_mutex_uart7 = H7UART_UART_MUTEX_UNLOCKED;

h7uart_driver_instance_state_t h7uart_state_uart7 =
{
  .fsm_state       = H7UART_FSM_STATE_UNINITIALIZED,
  .uart_base       = UART7,

  .cr1_value       = 0UL,
  .cr2_value       = 0UL,
  .cr3_value       = 0UL,
  .brr_value       = 0UL,
  .gtpr_value      = 0UL,
  .rtor_value      = 0UL,
  .rqr_value       = 0UL,

  .data_rx         = {0},

  .len_tx          = 0UL,
  .cont_tx         = 0UL,
  .data_tx         = NULL,

  .irq_callback    = NULL,
  .rx_callback     = NULL,

  .timestart       = 0UL,
  .timeout         = 0UL
};

h7uart_periph_init_config_t current_periph_init_config_uart7 =
{
  .pin_rx        = H7UART_PIN_USART1_RX_PA10,
  .pin_tx        = H7UART_PIN_USART1_TX_PB14,
  .rcc_clksource = RCC_USART1CLKSOURCE_D2PCLK2,
  .data_config   = DATA_WORD_LENGTH_8_NO_PARITY,
  .fifo_enable   = FIFO_MODE_DISABLE,
  .fifo_rx_thres = FIFO_TH_1_8,
  .presc         = H7UART_PRESC_DIV_1,
  .baud_rate     = 115200,
  .irq_callback  = NULL,
};
#endif

#if H7UART_PERIPH_ENABLE_UART8 == 1
uint8_t h7uart_mutex_uart8 = H7UART_UART_MUTEX_UNLOCKED;

h7uart_driver_instance_state_t h7uart_state_uart8 =
{
  .fsm_state       = H7UART_FSM_STATE_UNINITIALIZED,
  .uart_base       = UART8,

  .cr1_value       = 0UL,
  .cr2_value       = 0UL,
  .cr3_value       = 0UL,
  .brr_value       = 0UL,
  .gtpr_value      = 0UL,
  .rtor_value      = 0UL,
  .rqr_value       = 0UL,

  .data_rx         = {0},

  .len_tx          = 0UL,
  .cont_tx         = 0UL,
  .data_tx         = NULL,

  .irq_callback    = NULL,
  .rx_callback     = NULL,

  .timestart       = 0UL,
  .timeout         = 0UL
};

h7uart_periph_init_config_t current_periph_init_config_uart8 =
{
  .pin_rx        = H7UART_PIN_USART1_RX_PA10,
  .pin_tx        = H7UART_PIN_USART1_TX_PB14,
  .rcc_clksource = RCC_USART1CLKSOURCE_D2PCLK2,
  .data_config   = DATA_WORD_LENGTH_8_NO_PARITY,
  .fifo_enable   = FIFO_MODE_DISABLE,
  .fifo_rx_thres = FIFO_TH_1_8,
  .presc         = H7UART_PRESC_DIV_1,
  .baud_rate     = 115200,
  .irq_callback  = NULL,
};
#endif

#if H7UART_PERIPH_ENABLE_LPUART1 == 1
uint8_t h7uart_mutex_lpuart1 = H7UART_UART_MUTEX_UNLOCKED;

h7uart_driver_instance_state_t h7uart_state_lpuart1 =
{
  .fsm_state       = H7UART_FSM_STATE_UNINITIALIZED,
  .uart_base       = LPUART1,

  .cr1_value       = 0UL,
  .cr2_value       = 0UL,
  .cr3_value       = 0UL,
  .brr_value       = 0UL,
  .gtpr_value      = 0UL,
  .rtor_value      = 0UL,
  .rqr_value       = 0UL,

  .data_rx         = {0},

  .len_tx          = 0UL,
  .cont_tx         = 0UL,
  .data_tx         = NULL,

  .irq_callback    = NULL,
  .rx_callback     = NULL,

  .timestart       = 0UL,
  .timeout         = 0UL
};

h7uart_periph_init_config_t current_periph_init_config_lpuart1 =
{
  .pin_rx        = H7UART_PIN_USART1_RX_PA10,
  .pin_tx        = H7UART_PIN_USART1_TX_PB14,
  .rcc_clksource = RCC_USART1CLKSOURCE_D2PCLK2,
  .data_config   = DATA_WORD_LENGTH_8_NO_PARITY,
  .fifo_enable   = FIFO_MODE_DISABLE,
  .fifo_rx_thres = FIFO_TH_1_8,
  .presc         = H7UART_PRESC_DIV_1,
  .baud_rate     = 115200,
  .irq_callback  = NULL,
};
#endif

#if H7UART_PERIPH_ENABLE_USART1 == 1
  void USART1_IRQHandler(void)
  {
    H7UART_IRQHandler_Impl(H7UART_USART1);
  }
#endif

#if H7UART_PERIPH_ENABLE_USART2 == 1
  void USART2_IRQHandler(void)
  {
    H7UART_IRQHandler_Impl(H7UART_USART2);
  }
#endif

#if H7UART_PERIPH_ENABLE_USART3 == 1
  void USART3_IRQHandler(void)
  {
    H7UART_IRQHandler_Impl(H7UART_USART3);
  }
#endif

#if H7UART_PERIPH_ENABLE_UART4 == 1
  void UART4_IRQHandler(void)
  {
    H7UART_IRQHandler_Impl(H7UART_UART4);
  }
#endif

#if H7UART_PERIPH_ENABLE_UART5 == 1
  void UART5_IRQHandler(void)
  {
    H7UART_IRQHandler_Impl(H7UART_UART5);
  }
#endif

#if H7UART_PERIPH_ENABLE_USART6 == 1
  void USART6_IRQHandler(void)
  {
    H7UART_IRQHandler_Impl(H7UART_USART6);
  }
#endif

#if H7UART_PERIPH_ENABLE_UART7 == 1
  void UART7_IRQHandler(void)
  {
    H7UART_IRQHandler_Impl(H7UART_UART7);
  }
#endif

#if H7UART_PERIPH_ENABLE_UART8 == 1
  void UART8_IRQHandler(void)
  {
    H7UART_IRQHandler_Impl(H7UART_UART8);
  }
#endif

#if H7UART_PERIPH_ENABLE_LPUART1 == 1
  void LPUART1_IRQHandler(void)
  {
    H7UART_IRQHandler_Impl(H7UART_LPUART1);
  }
#endif


inline static h7uart_driver_instance_state_t* h7uart_get_driver_instance(h7uart_periph_t peripheral)
{
  switch(peripheral)
  {
  #if H7UART_PERIPH_ENABLE_USART1 == 1
    case H7UART_USART1:
      return &h7uart_state_usart1;
  #endif

  #if H7UART_PERIPH_ENABLE_USART2 == 1
    case H7UART_USART2:
      return &h7uart_state_usart2;
  #endif

  #if H7UART_PERIPH_ENABLE_USART3 == 1
    case H7UART_USART3:
      return &h7uart_state_usart3;
  #endif

  #if H7UART_PERIPH_ENABLE_UART4 == 1
    case H7UART_UART4:
      return &h7uart_state_uart4;
  #endif

  #if H7UART_PERIPH_ENABLE_UART5 == 1
    case H7UART_UART5:
      return &h7uart_state_uart5;
  #endif

  #if H7UART_PERIPH_ENABLE_USART6 == 1
    case H7UART_USART6:
      return &h7uart_state_usart6;
  #endif

  #if H7UART_PERIPH_ENABLE_UART7 == 1
    case H7UART_UART7:
      return &h7uart_state_uart7;
  #endif

  #if H7UART_PERIPH_ENABLE_UART8 == 1
    case H7UART_UART8:
      return &h7uart_state_uart8;
  #endif

  #if H7UART_PERIPH_ENABLE_LPUART1 == 1
    case H7UART_LPUART1:
      return &h7uart_state_lpuart1;
  #endif

    default:
      return NULL;
  };
}

inline int h7uart_is_peripheral_managed_by_this_driver(h7uart_periph_t peripheral)
{
  switch(peripheral)
  {
  #if H7UART_PERIPH_ENABLE_USART1 == 1
    case H7UART_USART1:
      return 1;
  #endif

  #if H7UART_PERIPH_ENABLE_USART2 == 1
    case H7UART_USART2:
      return 1;
  #endif

  #if H7UART_PERIPH_ENABLE_USART3 == 1
    case H7UART_USART3:
      return 1;
  #endif

  #if H7UART_PERIPH_ENABLE_UART4 == 1
    case H7UART_UART4:
      return 1;
  #endif

  #if H7UART_PERIPH_ENABLE_UART5 == 1
    case H7UART_UART5:
      return 1;
  #endif

  #if H7UART_PERIPH_ENABLE_USART6 == 1
    case H7UART_USART6:
      return 1;
  #endif

  #if H7UART_PERIPH_ENABLE_UART7 == 1
    case H7UART_UART7:
      return 1;
  #endif

  #if H7UART_PERIPH_ENABLE_UART8 == 1
    case H7UART_UART8:
      return 1;
  #endif

  #if H7UART_PERIPH_ENABLE_LPUART1 == 1
    case H7UART_LPUART1:
      return 1;
  #endif

    default:
      return 0;
  }
}

h7uart_uart_fsm_state_t h7uart_get_state(h7uart_periph_t peripheral)
{
  switch(peripheral)
    {
    #if H7UART_PERIPH_ENABLE_USART1 == 1
      case H7UART_USART1:
        return h7uart_state_usart1.fsm_state;
    #endif

    #if H7UART_PERIPH_ENABLE_USART2 == 1
      case H7UART_USART2:
        return h7uart_state_usart2.fsm_state;
    #endif

    #if H7UART_PERIPH_ENABLE_USART3 == 1
      case H7UART_USART3:
        return h7uart_state_usart3.fsm_state;
    #endif

    #if H7UART_PERIPH_ENABLE_UART4 == 1
      case H7UART_UART4:
        return h7uart_state_uart4.fsm_state;
    #endif

    #if H7UART_PERIPH_ENABLE_UART5 == 1
      case H7UART_UART5:
        return h7uart_state_uart5.fsm_state;
    #endif

    #if H7UART_PERIPH_ENABLE_USART6 == 1
      case H7UART_USART6:
        return h7uart_state_usart6.fsm_state;
    #endif

    #if H7UART_PERIPH_ENABLE_UART7 == 1
      case H7UART_UART7:
        return h7uart_state_uart7.fsm_state;
    #endif

    #if H7UART_PERIPH_ENABLE_UART8 == 1
      case H7UART_UART8:
        return h7uart_state_uart8.fsm_state;
    #endif

    #if H7UART_PERIPH_ENABLE_LPUART1 == 1
      case H7UART_LPUART1:
        return h7uart_state_lpuart1.fsm_state;
    #endif

      default:
        break;
    };

  return H7UART_FSM_STATE_UNMANAGED_BY_DRIVER;
}

int h7uart_is_in_error(h7uart_periph_t peripheral)
{
  switch(h7uart_get_state(peripheral))
  {
    case H7UART_FSM_STATE_ERROR_ABRE:
    case H7UART_FSM_STATE_ERROR_UDR:
    case H7UART_FSM_STATE_ERROR_ORE:
    case H7UART_FSM_STATE_ERROR_NE:
    case H7UART_FSM_STATE_ERROR_FE:
    case H7UART_FSM_STATE_ERROR_PE:
      return 1;
    default:
      break;
  }
  return 0;
}

int h7uart_is_ready(h7uart_periph_t peripheral)
{
  switch(h7uart_get_state(peripheral))
  {
    case H7UART_FSM_STATE_UNINITIALIZED:
    case H7UART_FSM_STATE_IDLE:
      return 1;
    default:
      break;
  }
  return 0;
}

__weak h7uart_uart_ret_code_t h7uart_wait_until_ready(h7uart_periph_t peripheral, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  // Blocking loop, waiting for the UART peripheral to become free, to get into error state or to timeout
  // This version loops until timeout if device is busy
  while (HAL_GetTick() - timestart < timeout)
  {
    if (h7uart_is_ready(peripheral))
      return H7UART_RET_CODE_OK;
  }
  return H7UART_RET_CODE_BUSY;
}

h7uart_uart_ret_code_t h7uart_uart_mutex_lock_impl(h7uart_periph_t peripheral)
{
  uint8_t* p_mutex = NULL;

  switch(peripheral)
  {
  #if H7UART_PERIPH_ENABLE_USART1 == 1
    case H7UART_USART1:
      p_mutex = &h7uart_mutex_usart1;
      break;
  #endif

  #if H7UART_PERIPH_ENABLE_USART2 == 1
    case H7UART_USART2:
      p_mutex = &h7uart_mutex_usart2;
      break;
  #endif

  #if H7UART_PERIPH_ENABLE_USART3 == 1
    case H7UART_USART3:
      p_mutex = &h7uart_mutex_usart3;
      break;
  #endif

  #if H7UART_PERIPH_ENABLE_UART4 == 1
    case H7UART_UART4:
      p_mutex = &h7uart_mutex_uart4;
      break;
  #endif

  #if H7UART_PERIPH_ENABLE_UART5 == 1
    case H7UART_UART5:
      p_mutex = &h7uart_mutex_uart5;
      break;
  #endif

  #if H7UART_PERIPH_ENABLE_USART6 == 1
    case H7UART_USART6:
      p_mutex = &h7uart_mutex_usart6;
      break;
  #endif

  #if H7UART_PERIPH_ENABLE_UART7 == 1
    case H7UART_UART7:
      p_mutex = &h7uart_mutex_uart7;
      break;
  #endif

  #if H7UART_PERIPH_ENABLE_UART8 == 1
    case H7UART_UART8:
      p_mutex = &h7uart_mutex_uart8;
      break;
  #endif

  #if H7UART_PERIPH_ENABLE_LPUART1 == 1
    case H7UART_LPUART1:
      p_mutex = &h7uart_mutex_lpuart1;
      break;
  #endif

    default:
      return H7UART_RET_CODE_UNMANAGED_BY_DRIVER;
  };

  // Cortex-M exclusive monitor read: sets the exclusive monitor flag on address p_mutex
  if (H7UART_UART_MUTEX_UNLOCKED == __LDREXB(p_mutex))
  {
    // Cortex-M exclusive monitor write: only writes and returns 0 if exclusive
    // monitor flag is still set, otherwise return nonzero and write nothing
    if (0 == __STREXB(H7UART_UART_MUTEX_LOCKED, p_mutex))
    {
      __DMB(); // Data Memory Barrier
      return H7UART_RET_CODE_OK;
    }
  }
  return H7UART_RET_CODE_BUSY;
}

__weak h7uart_uart_ret_code_t h7uart_uart_mutex_lock(h7uart_periph_t peripheral, uint32_t timeout)
{
  uint32_t const timestart = HAL_GetTick();

  // This is a simple infinite loop. As this is bare metal code it's supposed not to be an issue,
  // but in general it is inefficient. The RTOS implementation solves this with a yield call.
  while (HAL_GetTick() - timestart < timeout)
  {
    switch ( h7uart_uart_mutex_lock_impl(peripheral) )
    {
      // We got the mutex
      case H7UART_RET_CODE_OK:
        return H7UART_RET_CODE_OK;

      // This device is not managed by the driver
      case H7UART_RET_CODE_UNMANAGED_BY_DRIVER:
        return H7UART_RET_CODE_UNMANAGED_BY_DRIVER;

      // We havent't got the mutex (yet)
      case H7UART_RET_CODE_BUSY:
      default:
        continue;
    }
  }

  // We timed out
  return H7UART_RET_CODE_BUSY;
}


h7uart_uart_ret_code_t h7uart_uart_mutex_release(h7uart_periph_t peripheral)
{
  uint8_t* p_mutex = NULL;

  switch(peripheral)
  {
  #if H7UART_PERIPH_ENABLE_USART1 == 1
    case H7UART_USART1:
      p_mutex = &h7uart_mutex_usart1;
      break;
  #endif

  #if H7UART_PERIPH_ENABLE_USART2 == 1
    case H7UART_USART2:
      p_mutex = &h7uart_mutex_usart2;
      break;
  #endif

  #if H7UART_PERIPH_ENABLE_USART3 == 1
    case H7UART_USART3:
      p_mutex = &h7uart_mutex_usart3;
      break;
  #endif

  #if H7UART_PERIPH_ENABLE_UART4 == 1
    case H7UART_UART4:
      p_mutex = &h7uart_mutex_uart4;
      break;
  #endif

  #if H7UART_PERIPH_ENABLE_UART5 == 1
    case H7UART_UART5:
      p_mutex = &h7uart_mutex_uart5;
      break;
  #endif

  #if H7UART_PERIPH_ENABLE_USART6 == 1
    case H7UART_USART6:
      p_mutex = &h7uart_mutex_usart6;
      break;
  #endif

  #if H7UART_PERIPH_ENABLE_UART7 == 1
    case H7UART_UART7:
      p_mutex = &h7uart_mutex_uart7;
      break;
  #endif

  #if H7UART_PERIPH_ENABLE_UART8 == 1
    case H7UART_UART8:
      p_mutex = &h7uart_mutex_uart8;
      break;
  #endif

  #if H7UART_PERIPH_ENABLE_LPUART1 == 1
    case H7UART_LPUART1:
      p_mutex = &h7uart_mutex_lpuart1;
      break;
  #endif

    default:
      return H7UART_RET_CODE_UNMANAGED_BY_DRIVER;
  };

  *p_mutex = H7UART_UART_MUTEX_UNLOCKED;

  return H7UART_RET_CODE_OK;
}

h7uart_uart_ret_code_t h7uart_uart_mutex_release_fromISR(h7uart_periph_t peripheral)
{
  return h7uart_uart_mutex_release(peripheral);
}

h7uart_uart_ret_code_t h7uart_uart_reset_peripheral_full(h7uart_periph_t peripheral)
{
  h7uart_driver_instance_state_t* instance = h7uart_get_driver_instance(peripheral);

  if (!instance)
    return H7UART_RET_CODE_UNMANAGED_BY_DRIVER;

  USART_TypeDef* const uartx = (USART_TypeDef*) instance->uart_base;

  // Clear the Control Register 1 (disable some interrupts)
  MODIFY_REG(uartx->CR1,
        USART_CR1_RXFFIE         | USART_CR1_TXFEIE | USART_CR1_FIFOEN | USART_CR1_M1   | USART_CR1_EOBIE         | USART_CR1_RTOIE
      | USART_CR1_DEAT           | USART_CR1_DEDT   | USART_CR1_OVER8  | USART_CR1_CMIE | USART_CR1_MME           | USART_CR1_M0
      | USART_CR1_WAKE           | USART_CR1_PCE    | USART_CR1_PS     | USART_CR1_PEIE | USART_CR1_TXEIE_TXFNFIE | USART_CR1_TCIE
      | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_IDLEIE | USART_CR1_TE     | USART_CR1_RE   | USART_CR1_UESM          | USART_CR1_UE,
      0x00000000);

  // Clear the Control Register 2 (disable some interrupts)
  MODIFY_REG(uartx->CR2,
        USART_CR2_ADD     | USART_CR2_RTOEN | USART_CR2_ABRMODE | USART_CR2_ABREN | USART_CR2_MSBFIRST | USART_CR2_DATAINV
      | USART_CR2_TXINV   | USART_CR2_RXINV | USART_CR2_SWAP    | USART_CR2_LINEN | USART_CR2_STOP     | USART_CR2_CLKEN
      | USART_CR2_CPOL    | USART_CR2_CPHA  | USART_CR2_LBCL    | USART_CR2_LBDIE | USART_CR2_LBDL     | USART_CR2_ADDM7
      | USART_CR2_DIS_NSS | USART_CR2_SLVEN,
      0x00000000);

  // Clear the Control Register 3 (disable some interrupts)
  MODIFY_REG(uartx->CR3,
      USART_CR3_TXFTCFG | USART_CR3_RXFTIE  | USART_CR3_RXFTCFG | USART_CR3_TCBGTIE | USART_CR3_TXFTIE | USART_CR3_WUFIE
    | USART_CR3_WUS     | USART_CR3_SCARCNT | USART_CR3_DEP     | USART_CR3_DEM     | USART_CR3_DDRE   | USART_CR3_OVRDIS
    | USART_CR3_ONEBIT  | USART_CR3_CTSIE   | USART_CR3_CTSE    | USART_CR3_RTSE    | USART_CR3_DMAT   | USART_CR3_DMAR
    | USART_CR3_SCEN    | USART_CR3_NACK    | USART_CR3_HDSEL   | USART_CR3_IRLP    | USART_CR3_IREN   | USART_CR3_EIE,
    0x00000000);

  // Interrupt Clear Register
  MODIFY_REG(uartx->ICR,
      USART_ICR_WUCF  | USART_ICR_CMCF    |  USART_ICR_UDRCF | USART_ICR_EOBCF  | USART_ICR_RTOCF  | USART_ICR_CTSCF
    | USART_ICR_LBDCF | USART_ICR_TCBGTCF | USART_ICR_TCCF   | USART_ICR_TXFECF | USART_ICR_IDLECF | USART_ICR_ORECF
    | USART_ICR_NECF  | USART_ICR_FECF    | USART_ICR_PECF,
    0xFFFFFFFF);

  // Mutex release
  h7uart_uart_mutex_release(peripheral);

  // Update driver state
  instance->fsm_state = H7UART_FSM_STATE_UNINITIALIZED;

  return H7UART_RET_CODE_OK;
}

h7uart_uart_ret_code_t h7uart_uart_reset_peripheral_soft(h7uart_periph_t peripheral)
{
  h7uart_driver_instance_state_t* instance = h7uart_get_driver_instance(peripheral);

  if (!instance)
    return H7UART_RET_CODE_UNMANAGED_BY_DRIVER;

  USART_TypeDef* const uartx = (USART_TypeDef*) instance->uart_base;

  CLEAR_BIT(uartx->CR1, USART_CR1_UE);
  ((void) READ_BIT(uartx->CR1, USART_CR1_UE)); // the cast to void to suppress the "value computed is not used [-Wunused-value]" warning
  SET_BIT(uartx->CR1, USART_CR1_UE);

  // Mutex release
  h7uart_uart_mutex_release(peripheral);

  // Update driver state
  instance->fsm_state = H7UART_FSM_STATE_IDLE;

  return H7UART_RET_CODE_OK;
}

h7uart_uart_ret_code_t h7uart_uart_init(h7uart_periph_t peripheral)
{
  switch(peripheral)
    {
    #if H7UART_PERIPH_ENABLE_USART1 == 1
      case H7UART_USART1:
        h7uart_uart_init_by_config(peripheral,&current_periph_init_config_usart1);
        break;
    #endif

    #if H7UART_PERIPH_ENABLE_USART2 == 1
      case H7UART_USART2:
        h7uart_uart_init_by_config(peripheral,&current_periph_init_config_usart2);
        break;
    #endif

    #if H7UART_PERIPH_ENABLE_USART3 == 1
      case H7UART_USART3:
        h7uart_uart_init_by_config(peripheral,&current_periph_init_config_usart3);
        break;
    #endif

    #if H7UART_PERIPH_ENABLE_UART4 == 1
      case H7UART_UART4:
        h7uart_uart_init_by_config(peripheral,&current_periph_init_config_uart4);
        break;
    #endif

    #if H7UART_PERIPH_ENABLE_UART5 == 1
      case H7UART_UART5:
        h7uart_uart_init_by_config(peripheral,&current_periph_init_config_uart5);
        break;
    #endif

    #if H7UART_PERIPH_ENABLE_USART6 == 1
      case H7UART_USART6:
        h7uart_uart_init_by_config(peripheral,&current_periph_init_config_usart6);
        break;
    #endif

    #if H7UART_PERIPH_ENABLE_UART7 == 1
      case H7UART_UART7:
        h7uart_uart_init_by_config(peripheral,&current_periph_init_config_uart7);
        break;
    #endif

    #if H7UART_PERIPH_ENABLE_UART8 == 1
      case H7UART_UART8:
        h7uart_uart_init_by_config(peripheral,&current_periph_init_config_uart8);
        break;
    #endif

    #if H7UART_PERIPH_ENABLE_LPUART1 == 1
      case H7UART_LPUART1:
        h7uart_uart_init_by_config(peripheral,&current_periph_init_config_lpuart1);
        break;
    #endif

      default:
        break;
    };

  return H7UART_RET_CODE_UNMANAGED_BY_DRIVER;
}

h7uart_uart_ret_code_t h7uart_uart_init_by_config(h7uart_periph_t peripheral, h7uart_periph_init_config_t* init_config)
{
  // variables for peripheral init. configuration
  uint32_t cr1_value   = 0UL;
  uint32_t cr2_value   = 0UL;
  uint32_t cr3_value   = 0UL;
  uint32_t brr_value   = 0UL;
  uint32_t gtpr_value  = 0UL;
  uint32_t rtor_value  = 0UL;
  uint32_t rqr_value   = 0UL;
  uint32_t presc_value = 0UL;

  uint32_t usart_ker_ckpres;

  if(!init_config)
    return H7UART_RET_CODE_INVALID_ARGS;

  // Data word length configuration
  switch(init_config->data_config)
  {
    case DATA_WORD_LENGTH_8_NO_PARITY:   // 1 start, 8 Data bits, no parity, 1 Stop bit
      cr1_value =
          ( (0UL << USART_CR1_M1_Pos   ) & USART_CR1_M1   ) | //
          ( (0UL << USART_CR1_M0_Pos   ) & USART_CR1_M0   ) | // M[1:0] = '00': 1 start, 8 Data bits, n Stop bit.
          ( (0UL << USART_CR1_PCE_Pos  ) & USART_CR1_PCE  ) | // PCE    = Parity Control (generation and detection) disable.
          ( (0UL << USART_CR1_PS_Pos   ) & USART_CR1_PS   ) | // PS     = Parity Selection don't care because it is disable.
          ( (0UL << USART_CR1_PEIE_Pos ) & USART_CR1_PEIE ) ; // PEIE   = Parity error interrupt inhibited.

      cr2_value =
          ( (0UL << USART_CR2_STOP_Pos ) & USART_CR2_STOP ) ; // STOP   = 1 stop bit
      break;

    case DATA_WORD_LENGTH_8_EVEN_PARITY: // 1 start, 8 Data bits, even parity, 1 Stop bit
      cr1_value =
          ( (0UL << USART_CR1_M1_Pos   ) & USART_CR1_M1   ) | //
          ( (1UL << USART_CR1_M0_Pos   ) & USART_CR1_M0   ) | // M[1:0] = '00': 1 start, 9 Data bits, n Stop bit.
          ( (1UL << USART_CR1_PCE_Pos  ) & USART_CR1_PCE  ) | // PCE    = Parity Control (generation and detection) enable in MSB.
          ( (0UL << USART_CR1_PS_Pos   ) & USART_CR1_PS   ) | // PS     = Even Parity
          ( (1UL << USART_CR1_PEIE_Pos ) & USART_CR1_PEIE ) ; // PEIE   = USART interrupt generated whenever PE=1 in the USART_ISR reg.

      cr2_value =
          ( (0UL << USART_CR2_STOP_Pos ) & USART_CR2_STOP ) ;   // STOP   = 1 stop bit
      break;

    case DATA_WORD_LENGTH_8_ODD_PARITY:  // 1 start, 8 Data bits, odd parity, 1 Stop bit
      cr1_value =
          ( (0UL << USART_CR1_M1_Pos   ) & USART_CR1_M1   ) | //
          ( (1UL << USART_CR1_M0_Pos   ) & USART_CR1_M0   ) | // M[1:0] = '00': 1 start, 9 Data bits, n Stop bit.
          ( (1UL << USART_CR1_PCE_Pos  ) & USART_CR1_PCE  ) | // PCE    = Parity Control (generation and detection) enable in MSB.
          ( (1UL << USART_CR1_PS_Pos   ) & USART_CR1_PS   ) | // PS     = Odd Parity
          ( (1UL << USART_CR1_PEIE_Pos ) & USART_CR1_PEIE ) ; // PEIE   = USART interrupt generated whenever PE=1 in the USART_ISR reg.

      cr2_value =
          ( (0UL << USART_CR2_STOP_Pos ) & USART_CR2_STOP ) ; // STOP   = 1 stop bit
      break;

    default:
      Error_Handler();
  }

  // Configuration if the peripheral works as transmitter and/or receiver
  switch(init_config->function)
  {
    case PERIPH_TX_ONLY:
      cr1_value =
          ((1UL << USART_CR1_TE_Pos    ) & USART_CR1_TE   );  // TE  = Transmitter enable
      break;

    case PERIPH_RX_ONLY:
      cr1_value =
          ((1UL << USART_CR1_RE_Pos    ) & USART_CR1_RE   );  // RE  = Receiver enable
      break;

    case PERIPH_TX_RX:
      cr1_value =
          ((1UL << USART_CR1_TE_Pos    ) & USART_CR1_TE   ) | // TE  = Transmitter enable
          ((1UL << USART_CR1_RE_Pos    ) & USART_CR1_RE   ) ; // RE  = Receiver enable
      break;

    default:
      Error_Handler();
  }

  // FIFO enable and configuration
  switch(init_config->fifo_enable)
  {
   case FIFO_MODE_DISABLE:
     cr1_value =
         ( (0UL << USART_CR1_FIFOEN_Pos        ) & USART_CR1_FIFOEN        ) | // FIFOEN = FIFO mode is not enable.
         ( (1UL << USART_CR1_TXEIE_TXFNFIE_Pos ) & USART_CR1_TXEIE_TXFNFIE ) | // TXEIE  = Transmit data reg. empty interrupt.
         ( (1UL << USART_CR1_RXNEIE_RXFNEIE_Pos) & USART_CR1_RXNEIE        ) ; // RXNEIE = Receive data reg. not empty interrupt.
     break;

   case FIFO_MODE_ENABLE:
     cr1_value =
         ( (1UL << USART_CR1_RXFFIE_Pos        ) & USART_CR1_RXFFIE             ) | // RXFFIE  = RXFIFO Full interrupt enable.
         ( (1UL << USART_CR1_TXFEIE_Pos        ) & USART_CR1_TXFEIE             ) | // TXFEIE: = TXFIDO empty interrupt enable.
         ( (1UL << USART_CR1_FIFOEN_Pos        ) & USART_CR1_FIFOEN             ) | // FIFOEN  = FIFO mode is not enable.
         ( (1UL << USART_CR1_TXEIE_TXFNFIE_Pos ) & USART_CR1_TXEIE_TXFNFIE      ) | // TXFNFIE = TXFIFO not full interrupt enable.
         ( (1UL << USART_CR1_RXNEIE_RXFNEIE_Pos) & USART_CR1_RXNEIE_RXFNEIE_Pos ) ; // RXFNEIE = RXFIFO not empty interrupt enable.

     cr3_value =
         ( (0UL                                      << USART_CR3_TXFTCFG_Pos ) & USART_CR3_TXFTCFG ) | // TXFTCFG = TXFIFO thres. config.
         ( (1UL                                      << USART_CR3_RXFTIE_Pos  ) & USART_CR3_RXFTIE  ) | // RXFTIE  = RXFIFO thres. interrupt enable
         ( (((uint32_t) init_config->fifo_rx_thres ) << USART_CR3_RXFTCFG_Pos ) & USART_CR3_RXFTCFG ) | // RXFTCFG = TXFIFO thres. config.
         ( (1UL                                      << USART_CR3_TXFTIE_Pos  ) & USART_CR3_TXFTIE  ) ; // RXFTIE  = RXFIFO thres. interrupt enable
     break;

   default:
     Error_Handler();
  }

  //

  // Set the Prescaler value
  presc_value = (uint32_t) init_config->presc;

  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  switch(peripheral)
  {
    #if H7UART_PERIPH_ENABLE_USART1 == 1
      case H7UART_USART1:
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
        PeriphClkInitStruct.Usart16ClockSelection  = init_config->rcc_clksource;
        memcpy(&current_periph_init_config_usart1, init_config, sizeof(h7uart_periph_init_config_t));
        break;
    #endif

    #if H7UART_PERIPH_ENABLE_USART2 == 1
      case H7UART_USART2:
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART2;
        PeriphClkInitStruct.Usart234578ClockSelection  = init_config->rcc_clksource;
        memcpy(&current_periph_init_config_usart2, init_config, sizeof(h7uart_periph_init_config_t));
        break;
    #endif

    #if H7UART_PERIPH_ENABLE_USART3 == 1
      case H7UART_USART3:
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART3;
        PeriphClkInitStruct.Usart234578ClockSelection  = init_config->rcc_clksource;
        memcpy(&current_periph_init_config_usart3, init_config, sizeof(h7uart_periph_init_config_t));
        break;
    #endif

    #if H7UART_PERIPH_ENABLE_UART4 == 1
      case H7UART_UART4:
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART4;
        PeriphClkInitStruct.Usart234578ClockSelection  = init_config->rcc_clksource;
        memcpy(&current_periph_init_config_uart4, init_config, sizeof(h7uart_periph_init_config_t));
        break;
    #endif

    #if H7UART_PERIPH_ENABLE_UART5 == 1
      case H7UART_UART5:
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART5;
        PeriphClkInitStruct.Usart234578ClockSelection  = init_config->rcc_clksource;
        memcpy(&current_periph_init_config_uart5, init_config, sizeof(h7uart_periph_init_config_t));
        break;
    #endif

    #if H7UART_PERIPH_ENABLE_USART6 == 1
      case H7UART_USART6:
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART6;
        PeriphClkInitStruct.Usart16ClockSelection  = init_config->rcc_clksource;
        memcpy(&current_periph_init_config_usart6, init_config, sizeof(h7uart_periph_init_config_t));
        break;
    #endif

    #if H7UART_PERIPH_ENABLE_UART7 == 1
      case H7UART_UART7:
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART7;
        PeriphClkInitStruct.Usart234578ClockSelection  = init_config->rcc_clksource;
        memcpy(&current_periph_init_config_uart7, init_config, sizeof(h7uart_periph_init_config_t));
        break;
    #endif

    #if H7UART_PERIPH_ENABLE_UART8 == 1
      case H7UART_UART8:
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_UART8;
        PeriphClkInitStruct.Usart234578ClockSelection  = init_config->rcc_clksource;
        memcpy(&current_periph_init_config_uart8, init_config, sizeof(h7uart_periph_init_config_t));
        break;
    #endif

    #if H7UART_PERIPH_ENABLE_LPUART1 == 1
      case H7UART_LPUART1:
        PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LPUART1;
        PeriphClkInitStruct.Lpuart1ClockSelection  = init_config->rcc_clksource;
        memcpy(&current_periph_init_config_lpuart1, init_config, sizeof(h7uart_periph_init_config_t));
        break;
    #endif

      default:
        return H7UART_RET_CODE_UNMANAGED_BY_DRIVER;
  };

  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  switch(peripheral)
  {
#if H7UART_PERIPH_ENABLE_USART1  == 1
    case H7UART_USART1:
      //
      // *** GPIO PIN SETUP ***
      //

      if ((init_config->function == PERIPH_RX_ONLY) || (init_config->function == PERIPH_TX_RX))
      {
        switch(init_config->pin_rx)
        {
          case H7UART_PIN_USART1_RX_PA10:
            __HAL_RCC_GPIOA_CLK_ENABLE();

            // GPIOA AFRL: Set alternate function USART1 = 7 = 0b0111 (see datasheet chapt 5) to pin PA10 (USART1_RX)
            MODIFY_REG(GPIOA->AFR[1], 0b1111 << 8, 0b0111 << 8);
            // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA10
            MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 20, 0b11 << 20);
            // GPIOA PUPDR: Set pull-up = 0b01 to pin PA10
            MODIFY_REG(GPIOA->PUPDR, 0b11 << 20, 0b01 << 20);
            // GPIOA OTYPEDR: Set no open drain = 0b0 to pin PA10
            MODIFY_REG(GPIOA->OTYPER, 0b1 <<  10, 0b0 <<  10);
            // GPIOA MODER: Set alternate mode = 0b10 to pins PA10
            MODIFY_REG(GPIOA->MODER, 0b11 << 20, 0b10 << 20);
            break;

          case H7UART_PIN_USART1_RX_PB7:
            __HAL_RCC_GPIOB_CLK_ENABLE();

            // GPIOB AFRL: Set alternate function USART1 = 7 = 0b0111 (see datasheet chapt 5) to pin PB7 (USART1_RX)
            MODIFY_REG(GPIOB->AFR[0], 0b1111 << 28, 0b0111 << 28);
            // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB7
            MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 14, 0b11 << 14);
            // GPIOB PUPDR: Set pull-up = 0b01 to pin PB7
            MODIFY_REG(GPIOB->PUPDR, 0b11 << 14, 0b01 << 14);
            // GPIOB OTYPEDR: Set no open drain = 0b0 to pin PB7
            MODIFY_REG(GPIOB->OTYPER, 0b1 <<  7, 0b0 <<  7);
            // GPIOB MODER: Set alternate mode = 0b10 to pins PB7
            MODIFY_REG(GPIOB->MODER, 0b11 << 14, 0b10 << 14);
            break;

          case H7UART_PIN_USART1_RX_PB15:
            __HAL_RCC_GPIOB_CLK_ENABLE();

            // GPIOB AFRL: Set alternate function USART1 = 4 = 0b0100 (see datasheet chapt 5) to pin PB15  (USART1_RX)
            MODIFY_REG(GPIOB->AFR[1], 0b1111 << 28, 0b0100 << 28);
            // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB15
            MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 30, 0b11 << 30);
            // GPIOB PUPDR: Set pull-up = 0b01 to pin PB15
            MODIFY_REG(GPIOB->PUPDR, 0b11 << 30, 0b01 << 30);
            // GPIOB OTYPEDR: Set no open drain = 0b to pin PB15
            MODIFY_REG(GPIOB->OTYPER, 0b1 <<  15, 0b0 <<  15);
            // GPIOB MODER: Set alternate mode = 0b10 to pins PB15
            MODIFY_REG(GPIOB->MODER, 0b11 << 30, 0b10 << 30);
            break;

          default:
            Error_Handler();
        }
      }

      if ((init_config->function == PERIPH_TX_ONLY) || (init_config->function == PERIPH_TX_RX))
      {

        switch(init_config->pin_tx)
        {
          case H7UART_PIN_USART1_TX_PA9:
            __HAL_RCC_GPIOA_CLK_ENABLE();

            // GPIOA AFRL: Set alternate function USART1 = 7 = 0b0111 (see datasheet chapt 5) to pin PA9  (USART1_TX)
            MODIFY_REG(GPIOA->AFR[1], 0b1111 << 4, 0b0111 << 4);
            // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA9
            MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 18, 0b11 << 18);
            // GPIOA PUPDR: Set pull-up = 0b01 to pin PA9
            MODIFY_REG(GPIOA->PUPDR, 0b11 << 18, 0b01 << 18);
            // GPIOA OTYPEDR: Set no open drain = 0b to pin PA9
            MODIFY_REG(GPIOA->OTYPER, 0b1 <<  9, 0b0 <<  9);
            // GPIOA MODER: Set alternate mode = 0b10 to pins PA9
            MODIFY_REG(GPIOA->MODER, 0b11 << 18, 0b10 << 18);
            break;

          case H7UART_PIN_USART1_TX_PB6:
            __HAL_RCC_GPIOB_CLK_ENABLE();

            // GPIOB AFRL: Set alternate function USART1 = 7 = 0b0111 (see datasheet chapt 5) to pin PB6  (USART1_TX)
            MODIFY_REG(GPIOB->AFR[0], 0b1111 << 24, 0b0111 << 24);
            // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB6
            MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 12, 0b11 << 12);
            // GPIOB PUPDR: Set pull-up = 0b01 to pin PB6
            MODIFY_REG(GPIOB->PUPDR, 0b11 << 12, 0b01 << 12);
            // GPIOB OTYPEDR: Set no open drain = 0b to pin PB6
            MODIFY_REG(GPIOB->OTYPER, 0b1 <<  6, 0b0 <<  6);
            // GPIOB MODER: Set alternate mode = 0b10 to pins PB6
            MODIFY_REG(GPIOB->MODER, 0b11 << 12, 0b10 << 12);
            break;

          case H7UART_PIN_USART1_TX_PB14:
            __HAL_RCC_GPIOB_CLK_ENABLE();

            // GPIOB AFRL: Set alternate function USART1 = 7 = 0b0111 (see datasheet chapt 5) to pin PB14  (USART1_TX)
            MODIFY_REG(GPIOB->AFR[1], 0b1111 << 24, 0b0111 << 24);
            // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB14
            MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 28, 0b11 << 28);
            // GPIOB PUPDR: Set pull-up = 0b01 to pin PB14
            MODIFY_REG(GPIOB->PUPDR, 0b11 << 28, 0b01 << 28);
            // GPIOB OTYPEDR: Set no open drain = 0b to pin PB14
            MODIFY_REG(GPIOB->OTYPER, 0b1 <<  14, 0b0 <<  14);
            // GPIOB MODER: Set alternate mode = 0b10 to pins PB14
            MODIFY_REG(GPIOB->MODER, 0b11 << 28, 0b10 << 28);
            break;

          default:
            Error_Handler();
        }
      }

      //
      // *** INTERRUPT CALLBACK CONFIG.
      //

      //- Global peripheral callback
      h7uart_state_usart1.irq_callback = init_config->irq_callback;

      //- Reception callback
      h7uart_state_usart1.rx_callback  = init_config->rx_callback;

      //
      // *** UART SETUP ***
      //
      __HAL_RCC_C1_USART1_CLK_ENABLE();

      h7uart_uart_reset_peripheral_full(H7UART_USART1);

      // Set the Config Register 1
      MODIFY_REG(USART1->CR1,
            USART_CR1_M1             | USART_CR1_EOBIE  | USART_CR1_RTOIE
          | USART_CR1_DEAT           | USART_CR1_DEDT   | USART_CR1_OVER8  | USART_CR1_CMIE | USART_CR1_MME           | USART_CR1_M0
          | USART_CR1_WAKE           | USART_CR1_PCE    | USART_CR1_PS     | USART_CR1_PEIE | USART_CR1_TXEIE_TXFNFIE | USART_CR1_TCIE
          | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_IDLEIE | USART_CR1_TE     | USART_CR1_RE   | USART_CR1_UESM          | USART_CR1_UE,
          cr1_value);

      // Set the Config Register 2
      MODIFY_REG(USART1->CR2,
            USART_CR2_ADD     | USART_CR2_RTOEN | USART_CR2_ABRMODE | USART_CR2_ABREN | USART_CR2_MSBFIRST | USART_CR2_DATAINV
          | USART_CR2_TXINV   | USART_CR2_RXINV | USART_CR2_SWAP    | USART_CR2_LINEN | USART_CR2_STOP     | USART_CR2_CLKEN
          | USART_CR2_CPOL    | USART_CR2_CPHA  | USART_CR2_LBCL    | USART_CR2_LBDIE | USART_CR2_LBDL     | USART_CR2_ADDM7
          | USART_CR2_DIS_NSS | USART_CR2_SLVEN,
          cr2_value);

      // Set the Config Register 3
      MODIFY_REG(USART1->CR3,
          USART_CR3_TCBGTIE | USART_CR3_WUFIE
        | USART_CR3_WUS     | USART_CR3_SCARCNT | USART_CR3_DEP     | USART_CR3_DEM     | USART_CR3_DDRE   | USART_CR3_OVRDIS
        | USART_CR3_ONEBIT  | USART_CR3_CTSIE   | USART_CR3_CTSE    | USART_CR3_RTSE    | USART_CR3_DMAT   | USART_CR3_DMAR
        | USART_CR3_SCEN    | USART_CR3_NACK    | USART_CR3_HDSEL   | USART_CR3_IRLP    | USART_CR3_IREN   | USART_CR3_EIE,
        cr3_value);

      // Set the Guard Time and Prescaler Register
      // ( It is related to Smartcard function which is not implemented by the present driver )
      MODIFY_REG(USART1->GTPR,USART_GTPR_GT | USART_GTPR_PSC,0UL);

      // Set Receiver Timeout Register
      // - BLEN: It is used only in Smartcard mode which is not implmented by the present driver
      // - RTO: Receive timeout value it is not used in the present driver.
      MODIFY_REG(USART1->RTOR,USART_RTOR_BLEN | USART_RTOR_RTO,0UL);

      // Set Baud Rate Configuration

      // - Kernel Clock after prescaler
      usart_ker_ckpres = USART_CLOCK_KER_USART_1 / presc_value;

      // - Compute baud rate register value
      brr_value = usart_ker_ckpres / init_config->baud_rate;

      // Checks if the resulting brr values fits in 16-bits
      if ( brr_value != ( brr_value & 0x0000FFFF ) )
        Error_Handler();

      // Set Prescaler Register
      MODIFY_REG(USART1->PRESC,USART_PRESC_PRESCALER,presc_value);

      // Set Baud Rate Register
      MODIFY_REG(USART1->BRR,USART_BRR_DIV_FRACTION | USART_BRR_DIV_MANTISSA,brr_value);

      //
      //  *** NVIC SETUP ***
      //
      HAL_NVIC_SetPriority(USART1_IRQn,H7UART_IRQ_USART1_PRI, H7UART_IRQ_USART1_SUBPRI);
      HAL_NVIC_EnableIRQ(USART1_IRQn);

      // Peripheral enable
      SET_BIT(USART1->CR1,USART_CR1_UE);
      READ_REG(USART1->CR1);

      // Update driver state
      h7uart_state_usart1.fsm_state = H7UART_FSM_STATE_IDLE;

      break;
#endif

#if H7UART_PERIPH_ENABLE_USART2  == 1
    case H7UART_USART2:
      //
      // *** GPIO PIN SETUP ***
      //

      if ((init_config->function == PERIPH_RX_ONLY) || (init_config->function == PERIPH_TX_RX))
      {

        switch(init_config->pin_rx)
        {
          case H7UART_PIN_USART2_RX_PA3:
            __HAL_RCC_GPIOA_CLK_ENABLE();

            // GPIOA AFRL: Set alternate function USART2 = 7 = 0b0111 (see datasheet chapt 5) to pin PA3 (USART2_RX)
            MODIFY_REG(GPIOA->AFR[0], 0b1111 << 12, 0b0111 << 12);
            // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA3
            MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 6, 0b11 << 6);
            // GPIOA PUPDR: Set pull-up = 0b01 to pin PA3
            MODIFY_REG(GPIOA->PUPDR, 0b11 << 6, 0b01 << 6);
            // GPIOA OTYPEDR: Set no open drain = 0b0 to pin PA3
            MODIFY_REG(GPIOA->OTYPER, 0b1 <<  3, 0b0 <<  3);
            // GPIOA MODER: Set alternate mode = 0b10 to pins PA3
            MODIFY_REG(GPIOA->MODER, 0b11 << 6, 0b10 << 6);
            break;

          case H7UART_PIN_USART2_RX_PD6:
            __HAL_RCC_GPIOD_CLK_ENABLE();

            // GPIOD AFRL: Set alternate function USART2 = 7 = 0b0111 (see datasheet chapt 5) to pin PD6 (USART2_RX)
            MODIFY_REG(GPIOD->AFR[0], 0b1111 << 24, 0b0111 << 24);
            // GPIOD OSPEEDR: Set very high speed = 0b11 to pin PD6
            MODIFY_REG(GPIOD->OSPEEDR, 0b11 << 12, 0b11 << 12);
            // GPIOD PUPDR: Set pull-up = 0b01 to pin PD6
            MODIFY_REG(GPIOD->PUPDR, 0b11 << 12, 0b01 << 12);
            // GPIOD OTYPEDR: Set no open drain = 0b0 to pin PD6
            MODIFY_REG(GPIOD->OTYPER, 0b1 <<  6, 0b0 <<  6);
            // GPIOD MODER: Set alternate mode = 0b10 to pins PD6
            MODIFY_REG(GPIOD->MODER, 0b11 << 12, 0b10 << 12);
            break;

          default:
            Error_Handler();
        }
      }

      if ((init_config->function == PERIPH_TX_ONLY) || (init_config->function == PERIPH_TX_RX))
      {

        switch(init_config->pin_tx)
        {
          case H7UART_PIN_USART2_TX_PA2:
            __HAL_RCC_GPIOA_CLK_ENABLE();

            // GPIOA AFRL: Set alternate function USART2 = 7 = 0b0111 (see datasheet chapt 5) to pin PA2 (USART2_TX)
            MODIFY_REG(GPIOA->AFR[0], 0b1111 << 8, 0b0111 << 8);
            // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA2
            MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 4, 0b11 << 4);
            // GPIOA PUPDR: Set pull-up = 0b01 to pin PA2
            MODIFY_REG(GPIOA->PUPDR, 0b11 << 4, 0b01 << 4);
            // GPIOA OTYPEDR: Set no open drain = 0b0 to pin PA2
            MODIFY_REG(GPIOA->OTYPER, 0b1 <<  2, 0b0 <<  2);
            // GPIOA MODER: Set alternate mode = 0b10 to pins PA2
            MODIFY_REG(GPIOA->MODER, 0b11 << 4, 0b10 << 4);
            break;

          case H7UART_PIN_USART2_TX_PD5:
            __HAL_RCC_GPIOD_CLK_ENABLE();

            // GPIOD AFRL: Set alternate function USART2 = 7 = 0b0111 (see datasheet chapt 5) to pin PD5 (USART2_TX)
            MODIFY_REG(GPIOD->AFR[0], 0b1111 << 20, 0b0111 << 20);
            // GPIOD OSPEEDR: Set very high speed = 0b11 to pin PD5
            MODIFY_REG(GPIOD->OSPEEDR, 0b11 << 10, 0b11 << 10);
            // GPIOD PUPDR: Set pull-up = 0b01 to pin PD5
            MODIFY_REG(GPIOD->PUPDR, 0b11 << 10, 0b01 << 10);
            // GPIOD OTYPEDR: Set no open drain = 0b0 to pin PD5
            MODIFY_REG(GPIOD->OTYPER, 0b1 <<  5, 0b0 <<  5);
            // GPIOD MODER: Set alternate mode = 0b10 to pins PD5
            MODIFY_REG(GPIOD->MODER, 0b11 << 10, 0b10 << 10);
            break;

          default:
            Error_Handler();
        }
      }

      //
      // *** INTERRUPT CALLBACK CONFIG.
      //

      //- Global peripheral callback
      h7uart_state_usart2.irq_callback = init_config->irq_callback;

      //- Reception callback
      h7uart_state_usart2.rx_callback  = init_config->rx_callback;

      //
      // *** UART SETUP ***
      //
      __HAL_RCC_C1_USART2_CLK_ENABLE();

      h7uart_uart_reset_peripheral_full(H7UART_USART2);

      // Set the Config Register 1
      MODIFY_REG(USART2->CR1,
              USART_CR1_M1             | USART_CR1_EOBIE  | USART_CR1_RTOIE
            | USART_CR1_DEAT           | USART_CR1_DEDT   | USART_CR1_OVER8  | USART_CR1_CMIE | USART_CR1_MME           | USART_CR1_M0
            | USART_CR1_WAKE           | USART_CR1_PCE    | USART_CR1_PS     | USART_CR1_PEIE | USART_CR1_TXEIE_TXFNFIE | USART_CR1_TCIE
            | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_IDLEIE | USART_CR1_TE     | USART_CR1_RE   | USART_CR1_UESM          | USART_CR1_UE,
          cr1_value);

      // Set the Config Register 2
      MODIFY_REG(USART2->CR2,
              USART_CR2_ADD     | USART_CR2_RTOEN | USART_CR2_ABRMODE | USART_CR2_ABREN | USART_CR2_MSBFIRST | USART_CR2_DATAINV
            | USART_CR2_TXINV   | USART_CR2_RXINV | USART_CR2_SWAP    | USART_CR2_LINEN | USART_CR2_STOP     | USART_CR2_CLKEN
            | USART_CR2_CPOL    | USART_CR2_CPHA  | USART_CR2_LBCL    | USART_CR2_LBDIE | USART_CR2_LBDL     | USART_CR2_ADDM7
            | USART_CR2_DIS_NSS | USART_CR2_SLVEN,
          cr2_value);

      // Set the Config Register 3
      MODIFY_REG(USART2->CR3,
          USART_CR3_TCBGTIE | USART_CR3_WUFIE
        | USART_CR3_WUS     | USART_CR3_SCARCNT | USART_CR3_DEP     | USART_CR3_DEM     | USART_CR3_DDRE   | USART_CR3_OVRDIS
        | USART_CR3_ONEBIT  | USART_CR3_CTSIE   | USART_CR3_CTSE    | USART_CR3_RTSE    | USART_CR3_DMAT   | USART_CR3_DMAR
        | USART_CR3_SCEN    | USART_CR3_NACK    | USART_CR3_HDSEL   | USART_CR3_IRLP    | USART_CR3_IREN   | USART_CR3_EIE,
        cr3_value);

      // Set the Guard Time and Prescaler Register
      // ( It is related to Smartcard function which is not implemented by the present driver )
      MODIFY_REG(USART2->GTPR,USART_GTPR_GT | USART_GTPR_PSC,0UL);

      // Set Receiver Timeout Register
      // - BLEN: It is used only in Smartcard mode which is not implmented by the present driver
      // - RTO: Receive timeout value it is not used in the present driver.
      MODIFY_REG(USART2->RTOR,USART_RTOR_BLEN | USART_RTOR_RTO,0UL);

      // Set Baud Rate Configuration

      // - Kernel Clock after prescaler
      usart_ker_ckpres = USART_CLOCK_KER_USART_2345678 / presc_value;

      // - Compute baud rate register value
      brr_value = usart_ker_ckpres / init_config->baud_rate;

      // Checks if the resulting brr values fits in 16-bits
      if ( brr_value != ( brr_value & 0x0000FFFF ) )
        Error_Handler();

      // Set Prescaler Register
      MODIFY_REG(USART2->PRESC,USART_PRESC_PRESCALER,presc_value);

      // Set Baud Rate Register
      MODIFY_REG(USART2->BRR,USART_BRR_DIV_FRACTION | USART_BRR_DIV_MANTISSA,brr_value);

      //
      //  *** NVIC SETUP ***
      //
      HAL_NVIC_SetPriority(USART2_IRQn, H7UART_IRQ_USART2_PRI, H7UART_IRQ_USART2_SUBPRI);
      HAL_NVIC_EnableIRQ(USART2_IRQn);

      // Peripheral enable
      SET_BIT(USART2->CR1,USART_CR1_UE);
      READ_REG(USART2->CR1);

      // Update driver state
      h7uart_state_usart2.fsm_state = H7UART_FSM_STATE_IDLE;

      break;
#endif

#if H7UART_PERIPH_ENABLE_USART3  == 1
    case H7UART_USART3:
      //
      // *** GPIO PIN SETUP ***
      //

      if ((init_config->function == PERIPH_RX_ONLY) || (init_config->function == PERIPH_TX_RX))
      {

        switch(init_config->pin_rx)
        {
          case H7UART_PIN_USART3_RX_PB11:
            __HAL_RCC_GPIOB_CLK_ENABLE();

            // GPIOB AFRL: Set alternate function USART3 = 7 = 0b0111 (see datasheet chapt 5) to pin PB11 (USART3_RX)
            MODIFY_REG(GPIOB->AFR[1], 0b1111 << 12, 0b0111 << 12);
            // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB11
            MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 22, 0b11 << 22);
            // GPIOB PUPDR: Set pull-up = 0b01 to pin PB11
            MODIFY_REG(GPIOB->PUPDR, 0b11 << 22, 0b01 << 22);
            // GPIOB OTYPEDR: Set no open drain = 0b0 to pin PB11
            MODIFY_REG(GPIOB->OTYPER, 0b1 << 11, 0b0 << 11);
            // GPIOB MODER: Set alternate mode = 0b10 to pins PB11
            MODIFY_REG(GPIOB->MODER, 0b11 << 22, 0b10 << 22);
            break;

          case H7UART_PIN_USART3_RX_PC11:
            __HAL_RCC_GPIOC_CLK_ENABLE();

            // GPIOC AFRL: Set alternate function USART3 = 7 = 0b0111 (see datasheet chapt 5) to pin PC11 (USART3_RX)
            MODIFY_REG(GPIOC->AFR[1], 0b1111 << 12, 0b0111 << 12);
            // GPIOC OSPEEDR: Set very high speed = 0b11 to pin PB11
            MODIFY_REG(GPIOC->OSPEEDR, 0b11 << 22, 0b11 << 22);
            // GPIOC PUPDR: Set pull-up = 0b01 to pin PB11
            MODIFY_REG(GPIOC->PUPDR, 0b11 << 22, 0b01 << 22);
            // GPIOC OTYPEDR: Set no open drain = 0b0 to pin PB11
            MODIFY_REG(GPIOC->OTYPER, 0b1 << 11, 0b0 << 11);
            // GPIOC MODER: Set alternate mode = 0b10 to pins PB11
            MODIFY_REG(GPIOC->MODER, 0b11 << 22, 0b10 << 22);
            break;

          case H7UART_PIN_USART3_RX_PD9:
            __HAL_RCC_GPIOD_CLK_ENABLE();

            // GPIOD AFRL: Set alternate function USART3 = 7 = 0b0111 (see datasheet chapt 5) to pin PD9 (USART3_RX)
            MODIFY_REG(GPIOD->AFR[1], 0b1111 << 4, 0b0111 << 4);
            // GPIOD OSPEEDR: Set very high speed = 0b11 to pin PD9
            MODIFY_REG(GPIOD->OSPEEDR, 0b11 << 18, 0b11 << 18);
            // GPIOD PUPDR: Set pull-up = 0b01 to pin PD9
            MODIFY_REG(GPIOD->PUPDR, 0b11 << 18, 0b01 << 18);
            // GPIOD OTYPEDR: Set no open drain = 0b0 to pin PD9
            MODIFY_REG(GPIOD->OTYPER, 0b1 << 9, 0b0 << 9);
            // GPIOD MODER: Set alternate mode = 0b10 to pins PD9
            MODIFY_REG(GPIOD->MODER, 0b11 << 18, 0b10 << 18);
            break;

          default:
            Error_Handler();
        }
      }

      if ((init_config->function == PERIPH_TX_ONLY) || (init_config->function == PERIPH_TX_RX))
      {
        switch(init_config->pin_tx)
        {
          case H7UART_PIN_USART3_TX_PB10:
            __HAL_RCC_GPIOB_CLK_ENABLE();

            // GPIOB AFRL: Set alternate function USART3 = 7 = 0b0111 (see datasheet chapt 5) to pin PB10 (USART3_TX)
            MODIFY_REG(GPIOB->AFR[1], 0b1111 << 8, 0b0111 << 8);
            // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB10
            MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 20, 0b11 << 20);
            // GPIOB PUPDR: Set pull-up = 0b01 to pin PB10
            MODIFY_REG(GPIOB->PUPDR, 0b11 << 20, 0b01 << 20);
            // GPIOB OTYPEDR: Set no open drain = 0b0 to pin PB10
            MODIFY_REG(GPIOB->OTYPER, 0b1 << 10, 0b0 << 10);
            // GPIOB MODER: Set alternate mode = 0b10 to pins PB10
            MODIFY_REG(GPIOB->MODER, 0b11 << 20, 0b10 << 20);
            break;

          case H7UART_PIN_USART3_TX_PC10:
            __HAL_RCC_GPIOC_CLK_ENABLE();

            // GPIOC AFRL: Set alternate function USART3 = 7 = 0b0111 (see datasheet chapt 5) to pin PC10 (USART3_TX)
            MODIFY_REG(GPIOC->AFR[1], 0b1111 << 8, 0b0111 << 8);
            // GPIOC OSPEEDR: Set very high speed = 0b11 to pin PC10
            MODIFY_REG(GPIOC->OSPEEDR, 0b11 << 20, 0b11 << 20);
            // GPIOC PUPDR: Set pull-up = 0b01 to pin PC10
            MODIFY_REG(GPIOC->PUPDR, 0b11 << 20, 0b01 << 20);
            // GPIOC OTYPEDR: Set no open drain = 0b0 to pin PC10
            MODIFY_REG(GPIOC->OTYPER, 0b1 << 10, 0b0 << 10);
            // GPIOC MODER: Set alternate mode = 0b10 to pins PC10
            MODIFY_REG(GPIOC->MODER, 0b11 << 20, 0b10 << 20);
            break;

          case H7UART_PIN_USART3_TX_PD8:
            __HAL_RCC_GPIOD_CLK_ENABLE();

            // GPIOD AFRL: Set alternate function USART3 = 7 = 0b0111 (see datasheet chapt 5) to pin PD8 (USART3_TX)
            MODIFY_REG(GPIOD->AFR[0], 0b1111 << 0, 0b0111 << 0);
            // GPIOD OSPEEDR: Set very high speed = 0b11 to pin PD8
            MODIFY_REG(GPIOD->OSPEEDR, 0b11 << 16, 0b11 << 16);
            // GPIOD PUPDR: Set pull-up = 0b01 to pin PD8
            MODIFY_REG(GPIOD->PUPDR, 0b11 << 16, 0b01 << 16);
            // GPIOD OTYPEDR: Set no open drain = 0b0 to pin PD8
            MODIFY_REG(GPIOD->OTYPER, 0b1 << 8, 0b0 << 8);
            // GPIOD MODER: Set alternate mode = 0b10 to pins PD8
            MODIFY_REG(GPIOD->MODER, 0b11 << 16, 0b10 << 16);
            break;

          default:
            Error_Handler();
        }
      }

      //
      // *** INTERRUPT CALLBACK CONFIG.
      //

      //- Global peripheral callback
      h7uart_state_usart3.irq_callback = init_config->irq_callback;

      //- Reception callback
      h7uart_state_usart3.rx_callback  = init_config->rx_callback;


      //
      // *** UART SETUP ***
      //
      __HAL_RCC_C1_USART3_CLK_ENABLE();

      h7uart_uart_reset_peripheral_full(H7UART_USART3);

      // Set the Config Register 1
      MODIFY_REG(USART3->CR1,
            USART_CR1_M1             | USART_CR1_EOBIE  | USART_CR1_RTOIE
          | USART_CR1_DEAT           | USART_CR1_DEDT   | USART_CR1_OVER8  | USART_CR1_CMIE | USART_CR1_MME           | USART_CR1_M0
          | USART_CR1_WAKE           | USART_CR1_PCE    | USART_CR1_PS     | USART_CR1_PEIE | USART_CR1_TXEIE_TXFNFIE | USART_CR1_TCIE
          | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_IDLEIE | USART_CR1_TE     | USART_CR1_RE   | USART_CR1_UESM          | USART_CR1_UE,
          cr1_value);

      // Set the Config Register 2
      MODIFY_REG(USART3->CR2,
            USART_CR2_ADD     | USART_CR2_RTOEN | USART_CR2_ABRMODE | USART_CR2_ABREN | USART_CR2_MSBFIRST | USART_CR2_DATAINV
          | USART_CR2_TXINV   | USART_CR2_RXINV | USART_CR2_SWAP    | USART_CR2_LINEN | USART_CR2_STOP     | USART_CR2_CLKEN
          | USART_CR2_CPOL    | USART_CR2_CPHA  | USART_CR2_LBCL    | USART_CR2_LBDIE | USART_CR2_LBDL     | USART_CR2_ADDM7
          | USART_CR2_DIS_NSS | USART_CR2_SLVEN,
          cr2_value);

      // Set the Config Register 3
      MODIFY_REG(USART3->CR3,
          USART_CR3_TCBGTIE | USART_CR3_WUFIE
        | USART_CR3_WUS     | USART_CR3_SCARCNT | USART_CR3_DEP     | USART_CR3_DEM     | USART_CR3_DDRE   | USART_CR3_OVRDIS
        | USART_CR3_ONEBIT  | USART_CR3_CTSIE   | USART_CR3_CTSE    | USART_CR3_RTSE    | USART_CR3_DMAT   | USART_CR3_DMAR
        | USART_CR3_SCEN    | USART_CR3_NACK    | USART_CR3_HDSEL   | USART_CR3_IRLP    | USART_CR3_IREN   | USART_CR3_EIE,
        cr3_value);

      // Set the Guard Time and Prescaler Register
      // ( It is related to Smartcard function which is not implemented by the present driver )
      MODIFY_REG(USART3->GTPR,USART_GTPR_GT | USART_GTPR_PSC,0UL);

      // Set Receiver Timeout Register
      // - BLEN: It is used only in Smartcard mode which is not implmented by the present driver
      // - RTO: Receive timeout value it is not used in the present driver.
      MODIFY_REG(USART3->RTOR,USART_RTOR_BLEN | USART_RTOR_RTO,0UL);

      // Set Baud Rate Configuration

      // - Kernel Clock after prescaler
      usart_ker_ckpres = USART_CLOCK_KER_USART_2345678 / presc_value;

      // - Compute baud rate register value
      brr_value = usart_ker_ckpres / init_config->baud_rate;

      // Checks if the resulting brr values fits in 16-bits
      if ( brr_value != ( brr_value & 0x0000FFFF ) )
        Error_Handler();

      // Set Prescaler Register
      MODIFY_REG(USART3->PRESC,USART_PRESC_PRESCALER,presc_value);

      // Set Baud Rate Register
      MODIFY_REG(USART3->BRR,USART_BRR_DIV_FRACTION | USART_BRR_DIV_MANTISSA,brr_value);

      //
      //  *** NVIC SETUP ***
      //
      HAL_NVIC_SetPriority(USART3_IRQn, H7UART_IRQ_USART3_PRI, H7UART_IRQ_USART3_SUBPRI);
      HAL_NVIC_EnableIRQ(USART3_IRQn);

      // Peripheral enable
      SET_BIT(USART3->CR1,USART_CR1_UE);
      READ_REG(USART3->CR1);

      // Update driver state
      h7uart_state_usart3.fsm_state = H7UART_FSM_STATE_IDLE;
      break;
#endif

#if H7UART_PERIPH_ENABLE_UART4   == 1
    case H7UART_UART4:
      //
      // *** GPIO PIN SETUP ***
      //

      if ((init_config->function == PERIPH_RX_ONLY) || (init_config->function == PERIPH_TX_RX))
      {
        switch(init_config->pin_rx)
        {
          case H7UART_PIN_UART4_RX_PA1:
            __HAL_RCC_GPIOA_CLK_ENABLE();

            // GPIOA AFRL: Set alternate function UART4 = 8 = 0b1000 (see datasheet chapt 5) to pin PA1 (UART4_RX)
            MODIFY_REG(GPIOA->AFR[0], 0b1111 << 4, 0b1000 << 4);
            // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA1
            MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 2, 0b11 << 2);
            // GPIOA PUPDR: Set pull-up = 0b01 to pin PA1
            MODIFY_REG(GPIOA->PUPDR, 0b11 << 2, 0b01 << 2);
            // GPIOA OTYPED: Set no open drain = 0b0 to pin PA1
            MODIFY_REG(GPIOA->OTYPER, 0b1 << 1, 0b0 << 1);
            // GPIOA MODER: Set alternate mode = 0b10 to pins PA1
            MODIFY_REG(GPIOA->MODER, 0b11 << 2, 0b10 << 2);
            break;

          case H7UART_PIN_UART4_RX_PA11:
            __HAL_RCC_GPIOA_CLK_ENABLE();

            // GPIOA AFRL: Set alternate function UART4 = 6 = 0b0110 (see datasheet chapt 5) to pin PA11 (UART4_RX)
            MODIFY_REG(GPIOA->AFR[1], 0b1111 << 12, 0b0110 << 12);
            // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA11
            MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 22, 0b11 << 22);
            // GPIOA PUPDR: Set pull-up = 0b01 to pin PA11
            MODIFY_REG(GPIOA->PUPDR, 0b11 << 22, 0b01 << 22);
            // GPIOA OTYPED: Set no open drain = 0b0 to pin PA11
            MODIFY_REG(GPIOA->OTYPER, 0b1 << 11, 0b0 << 11);
            // GPIOA MODER: Set alternate mode = 0b10 to pins PA11
            MODIFY_REG(GPIOA->MODER, 0b11 << 22, 0b10 << 22);
            break;

          case H7UART_PIN_UART4_RX_PB8:
            __HAL_RCC_GPIOB_CLK_ENABLE();

            // GPIOB AFRL: Set alternate function UART4 = 8 = 0b1000 (see datasheet chapt 5) to pin PB8 (UART4_RX)
            MODIFY_REG(GPIOB->AFR[1], 0b1111 << 0, 0b1000 << 0);
            // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB8
            MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 16, 0b11 << 16);
            // GPIOB PUPDR: Set pull-up = 0b01 to pin PB8
            MODIFY_REG(GPIOB->PUPDR, 0b11 << 16, 0b01 << 16);
            // GPIOB OTYPED: Set no open drain = 0b0 to pin PB8
            MODIFY_REG(GPIOB->OTYPER, 0b1 << 8, 0b0 << 8);
            // GPIOB MODER: Set alternate mode = 0b10 to pins PB8
            MODIFY_REG(GPIOB->MODER, 0b11 << 16, 0b10 << 16);
            break;

          case H7UART_PIN_UART4_RX_PC11:
            __HAL_RCC_GPIOC_CLK_ENABLE();

            // GPIOC AFRL: Set alternate function UART4 = 8 = 0b1000 (see datasheet chapt 5) to pin PC11 (UART4_RX)
            MODIFY_REG(GPIOC->AFR[1], 0b1111 << 12, 0b1000 << 12);
            // GPIOC OSPEEDR: Set very high speed = 0b11 to pin PC11
            MODIFY_REG(GPIOC->OSPEEDR, 0b11 << 22, 0b11 << 22);
            // GPIOC PUPDR: Set pull-up = 0b01 to pin PC11
            MODIFY_REG(GPIOC->PUPDR, 0b11 << 22, 0b01 << 22);
            // GPIOC OTYPED: Set no open drain = 0b0 to pin PC11
            MODIFY_REG(GPIOC->OTYPER, 0b1 << 11, 0b0 << 11);
            // GPIOC MODER: Set alternate mode = 0b10 to pins PC11
            MODIFY_REG(GPIOC->MODER, 0b11 << 22, 0b10 << 22);
            break;

          case H7UART_PIN_UART4_RX_PD0:
            __HAL_RCC_GPIOD_CLK_ENABLE();

            // GPIOD AFRL: Set alternate function UART4 = 8 = 0b1000 (see datasheet chapt 5) to pin PD0 (UART4_RX)
            MODIFY_REG(GPIOD->AFR[1], 0b1111 << 0, 0b1000 << 0);
            // GPIOD OSPEEDR: Set very high speed = 0b11 to pin PD0
            MODIFY_REG(GPIOD->OSPEEDR, 0b11 << 0, 0b11 << 0);
            // GPIOD PUPDR: Set pull-up = 0b01 to pin PD0
            MODIFY_REG(GPIOD->PUPDR, 0b11 << 0, 0b01 << 0);
            // GPIOD OTYPED: Set no open drain = 0b0 to pin PD0
            MODIFY_REG(GPIOD->OTYPER, 0b1 << 0, 0b0 << 0);
            // GPIOD MODER: Set alternate mode = 0b10 to pins PD0
            MODIFY_REG(GPIOD->MODER, 0b11 << 0, 0b10 << 0);
            break;

          case H7UART_PIN_UART4_RX_PH14:
            __HAL_RCC_GPIOH_CLK_ENABLE();

            // GPIOH AFRL: Set alternate function UART4 = 8 = 0b1000 (see datasheet chapt 5) to pin PH14 (UART4_RX)
            MODIFY_REG(GPIOH->AFR[1], 0b1111 << 24, 0b1000 << 24);
            // GPIOH OSPEEDR: Set very high speed = 0b11 to pin PH14
            MODIFY_REG(GPIOH->OSPEEDR, 0b11 << 28, 0b11 << 28);
            // GPIOH PUPDR: Set pull-up = 0b01 to pin PH14
            MODIFY_REG(GPIOH->PUPDR, 0b11 << 28, 0b01 << 28);
            // GPIOH OTYPED: Set no open drain = 0b0 to pin PH14
            MODIFY_REG(GPIOH->OTYPER, 0b1 << 14, 0b0 << 14);
            // GPIOH MODER: Set alternate mode = 0b10 to pins PH14
            MODIFY_REG(GPIOH->MODER, 0b11 << 28, 0b10 << 28);
            break;

          case H7UART_PIN_UART4_RX_PI9:
            __HAL_RCC_GPIOI_CLK_ENABLE();

            // GPIOI AFRL: Set alternate function UART4 = 8 = 0b1000 (see datasheet chapt 5) to pin PI9 (UART4_RX)
            MODIFY_REG(GPIOI->AFR[1], 0b1111 << 4, 0b1000 << 4);
            // GPIOI OSPEEDR: Set very high speed = 0b11 to pin PI9
            MODIFY_REG(GPIOI->OSPEEDR, 0b11 << 18, 0b11 << 18);
            // GPIOI PUPDR: Set pull-up = 0b01 to pin PI9
            MODIFY_REG(GPIOI->PUPDR, 0b11 << 18, 0b01 << 18);
            // GPIOI OTYPED: Set no open drain = 0b0 to pin PI9
            MODIFY_REG(GPIOI->OTYPER, 0b1 << 9, 0b0 << 9);
            // GPIOI MODER: Set alternate mode = 0b10 to pins PI9
            MODIFY_REG(GPIOI->MODER, 0b11 << 18, 0b10 << 18);
            break;

          default:
            Error_Handler();
        }
      }

      if ((init_config->function == PERIPH_TX_ONLY) || (init_config->function == PERIPH_TX_RX))
      {
        switch(init_config->pin_tx)
        {
          case H7UART_PIN_UART4_TX_PA0:
            __HAL_RCC_GPIOA_CLK_ENABLE();

            // GPIOA AFRL: Set alternate function UART4 = 8 = 0b1000 (see datasheet chapt 5) to pin PA0 (UART4_TX)
            MODIFY_REG(GPIOA->AFR[0], 0b1111 << 0, 0b1000 << 0);
            // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA0
            MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 0, 0b11 << 0);
            // GPIOA PUPDR: Set pull-up = 0b01 to pin PA0
            MODIFY_REG(GPIOA->PUPDR, 0b11 << 0, 0b01 << 0);
            // GPIOA OTYPED: Set no open drain = 0b0 to pin PA0
            MODIFY_REG(GPIOA->OTYPER, 0b1 << 0, 0b0 << 0);
            // GPIOA MODER: Set alternate mode = 0b10 to pins PA0
            MODIFY_REG(GPIOA->MODER, 0b11 << 0, 0b10 << 0);
            break;

          case H7UART_PIN_UART4_TX_PA12:
            __HAL_RCC_GPIOA_CLK_ENABLE();

            // GPIOA AFRL: Set alternate function UART4 = 6 = 0b0110 (see datasheet chapt 5) to pin PA12 (UART4_TX)
            MODIFY_REG(GPIOA->AFR[1], 0b1111 << 16, 0b0110 << 16);
            // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA12
            MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 24, 0b11 << 24);
            // GPIOA PUPDR: Set pull-up = 0b01 to pin PA12
            MODIFY_REG(GPIOA->PUPDR, 0b11 << 24, 0b01 << 24);
            // GPIOA OTYPED: Set no open drain = 0b0 to pin PA12
            MODIFY_REG(GPIOA->OTYPER, 0b1 << 12, 0b0 << 12);
            // GPIOA MODER: Set alternate mode = 0b10 to pins PA12
            MODIFY_REG(GPIOA->MODER, 0b11 << 24, 0b10 << 24);
            break;

          case H7UART_PIN_UART4_TX_PB9:
            __HAL_RCC_GPIOB_CLK_ENABLE();

            // GPIOA AFRL: Set alternate function UART4 = 6 = 0b0110 (see datasheet chapt 5) to pin PB9 (UART4_TX)
            MODIFY_REG(GPIOA->AFR[1], 0b1111 << 16, 0b0110 << 16);
            // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PB9
            MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 24, 0b11 << 24);
            // GPIOA PUPDR: Set pull-up = 0b01 to pin PB9
            MODIFY_REG(GPIOA->PUPDR, 0b11 << 24, 0b01 << 24);
            // GPIOA OTYPED: Set no open drain = 0b0 to pin PB9
            MODIFY_REG(GPIOA->OTYPER, 0b1 << 12, 0b0 << 12);
            // GPIOA MODER: Set alternate mode = 0b10 to pins PB9
            MODIFY_REG(GPIOA->MODER, 0b11 << 24, 0b10 << 24);
            break;

          case H7UART_PIN_UART4_TX_PC10:
            __HAL_RCC_GPIOC_CLK_ENABLE();

            // GPIOC AFRL: Set alternate function UART4 = 7 = 0b0111 (see datasheet chapt 5) to pin PC10 (UART4_TX)
            MODIFY_REG(GPIOC->AFR[1], 0b1111 << 8, 0b0111 << 8);
            // GPIOC OSPEEDR: Set very high speed = 0b11 to pin PC10
            MODIFY_REG(GPIOC->OSPEEDR, 0b11 << 20, 0b11 << 20);
            // GPIOC PUPDR: Set pull-up = 0b01 to pin PC10
            MODIFY_REG(GPIOC->PUPDR, 0b11 << 20, 0b01 << 20);
            // GPIOC OTYPED: Set no open drain = 0b0 to pin PC10
            MODIFY_REG(GPIOC->OTYPER, 0b1 << 10, 0b0 << 10);
            // GPIOC MODER: Set alternate mode = 0b10 to pins PC10
            MODIFY_REG(GPIOC->MODER, 0b11 << 20, 0b10 << 20);

            break;

          case H7UART_PIN_UART4_TX_PD1:
            __HAL_RCC_GPIOD_CLK_ENABLE();

            // GPIOD AFRL: Set alternate function UART4 = 8 = 0b1000 (see datasheet chapt 5) to pin PD1 (UART4_TX)
            MODIFY_REG(GPIOD->AFR[0], 0b1111 << 4, 0b0111 << 4);
            // GPIOD OSPEEDR: Set very high speed = 0b11 to pin PD1
            MODIFY_REG(GPIOD->OSPEEDR, 0b11 << 2, 0b11 << 2);
            // GPIOD PUPDR: Set pull-up = 0b01 to pin PD1
            MODIFY_REG(GPIOD->PUPDR, 0b11 << 2, 0b01 << 2);
            // GPIOD OTYPED: Set no open drain = 0b0 to pin PD1
            MODIFY_REG(GPIOD->OTYPER, 0b1 << 1, 0b0 << 1);
            // GPIOD MODER: Set alternate mode = 0b10 to pin PD1
            MODIFY_REG(GPIOD->MODER, 0b11 << 2, 0b10 << 2);
            break;

          case H7UART_PIN_UART4_TX_PH13:
            __HAL_RCC_GPIOH_CLK_ENABLE();

            // GPIOH AFRL: Set alternate function UART4 = 8 = 0b1000 (see datasheet chapt 5) to pin PH13 (UART4_TX)
            MODIFY_REG(GPIOH->AFR[1], 0b1111 << 20, 0b1000 << 20);
            // GPIOH OSPEEDR: Set very high speed = 0b11 to pin PH13
            MODIFY_REG(GPIOH->OSPEEDR, 0b11 << 26, 0b11 << 26);
            // GPIOH PUPDR: Set pull-up = 0b01 to pin PH13
            MODIFY_REG(GPIOH->PUPDR, 0b11 << 26, 0b01 << 26);
            // GPIOH OTYPED: Set no open drain = 0b0 to pin PH13
            MODIFY_REG(GPIOH->OTYPER, 0b1 << 13, 0b0 << 13);
            // GPIOH MODER: Set alternate mode = 0b10 to pin PH13
            MODIFY_REG(GPIOH->MODER, 0b11 << 26, 0b10 << 26);
            break;

          default:
            Error_Handler();
        }
      }

      //
      // *** INTERRUPT CALLBACK CONFIG.
      //

      //- Global peripheral callback
      h7uart_state_uart4.irq_callback = init_config->irq_callback;

      //- Reception callback
      h7uart_state_uart4.rx_callback  = init_config->rx_callback;

      //
      // *** UART SETUP ***
      //

      __HAL_RCC_C1_UART4_CLK_ENABLE();

      h7uart_uart_reset_peripheral_full(H7UART_UART4);

      // Set the Config Register 1
      MODIFY_REG(UART4->CR1,
            USART_CR1_M1             | USART_CR1_EOBIE  | USART_CR1_RTOIE
          | USART_CR1_DEAT           | USART_CR1_DEDT   | USART_CR1_OVER8  | USART_CR1_CMIE | USART_CR1_MME           | USART_CR1_M0
          | USART_CR1_WAKE           | USART_CR1_PCE    | USART_CR1_PS     | USART_CR1_PEIE | USART_CR1_TXEIE_TXFNFIE | USART_CR1_TCIE
          | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_IDLEIE | USART_CR1_TE     | USART_CR1_RE   | USART_CR1_UESM          | USART_CR1_UE,
          cr1_value);

      // Set the Config Register 2
      MODIFY_REG(UART4->CR2,
            USART_CR2_ADD     | USART_CR2_RTOEN | USART_CR2_ABRMODE | USART_CR2_ABREN | USART_CR2_MSBFIRST | USART_CR2_DATAINV
          | USART_CR2_TXINV   | USART_CR2_RXINV | USART_CR2_SWAP    | USART_CR2_LINEN | USART_CR2_STOP     | USART_CR2_CLKEN
          | USART_CR2_CPOL    | USART_CR2_CPHA  | USART_CR2_LBCL    | USART_CR2_LBDIE | USART_CR2_LBDL     | USART_CR2_ADDM7
          | USART_CR2_DIS_NSS | USART_CR2_SLVEN,
          cr2_value);

      // Set the Config Register 3
      MODIFY_REG(UART4->CR3,
          USART_CR3_TCBGTIE | USART_CR3_WUFIE
        | USART_CR3_WUS     | USART_CR3_SCARCNT | USART_CR3_DEP     | USART_CR3_DEM     | USART_CR3_DDRE   | USART_CR3_OVRDIS
        | USART_CR3_ONEBIT  | USART_CR3_CTSIE   | USART_CR3_CTSE    | USART_CR3_RTSE    | USART_CR3_DMAT   | USART_CR3_DMAR
        | USART_CR3_SCEN    | USART_CR3_NACK    | USART_CR3_HDSEL   | USART_CR3_IRLP    | USART_CR3_IREN   | USART_CR3_EIE,
        cr3_value);

      // Set the Guard Time and Prescaler Register
      // ( It is related to Smartcard function which is not implemented by the present driver )
      MODIFY_REG(UART4->GTPR,USART_GTPR_GT | USART_GTPR_PSC,0UL);

      // Set Receiver Timeout Register
      // - BLEN: It is used only in Smartcard mode which is not implmented by the present driver
      // - RTO: Receive timeout value it is not used in the present driver.
      MODIFY_REG(UART4->RTOR,USART_RTOR_BLEN | USART_RTOR_RTO,0UL);

      // Set Baud Rate Configuration

      // - Kernel Clock after prescaler
      usart_ker_ckpres = USART_CLOCK_KER_USART_2345678 / presc_value;

      // - Compute baud rate register value
      brr_value = usart_ker_ckpres / init_config->baud_rate;

      // Checks if the resulting brr values fits in 16-bits
      if ( brr_value != ( brr_value & 0x0000FFFF ) )
        Error_Handler();

      // Set Prescaler Register
      MODIFY_REG(UART4->PRESC,USART_PRESC_PRESCALER,presc_value);

      // Set Baud Rate Register
      MODIFY_REG(UART4->BRR,USART_BRR_DIV_FRACTION | USART_BRR_DIV_MANTISSA,brr_value);

      //
      //  *** NVIC SETUP ***
      //
      HAL_NVIC_SetPriority(UART4_IRQn, H7UART_IRQ_UART4_PRI, H7UART_IRQ_UART4_SUBPRI);
      HAL_NVIC_EnableIRQ(UART4_IRQn);

      // Peripheral enable
      SET_BIT(UART4->CR1,USART_CR1_UE);
      READ_REG(UART4->CR1);

      // Update driver state
      h7uart_state_uart4.fsm_state = H7UART_FSM_STATE_IDLE;

      break;
#endif

#if H7UART_PERIPH_ENABLE_UART5   == 1
    case H7UART_UART5:
      //
      // *** GPIO PIN SETUP ***
      //

      if ((init_config->function == PERIPH_RX_ONLY) || (init_config->function == PERIPH_TX_RX))
      {
        switch(init_config->pin_rx)
        {
          case H7UART_PIN_UART5_RX_PD2:
            __HAL_RCC_GPIOD_CLK_ENABLE();

            // GPIOD AFRL: Set alternate function UART5 = 8 = 0b1000 (see datasheet chapt 5) to pin PD2 (UART5_RX)
            MODIFY_REG(GPIOD->AFR[1], 0b1111 << 8, 0b1000 << 8);
            // GPIOD OSPEEDR: Set very high speed = 0b11 to pin PD2
            MODIFY_REG(GPIOD->OSPEEDR, 0b11 << 4, 0b11 << 4);
            // GPIOD PUPDR: Set pull-up = 0b01 to pin PD2
            MODIFY_REG(GPIOD->PUPDR, 0b11 << 4, 0b01 << 4);
            // GPIOD OTYPED: Set no open drain = 0b0 to pin PD2
            MODIFY_REG(GPIOD->OTYPER, 0b1 << 2, 0b0 << 2);
            // GPIOD MODER: Set alternate mode = 0b10 to pin PD2
            MODIFY_REG(GPIOD->MODER, 0b11 << 4, 0b10 << 4);
            break;

          default:
            Error_Handler();
        }

        switch(init_config->pin_tx)
        {
          case H7UART_PIN_UART5_TX_PC12:
            __HAL_RCC_GPIOC_CLK_ENABLE();

            // GPIOC AFRL: Set alternate function UART5 = 8 = 0b1000 (see datasheet chapt 5) to pin PC12 (UART5_TX)
            MODIFY_REG(GPIOC->AFR[1], 0b1111 << 16, 0b1000 << 16);
            // GPIOC OSPEEDR: Set very high speed = 0b11 to pin PC12
            MODIFY_REG(GPIOC->OSPEEDR, 0b11 << 24, 0b11 << 24);
            // GPIOC PUPDR: Set pull-up = 0b01 to pin PC12
            MODIFY_REG(GPIOC->PUPDR, 0b11 << 24, 0b01 << 24);
            // GPIOC OTYPED: Set no open drain = 0b0 to pin PC12
            MODIFY_REG(GPIOC->OTYPER, 0b1 << 12, 0b0 << 12);
            // GPIOC MODER: Set alternate mode = 0b10 to pin PC12
            MODIFY_REG(GPIOC->MODER, 0b11 << 24, 0b10 << 24);
            break;

          default:
            Error_Handler();
        }
      }

      //
      // *** INTERRUPT CALLBACK CONFIG.
      //

      //- Global peripheral callback
      h7uart_state_uart5.irq_callback = init_config->irq_callback;

      //- Reception callback
      h7uart_state_uart5.rx_callback  = init_config->rx_callback;

      //
      // *** UART SETUP ***
      //
      __HAL_RCC_C1_UART5_CLK_ENABLE();

      h7uart_uart_reset_peripheral_full(H7UART_UART5);

      // Set the Config Register 1
      MODIFY_REG(UART5->CR1,
            USART_CR1_M1             | USART_CR1_EOBIE  | USART_CR1_RTOIE
          | USART_CR1_DEAT           | USART_CR1_DEDT   | USART_CR1_OVER8  | USART_CR1_CMIE | USART_CR1_MME           | USART_CR1_M0
          | USART_CR1_WAKE           | USART_CR1_PCE    | USART_CR1_PS     | USART_CR1_PEIE | USART_CR1_TXEIE_TXFNFIE | USART_CR1_TCIE
          | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_IDLEIE | USART_CR1_TE     | USART_CR1_RE   | USART_CR1_UESM          | USART_CR1_UE,
          cr1_value);

      // Set the Config Register 2
      MODIFY_REG(UART5->CR2,
            USART_CR2_ADD     | USART_CR2_RTOEN | USART_CR2_ABRMODE | USART_CR2_ABREN | USART_CR2_MSBFIRST | USART_CR2_DATAINV
          | USART_CR2_TXINV   | USART_CR2_RXINV | USART_CR2_SWAP    | USART_CR2_LINEN | USART_CR2_STOP     | USART_CR2_CLKEN
          | USART_CR2_CPOL    | USART_CR2_CPHA  | USART_CR2_LBCL    | USART_CR2_LBDIE | USART_CR2_LBDL     | USART_CR2_ADDM7
          | USART_CR2_DIS_NSS | USART_CR2_SLVEN,
          cr2_value);

      // Set the Config Register 3
      MODIFY_REG(UART5->CR3,
          USART_CR3_TCBGTIE | USART_CR3_WUFIE
        | USART_CR3_WUS     | USART_CR3_SCARCNT | USART_CR3_DEP     | USART_CR3_DEM     | USART_CR3_DDRE   | USART_CR3_OVRDIS
        | USART_CR3_ONEBIT  | USART_CR3_CTSIE   | USART_CR3_CTSE    | USART_CR3_RTSE    | USART_CR3_DMAT   | USART_CR3_DMAR
        | USART_CR3_SCEN    | USART_CR3_NACK    | USART_CR3_HDSEL   | USART_CR3_IRLP    | USART_CR3_IREN   | USART_CR3_EIE,
        cr3_value);

      // Set the Guard Time and Prescaler Register
      // ( It is related to Smartcard function which is not implemented by the present driver )
      MODIFY_REG(UART5->GTPR,USART_GTPR_GT | USART_GTPR_PSC,0UL);

      // Set Receiver Timeout Register
      // - BLEN: It is used only in Smartcard mode which is not implmented by the present driver
      // - RTO: Receive timeout value it is not used in the present driver.
      MODIFY_REG(UART5->RTOR,USART_RTOR_BLEN | USART_RTOR_RTO,0UL);

      // Set Baud Rate Configuration

      // - Kernel Clock after prescaler
      usart_ker_ckpres = USART_CLOCK_KER_USART_2345678 / presc_value;

      // - Compute baud rate register value
      brr_value = usart_ker_ckpres / init_config->baud_rate;

      // Checks if the resulting brr values fits in 16-bits
      if ( brr_value != ( brr_value & 0x0000FFFF ) )
        Error_Handler();

      // Set Prescaler Register
      MODIFY_REG(UART5->PRESC,USART_PRESC_PRESCALER,presc_value);

      // Set Baud Rate Register
      MODIFY_REG(UART5->BRR,USART_BRR_DIV_FRACTION | USART_BRR_DIV_MANTISSA,brr_value);

      //
      //  *** NVIC SETUP ***
      //
      HAL_NVIC_SetPriority(UART5_IRQn, H7UART_IRQ_UART5_PRI, H7UART_IRQ_UART5_SUBPRI);
      HAL_NVIC_EnableIRQ(UART5_IRQn);

      // Peripheral enable
      SET_BIT(UART5->CR1,USART_CR1_UE);
      READ_REG(UART5->CR1);

      // Update driver state
      h7uart_state_uart5.fsm_state = H7UART_FSM_STATE_IDLE;

      break;
#endif

#if H7UART_PERIPH_ENABLE_USART6  == 1
    case H7UART_USART6:
      //
      // *** GPIO PIN SETUP ***
      //

      if ((init_config->function == PERIPH_RX_ONLY) || (init_config->function == PERIPH_TX_RX))
      {
        switch(init_config->pin_rx)
        {
          case H7UART_PIN_USART6_RX_PC7:
            __HAL_RCC_GPIOC_CLK_ENABLE();

            // GPIOC AFRL: Set alternate function USART6 = 7 = 0b0111 (see datasheet chapt 5) to pin PC7 (USART6_RX)
            MODIFY_REG(GPIOC->AFR[0], 0b1111 << 28, 0b0111 << 28);
            // GPIOC OSPEEDR: Set very high speed = 0b11 to pin PC7
            MODIFY_REG(GPIOC->OSPEEDR, 0b11 << 14, 0b11 << 14);
            // GPIOC PUPDR: Set pull-up = 0b01 to pin PC7
            MODIFY_REG(GPIOC->PUPDR, 0b11 << 14, 0b01 << 14);
            // GPIOC OTYPED: Set no open drain = 0b0 to pin PC7
            MODIFY_REG(GPIOC->OTYPER, 0b1 << 7, 0b0 << 7);
            // GPIOC MODER: Set alternate mode = 0b10 to pin PC7
            MODIFY_REG(GPIOC->MODER, 0b11 << 14, 0b10 << 14);
            break;

          case H7UART_PIN_USART6_RX_PG9:
            __HAL_RCC_GPIOG_CLK_ENABLE();

            // GPIOG AFRL: Set alternate function USART6 = 7 = 0b0111 (see datasheet chapt 5) to pin PG9 (USART6_RX)
            MODIFY_REG(GPIOG->AFR[1], 0b1111 << 4, 0b0111 << 4);
            // GPIOG OSPEEDR: Set very high speed = 0b11 to pin PG9
            MODIFY_REG(GPIOG->OSPEEDR, 0b11 << 18, 0b11 << 18);
            // GPIOG PUPDR: Set pull-up = 0b01 to pin PG9
            MODIFY_REG(GPIOG->PUPDR, 0b11 << 18, 0b01 << 18);
            // GPIOC OTYPED: Set no open drain = 0b0 to pin PG9
            MODIFY_REG(GPIOG->OTYPER, 0b1 << 9, 0b0 << 9);
            // GPIOG MODER: Set alternate mode = 0b10 to pin PG9
            MODIFY_REG(GPIOG->MODER, 0b11 << 18, 0b10 << 18);
            break;

          default:
            Error_Handler();
        }
      }

      if ((init_config->function == PERIPH_TX_ONLY) || (init_config->function == PERIPH_TX_RX))
      {
        switch(init_config->pin_tx)
        {
          case H7UART_PIN_USART6_TX_PC6:
            __HAL_RCC_GPIOC_CLK_ENABLE();

            // GPIOC AFRL: Set alternate function USART6 = 7 = 0b0111 (see datasheet chapt 5) to pin PC6 (USART6_TX)
            MODIFY_REG(GPIOC->AFR[0], 0b1111 << 24, 0b0111 << 24);
            // GPIOC OSPEEDR: Set very high speed = 0b11 to pin PC6
            MODIFY_REG(GPIOC->OSPEEDR, 0b11 << 12, 0b11 << 12);
            // GPIOC PUPDR: Set pull-up = 0b01 to pin PC6
            MODIFY_REG(GPIOC->PUPDR, 0b11 << 12, 0b01 << 12);
            // GPIOC OTYPED: Set no open drain = 0b0 to pin PC6
            MODIFY_REG(GPIOC->OTYPER, 0b1 << 6, 0b0 << 6);
            // GPIOC MODER: Set alternate mode = 0b10 to pin PC6
            MODIFY_REG(GPIOC->MODER, 0b11 << 12, 0b10 << 12);
            break;

          case H7UART_PIN_USART6_TX_PG14:
            __HAL_RCC_GPIOG_CLK_ENABLE();

            // GPIOG AFRL: Set alternate function USART6 = 7 = 0b0111 (see datasheet chapt 5) to pin PG14 (USART6_TX)
            MODIFY_REG(GPIOG->AFR[1], 0b1111 << 24, 0b0111 << 24);
            // GPIOG OSPEEDR: Set very high speed = 0b11 to pin PG14
            MODIFY_REG(GPIOG->OSPEEDR, 0b11 << 28, 0b11 << 28);
            // GPIOG PUPDR: Set pull-up = 0b01 to pin PG14
            MODIFY_REG(GPIOG->PUPDR, 0b11 << 28, 0b01 << 28);
            // GPIOC OTYPED: Set no open drain = 0b0 to pin PG14
            MODIFY_REG(GPIOG->OTYPER, 0b1 << 14, 0b0 << 14);
            // GPIOG MODER: Set alternate mode = 0b10 to pin PG14
            MODIFY_REG(GPIOG->MODER, 0b11 << 28, 0b10 << 28);
            break;

          default:
            Error_Handler();
        }
      }

      //
      // *** INTERRUPT CALLBACK CONFIG.
      //

      //- Global peripheral callback
      h7uart_state_usart6.irq_callback = init_config->irq_callback;

      //- Reception callback
      h7uart_state_usart6.rx_callback  = init_config->rx_callback;

      //
      // *** UART SETUP ***
      //

      __HAL_RCC_C1_USART6_CLK_ENABLE();

      h7uart_uart_reset_peripheral_full(H7UART_USART6);

      // Set the Config Register 1
      MODIFY_REG(USART6->CR1,
            USART_CR1_M1             | USART_CR1_EOBIE  | USART_CR1_RTOIE
          | USART_CR1_DEAT           | USART_CR1_DEDT   | USART_CR1_OVER8  | USART_CR1_CMIE | USART_CR1_MME           | USART_CR1_M0
          | USART_CR1_WAKE           | USART_CR1_PCE    | USART_CR1_PS     | USART_CR1_PEIE | USART_CR1_TXEIE_TXFNFIE | USART_CR1_TCIE
          | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_IDLEIE | USART_CR1_TE     | USART_CR1_RE   | USART_CR1_UESM          | USART_CR1_UE,
          cr1_value);

      // Set the Config Register 2
      MODIFY_REG(USART6->CR2,
            USART_CR2_ADD     | USART_CR2_RTOEN | USART_CR2_ABRMODE | USART_CR2_ABREN | USART_CR2_MSBFIRST | USART_CR2_DATAINV
          | USART_CR2_TXINV   | USART_CR2_RXINV | USART_CR2_SWAP    | USART_CR2_LINEN | USART_CR2_STOP     | USART_CR2_CLKEN
          | USART_CR2_CPOL    | USART_CR2_CPHA  | USART_CR2_LBCL    | USART_CR2_LBDIE | USART_CR2_LBDL     | USART_CR2_ADDM7
          | USART_CR2_DIS_NSS | USART_CR2_SLVEN,
          cr2_value);

      // Set the Config Register 3
      MODIFY_REG(USART6->CR3,
          USART_CR3_TCBGTIE | USART_CR3_WUFIE
        | USART_CR3_WUS     | USART_CR3_SCARCNT | USART_CR3_DEP     | USART_CR3_DEM     | USART_CR3_DDRE   | USART_CR3_OVRDIS
        | USART_CR3_ONEBIT  | USART_CR3_CTSIE   | USART_CR3_CTSE    | USART_CR3_RTSE    | USART_CR3_DMAT   | USART_CR3_DMAR
        | USART_CR3_SCEN    | USART_CR3_NACK    | USART_CR3_HDSEL   | USART_CR3_IRLP    | USART_CR3_IREN   | USART_CR3_EIE,
        cr3_value);

      // Set the Guard Time and Prescaler Register
      // ( It is related to Smartcard function which is not implemented by the present driver )
      MODIFY_REG(USART6->GTPR,USART_GTPR_GT | USART_GTPR_PSC,0UL);

      // Set Receiver Timeout Register
      // - BLEN: It is used only in Smartcard mode which is not implmented by the present driver
      // - RTO: Receive timeout value it is not used in the present driver.
      MODIFY_REG(USART6->RTOR,USART_RTOR_BLEN | USART_RTOR_RTO,0UL);

      // Set Baud Rate Configuration

      // - Kernel Clock after prescaler
      usart_ker_ckpres = USART_CLOCK_KER_USART_2345678 / presc_value;

      // - Compute baud rate register value
      brr_value = usart_ker_ckpres / init_config->baud_rate;

      // Checks if the resulting brr values fits in 16-bits
      if ( brr_value != ( brr_value & 0x0000FFFF ) )
        Error_Handler();

      // Set Prescaler Register
      MODIFY_REG(USART6->PRESC,USART_PRESC_PRESCALER,presc_value);

      // Set Baud Rate Register
      MODIFY_REG(USART6->BRR,USART_BRR_DIV_FRACTION | USART_BRR_DIV_MANTISSA,brr_value);

      //
      //  *** NVIC SETUP ***
      //
      HAL_NVIC_SetPriority(USART6_IRQn, H7UART_IRQ_USART6_PRI, H7UART_IRQ_USART6_SUBPRI);
      HAL_NVIC_EnableIRQ(USART6_IRQn);

      // Peripheral enable
      SET_BIT(USART6->CR1,USART_CR1_UE);
      READ_REG(USART6->CR1);

      // Update driver state
      h7uart_state_usart6.fsm_state = H7UART_FSM_STATE_IDLE;

      break;
#endif

#if H7UART_PERIPH_ENABLE_UART7   == 1
    case H7UART_UART7:
      //
      // *** GPIO PIN SETUP ***
      //
      if ((init_config->function == PERIPH_RX_ONLY) || (init_config->function == PERIPH_TX_RX))
      {
        switch(init_config->pin_rx)
        {
          case H7UART_PIN_UART7_RX_PA8:
            __HAL_RCC_GPIOA_CLK_ENABLE();

            // GPIOA AFRL: Set alternate function UART7 = 11 = 0b1011 (see datasheet chapt 5) to pin PA8 (UART7_RX)
            MODIFY_REG(GPIOA->AFR[1], 0b1111 << 0, 0b1011 << 0);
            // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA8
            MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 16, 0b11 << 16);
            // GPIOA PUPDR: Set pull-up = 0b01 to pin PA8
            MODIFY_REG(GPIOA->PUPDR, 0b11 << 16, 0b01 << 16);
            // GPIOA OTYPED: Set no open drain = 0b0 to pin PA8
            MODIFY_REG(GPIOA->OTYPER, 0b1 << 8, 0b0 << 8);
            // GPIOA MODER: Set alternate mode = 0b10 to pin PA8
            MODIFY_REG(GPIOA->MODER, 0b11 << 16, 0b10 << 16);
            break;

          case H7UART_PIN_UART7_RX_PB3:
            __HAL_RCC_GPIOB_CLK_ENABLE();

            // GPIOB AFRL: Set alternate function UART7 = 11 = 0b1011 (see datasheet chapt 5) to pin PB3 (UART7_RX)
            MODIFY_REG(GPIOB->AFR[0], 0b1111 << 12, 0b1011 << 12);
            // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB3
            MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 6, 0b11 << 6);
            // GPIOB PUPDR: Set pull-up = 0b01 to pin PB3
            MODIFY_REG(GPIOB->PUPDR, 0b11 << 6, 0b01 << 6);
            // GPIOB OTYPED: Set no open drain = 0b0 to pin PB3
            MODIFY_REG(GPIOB->OTYPER, 0b1 << 3, 0b0 << 3);
            // GPIOB MODER: Set alternate mode = 0b10 to pin PB3
            MODIFY_REG(GPIOB->MODER, 0b11 << 6, 0b10 << 6);
            break;

          case H7UART_PIN_UART7_RX_PE7:
            __HAL_RCC_GPIOE_CLK_ENABLE();

            // GPIOE AFRL: Set alternate function UART7 = 7 = 0b0111 (see datasheet chapt 5) to pin PE7 (UART7_RX)
            MODIFY_REG(GPIOE->AFR[0], 0b1111 << 28, 0b0111 << 28);
            // GPIOE OSPEEDR: Set very high speed = 0b11 to pin PE7
            MODIFY_REG(GPIOE->OSPEEDR, 0b11 << 14, 0b11 << 14);
            // GPIOE PUPDR: Set pull-up = 0b01 to pin PE7
            MODIFY_REG(GPIOE->PUPDR, 0b11 << 14, 0b01 << 14);
            // GPIOE OTYPED: Set no open drain = 0b0 to pin PE7
            MODIFY_REG(GPIOE->OTYPER, 0b1 << 7, 0b0 << 7);
            // GPIOE MODER: Set alternate mode = 0b10 to pin PE7
            MODIFY_REG(GPIOE->MODER, 0b11 << 14, 0b10 << 14);
            break;

          case H7UART_PIN_UART7_RX_PF6:
            __HAL_RCC_GPIOF_CLK_ENABLE();

            // GPIOF AFRL: Set alternate function UART7 = 7 = 0b0111 (see datasheet chapt 5) to pin PF6 (UART7_RX)
            MODIFY_REG(GPIOF->AFR[0], 0b1111 << 24, 0b0111 << 24);
            // GPIOF OSPEEDR: Set very high speed = 0b11 to pin PF6
            MODIFY_REG(GPIOF->OSPEEDR, 0b11 << 12, 0b11 << 12);
            // GPIOF PUPDR: Set pull-up = 0b01 to pin PF6
            MODIFY_REG(GPIOF->PUPDR, 0b11 << 12, 0b01 << 12);
            // GPIOF OTYPED: Set no open drain = 0b0 to pin PF6
            MODIFY_REG(GPIOF->OTYPER, 0b1 << 6, 0b0 << 6);
            // GPIOF MODER: Set alternate mode = 0b10 to pin PF6
            MODIFY_REG(GPIOF->MODER, 0b11 << 12, 0b10 << 12);
            break;

          default:
            Error_Handler();
        }
      }

      if ((init_config->function == PERIPH_TX_ONLY) || (init_config->function == PERIPH_TX_RX))
      {
        switch(init_config->pin_tx)
        {
          case H7UART_PIN_UART7_TX_PA15:
            __HAL_RCC_GPIOA_CLK_ENABLE();

            // GPIOA AFRL: Set alternate function UART7 = 11 = 0b1011 (see datasheet chapt 5) to pin PA15 (UART7_TX)
            MODIFY_REG(GPIOA->AFR[1], 0b1111 << 28, 0b1011 << 28);
            // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA15
            MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 30, 0b11 << 30);
            // GPIOA PUPDR: Set pull-up = 0b01 to pin PA15
            MODIFY_REG(GPIOA->PUPDR, 0b11 << 30, 0b01 << 30);
            // GPIOA OTYPED: Set no open drain = 0b0 to pin PA15
            MODIFY_REG(GPIOA->OTYPER, 0b1 << 15, 0b0 << 15);
            // GPIOA MODER: Set alternate mode = 0b10 to pin PA15
            MODIFY_REG(GPIOA->MODER, 0b11 << 30, 0b10 << 30);
            break;

          case H7UART_PIN_UART7_TX_PB4:
            __HAL_RCC_GPIOB_CLK_ENABLE();

            // GPIOB AFRL: Set alternate function UART7 = 11 = 0b1011 (see datasheet chapt 5) to pin PB4 (UART7_TX)
            MODIFY_REG(GPIOB->AFR[0], 0b1111 << 16, 0b1011 << 16);
            // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB3
            MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 8, 0b11 << 8);
            // GPIOB PUPDR: Set pull-up = 0b01 to pin PB3
            MODIFY_REG(GPIOB->PUPDR, 0b11 << 8, 0b01 << 8);
            // GPIOB OTYPED: Set no open drain = 0b0 to pin PB3
            MODIFY_REG(GPIOB->OTYPER, 0b1 << 4, 0b0 << 4);
            // GPIOB MODER: Set alternate mode = 0b10 to pin PB3
            MODIFY_REG(GPIOB->MODER, 0b11 << 8, 0b10 << 8);
            break;

          case H7UART_PIN_UART7_TX_PE8:
            __HAL_RCC_GPIOE_CLK_ENABLE();

            // GPIOE AFRL: Set alternate function UART7 = 7 = 0b0111 (see datasheet chapt 5) to pin PE8 (UART7_TX)
            MODIFY_REG(GPIOE->AFR[1], 0b1111 << 0, 0b0111 << 0);
            // GPIOE OSPEEDR: Set very high speed = 0b11 to pin PE8
            MODIFY_REG(GPIOE->OSPEEDR, 0b11 << 16, 0b11 << 16);
            // GPIOE PUPDR: Set pull-up = 0b01 to pin PE8
            MODIFY_REG(GPIOE->PUPDR, 0b11 << 16, 0b01 << 16);
            // GPIOE OTYPED: Set no open drain = 0b0 to pin PE8
            MODIFY_REG(GPIOE->OTYPER, 0b1 << 8, 0b0 << 8);
            // GPIOE MODER: Set alternate mode = 0b10 to pin PE8
            MODIFY_REG(GPIOE->MODER, 0b11 << 16, 0b10 << 16);
            break;

          case H7UART_PIN_UART7_TX_PF7:
            __HAL_RCC_GPIOF_CLK_ENABLE();

            // GPIOF AFRL: Set alternate function UART7 = 7 = 0b0111 (see datasheet chapt 5) to pin PF7 (UART7_TX)
            MODIFY_REG(GPIOF->AFR[0], 0b1111 << 28, 0b0111 << 28);
            // GPIOF OSPEEDR: Set very high speed = 0b11 to pin PF7
            MODIFY_REG(GPIOF->OSPEEDR, 0b11 << 14, 0b11 << 14);
            // GPIOF PUPDR: Set pull-up = 0b01 to pin PF7
            MODIFY_REG(GPIOF->PUPDR, 0b11 << 14, 0b01 << 14);
            // GPIOF OTYPED: Set no open drain = 0b0 to pin PF7
            MODIFY_REG(GPIOF->OTYPER, 0b1 << 7, 0b0 << 7);
            // GPIOF MODER: Set alternate mode = 0b10 to pin PF7
            MODIFY_REG(GPIOF->MODER, 0b11 << 14, 0b10 << 14);
            break;

          default:
            Error_Handler();
        }
      }

      //
      // *** INTERRUPT CALLBACK CONFIG.
      //

      //- Global peripheral callback
      h7uart_state_uart7.irq_callback = init_config->irq_callback;

      //- Reception callback
      h7uart_state_uart7.rx_callback  = init_config->rx_callback;

      //
      // *** UART SETUP ***
      //

      __HAL_RCC_C1_UART7_CLK_ENABLE();

      h7uart_uart_reset_peripheral_full(H7UART_UART7);

      // Set the Config Register 1
      MODIFY_REG(UART7->CR1,
            USART_CR1_M1             | USART_CR1_EOBIE  | USART_CR1_RTOIE
          | USART_CR1_DEAT           | USART_CR1_DEDT   | USART_CR1_OVER8  | USART_CR1_CMIE | USART_CR1_MME           | USART_CR1_M0
          | USART_CR1_WAKE           | USART_CR1_PCE    | USART_CR1_PS     | USART_CR1_PEIE | USART_CR1_TXEIE_TXFNFIE | USART_CR1_TCIE
          | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_IDLEIE | USART_CR1_TE     | USART_CR1_RE   | USART_CR1_UESM          | USART_CR1_UE,
          cr1_value);

      // Set the Config Register 2
      MODIFY_REG(UART7->CR2,
            USART_CR2_ADD     | USART_CR2_RTOEN | USART_CR2_ABRMODE | USART_CR2_ABREN | USART_CR2_MSBFIRST | USART_CR2_DATAINV
          | USART_CR2_TXINV   | USART_CR2_RXINV | USART_CR2_SWAP   | USART_CR2_LINEN | USART_CR2_STOP     | USART_CR2_CLKEN
          | USART_CR2_CPOL    | USART_CR2_CPHA  | USART_CR2_LBCL   | USART_CR2_LBDIE | USART_CR2_LBDL     | USART_CR2_ADDM7
          | USART_CR2_DIS_NSS | USART_CR2_SLVEN,
          cr2_value);

      // Set the Config Register 3
      MODIFY_REG(UART7->CR3,
          USART_CR3_TCBGTIE | USART_CR3_WUFIE
        | USART_CR3_WUS     | USART_CR3_SCARCNT | USART_CR3_DEP     | USART_CR3_DEM     | USART_CR3_DDRE   | USART_CR3_OVRDIS
        | USART_CR3_ONEBIT  | USART_CR3_CTSIE   | USART_CR3_CTSE    | USART_CR3_RTSE    | USART_CR3_DMAT   | USART_CR3_DMAR
        | USART_CR3_SCEN    | USART_CR3_NACK    | USART_CR3_HDSEL   | USART_CR3_IRLP    | USART_CR3_IREN   | USART_CR3_EIE,
        cr3_value);

      // Set the Guard Time and Prescaler Register
      // ( It is related to Smartcard function which is not implemented by the present driver )
      MODIFY_REG(UART7->GTPR,
          USART_GTPR_GT | USART_GTPR_PSC,
          0UL);

      // Set Receiver Timeout Register
      // - BLEN: It is used only in Smartcard mode which is not implmented by the present driver
      // - RTO: Receive timeout value it is not used in the present driver.
      MODIFY_REG(USART1->RTOR,USART_RTOR_BLEN | USART_RTOR_RTO,0UL);

      // Set Baud Rate Configuration

      // - Kernel Clock after prescaler
      usart_ker_ckpres = USART_CLOCK_KER_USART_2345678 / presc_value;

      // - Compute baud rate register value
      brr_value = usart_ker_ckpres / init_config->baud_rate;

      // Checks if the resulting brr values fits in 16-bits
      if ( brr_value != ( brr_value & 0x0000FFFF ) )
        Error_Handler();

      // Set Prescaler Register
      MODIFY_REG(UART7->PRESC,USART_PRESC_PRESCALER,presc_value);

      // Set Baud Rate Register
      MODIFY_REG(UART7->BRR,USART_BRR_DIV_FRACTION | USART_BRR_DIV_MANTISSA,brr_value);

      //
      //  *** NVIC SETUP ***
      //
      HAL_NVIC_SetPriority(UART7_IRQn, H7UART_IRQ_UART7_PRI, H7UART_IRQ_UART7_SUBPRI);
      HAL_NVIC_EnableIRQ(UART7_IRQn);

      // Peripheral enable
      SET_BIT(UART7->CR1,USART_CR1_UE);
      READ_REG(UART7->CR1);

      // Update driver state
      h7uart_state_uart7.fsm_state = H7UART_FSM_STATE_IDLE;

      break;
#endif

#if H7UART_PERIPH_ENABLE_UART8   == 1
    case H7UART_UART8:
      //
      // *** GPIO PIN SETUP ***
      //

      if ((init_config->function == PERIPH_RX_ONLY) || (init_config->function == PERIPH_TX_RX))
      {
        switch(init_config->pin_rx)
        {
          case H7UART_PIN_UART8_RX_PE0:
            __HAL_RCC_GPIOE_CLK_ENABLE();

            // GPIOE AFRL: Set alternate function UART8 = 7 = 0b1000 (see datasheet chapt 5) to pin PE0 (UART8_RX)
            MODIFY_REG(GPIOE->AFR[0], 0b1111 << 0, 0b1000 << 0);
            // GPIOE OSPEEDR: Set very high speed = 0b11 to pin PE0
            MODIFY_REG(GPIOE->OSPEEDR, 0b11 << 0, 0b11 << 0);
            // GPIOE PUPDR: Set pull-up = 0b01 to pin PE0
            MODIFY_REG(GPIOE->PUPDR, 0b11 << 0, 0b01 << 0);
            // GPIOE OTYPED: Set no open drain = 0b0 to pin PE0
            MODIFY_REG(GPIOE->OTYPER, 0b1 << 0, 0b0 << 0);
            // GPIOE MODER: Set alternate mode = 0b10 to pin PE0
            MODIFY_REG(GPIOE->MODER, 0b11 << 0, 0b10 << 0);
            break;

          case H7UART_PIN_UART8_RX_PJ9:
            __HAL_RCC_GPIOJ_CLK_ENABLE();

            // GPIOJ AFRL: Set alternate function UART8 = 7 = 0b1000 (see datasheet chapt 5) to pin PJ9 (UART8_RX)
            MODIFY_REG(GPIOJ->AFR[1], 0b1111 << 4, 0b1000 << 4);
            // GPIOJ OSPEEDR: Set very high speed = 0b11 to pin PJ9
            MODIFY_REG(GPIOJ->OSPEEDR, 0b11 << 18, 0b11 << 18);
            // GPIOJ PUPDR: Set pull-up = 0b01 to pin PJ9
            MODIFY_REG(GPIOJ->PUPDR, 0b11 << 18, 0b01 << 18);
            // GPIOJ OTYPED: Set no open drain = 0b0 to pin PJ9
            MODIFY_REG(GPIOJ->OTYPER, 0b1 << 9, 0b0 << 9);
            // GPIOJ MODER: Set alternate mode = 0b10 to pin PJ9
            MODIFY_REG(GPIOJ->MODER, 0b11 << 18, 0b10 << 18);
            break;

          default:
            Error_Handler();
        }
      }

      if ((init_config->function == PERIPH_TX_ONLY) || (init_config->function == PERIPH_TX_RX))
      {
        switch(init_config->pin_tx)
        {
          case H7UART_PIN_UART8_TX_PE1:
            __HAL_RCC_GPIOE_CLK_ENABLE();

            // GPIOE AFRL: Set alternate function UART8 = 7 = 0b1000 (see datasheet chapt 5) to pin PE1 (UART8_TX)
            MODIFY_REG(GPIOE->AFR[0], 0b1111 << 4, 0b1000 << 4);
            // GPIOE OSPEEDR: Set very high speed = 0b11 to pin PE1
            MODIFY_REG(GPIOE->OSPEEDR, 0b11 << 2, 0b11 << 2);
            // GPIOE PUPDR: Set pull-up = 0b01 to pin PE1
            MODIFY_REG(GPIOE->PUPDR, 0b11 << 2, 0b01 << 2);
            // GPIOE OTYPED: Set no open drain = 0b0 to pin PE1
            MODIFY_REG(GPIOE->OTYPER, 0b1 << 1, 0b0 << 1);
            // GPIOE MODER: Set alternate mode = 0b10 to pin PE1
            MODIFY_REG(GPIOE->MODER, 0b11 << 2, 0b10 << 2);
            break;

          case H7UART_PIN_UART8_TX_PJ8:
            __HAL_RCC_GPIOJ_CLK_ENABLE();

            // GPIOJ AFRL: Set alternate function UART8 = 7 = 0b1000 (see datasheet chapt 5) to pin PJ8 (UART8_TX)
            MODIFY_REG(GPIOJ->AFR[1], 0b1111 << 0, 0b1000 << 0);
            // GPIOJ OSPEEDR: Set very high speed = 0b11 to pin PJ8
            MODIFY_REG(GPIOJ->OSPEEDR, 0b11 << 16, 0b11 << 16);
            // GPIOJ PUPDR: Set pull-up = 0b01 to pin PJ8
            MODIFY_REG(GPIOJ->PUPDR, 0b11 << 16, 0b01 << 16);
            // GPIOJ OTYPED: Set no open drain = 0b0 to pin PJ8
            MODIFY_REG(GPIOJ->OTYPER, 0b1 << 8, 0b0 << 8);
            // GPIOJ MODER: Set alternate mode = 0b10 to pin PJ8
            MODIFY_REG(GPIOJ->MODER, 0b11 << 16, 0b10 << 16);
            break;

          default:
            Error_Handler();
        }
      }

      //
      // *** INTERRUPT CALLBACK CONFIG.
      //

      //- Global peripheral callback
      h7uart_state_uart8.irq_callback = init_config->irq_callback;

      //- Reception callback
      h7uart_state_uart8.rx_callback  = init_config->rx_callback;

      //
      // *** UART SETUP ***
      //
      __HAL_RCC_C1_UART8_CLK_ENABLE();

      h7uart_uart_reset_peripheral_full(H7UART_UART8);

      // Set the Config Register 1
      MODIFY_REG(UART8->CR1,
            USART_CR1_M1             | USART_CR1_EOBIE  | USART_CR1_RTOIE
          | USART_CR1_DEAT           | USART_CR1_DEDT   | USART_CR1_OVER8  | USART_CR1_CMIE | USART_CR1_MME           | USART_CR1_M0
          | USART_CR1_WAKE           | USART_CR1_PCE    | USART_CR1_PS     | USART_CR1_PEIE | USART_CR1_TXEIE_TXFNFIE | USART_CR1_TCIE
          | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_IDLEIE | USART_CR1_TE     | USART_CR1_RE   | USART_CR1_UESM          | USART_CR1_UE,
          cr1_value);

      // Set the Config Register 2
      MODIFY_REG(UART8->CR2,
            USART_CR2_ADD     | USART_CR2_RTOEN | USART_CR2_ABRMODE | USART_CR2_ABREN | USART_CR2_MSBFIRST | USART_CR2_DATAINV
          | USART_CR2_TXINV   | USART_CR2_RXINV | USART_CR2_SWAP    | USART_CR2_LINEN | USART_CR2_STOP     | USART_CR2_CLKEN
          | USART_CR2_CPOL    | USART_CR2_CPHA  | USART_CR2_LBCL    | USART_CR2_LBDIE | USART_CR2_LBDL     | USART_CR2_ADDM7
          | USART_CR2_DIS_NSS | USART_CR2_SLVEN,
          cr2_value);

      // Set the Config Register 3
      MODIFY_REG(UART8->CR3,
          USART_CR3_TCBGTIE | USART_CR3_WUFIE
        | USART_CR3_WUS     | USART_CR3_SCARCNT | USART_CR3_DEP     | USART_CR3_DEM     | USART_CR3_DDRE   | USART_CR3_OVRDIS
        | USART_CR3_ONEBIT  | USART_CR3_CTSIE   | USART_CR3_CTSE    | USART_CR3_RTSE    | USART_CR3_DMAT   | USART_CR3_DMAR
        | USART_CR3_SCEN    | USART_CR3_NACK    | USART_CR3_HDSEL   | USART_CR3_IRLP    | USART_CR3_IREN   | USART_CR3_EIE,
        cr3_value);

      // Set the Guard Time and Prescaler Register
      // ( It is related to Smartcard function which is not implemented by the present driver )
      MODIFY_REG(UART8->GTPR,USART_GTPR_GT | USART_GTPR_PSC,0UL);

      // Set Receiver Timeout Register
      // - BLEN: It is used only in Smartcard mode which is not implmented by the present driver
      // - RTO: Receive timeout value it is not used in the present driver.
      MODIFY_REG(UART8->RTOR,USART_RTOR_BLEN | USART_RTOR_RTO,0UL);

      // Set Baud Rate Configuration

      // - Kernel Clock after prescaler
      usart_ker_ckpres = USART_CLOCK_KER_USART_2345678 / presc_value;

      // - Compute baud rate register value
      brr_value = usart_ker_ckpres / init_config->baud_rate;

      // Checks if the resulting brr values fits in 16-bits
      if ( brr_value != ( brr_value & 0x0000FFFF ) )
        Error_Handler();

      // Set Prescaler Register
      MODIFY_REG(UART8->PRESC,USART_PRESC_PRESCALER,presc_value);

      // Set Baud Rate Register
      MODIFY_REG(UART8->BRR,USART_BRR_DIV_FRACTION | USART_BRR_DIV_MANTISSA,brr_value);

      //
      //  *** NVIC SETUP ***
      //
      HAL_NVIC_SetPriority(UART8_IRQn, H7UART_IRQ_UART8_PRI, H7UART_IRQ_UART8_SUBPRI);
      HAL_NVIC_EnableIRQ(UART8_IRQn);

      // Peripheral enable
      SET_BIT(UART8->CR1,USART_CR1_UE);
      READ_REG(UART8->CR1);

      // Update driver state
      h7uart_state_uart8.fsm_state = H7UART_FSM_STATE_IDLE;

      break;
#endif

#if H7UART_PERIPH_ENABLE_LPUART1 == 1
    case H7UART_LPUART1:
      //
      // *** GPIO PIN SETUP ***
      //
      if ((init_config->function == PERIPH_RX_ONLY) || (init_config->function == PERIPH_TX_RX))
      {
        switch(init_config->pin_rx)
        {
          case H7UART_PIN_LPUART1_RX_PA10:
            __HAL_RCC_GPIOA_CLK_ENABLE();

            // GPIOA AFRL: Set alternate function LPUART1 = 7 = 0b0111 (see datasheet chapt 5) to pin PA10 (LPUART1_RX)
            MODIFY_REG(GPIOA->AFR[1], 0b1111 << 8, 0b0111 << 8);
            // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA10
            MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 20, 0b11 << 20);
            // GPIOA PUPDR: Set pull-up = 0b01 to pin PA10
            MODIFY_REG(GPIOA->PUPDR, 0b11 << 20, 0b01 << 20);
            // GPIOA OTYPED: Set no open drain = 0b0 to pin PA10
            MODIFY_REG(GPIOA->OTYPER, 0b1 << 10, 0b0 << 10);
            // GPIOA MODER: Set alternate mode = 0b10 to pin PA10
            MODIFY_REG(GPIOA->MODER, 0b11 << 20, 0b10 << 20);
            break;

          case H7UART_PIN_LPUART1_RX_PB7:
            __HAL_RCC_GPIOB_CLK_ENABLE();

            // GPIOB AFRL: Set alternate function LPUART1 = 7 = 0b1000 (see datasheet chapt 5) to pin PB7 (LPUART1_RX)
            MODIFY_REG(GPIOB->AFR[0], 0b1111 << 28, 0b1000 << 28);
            // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB7
            MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 14, 0b11 << 14);
            // GPIOB PUPDR: Set pull-up = 0b01 to pin PB7
            MODIFY_REG(GPIOB->PUPDR, 0b11 << 14, 0b01 << 14);
            // GPIOB OTYPED: Set no open drain = 0b0 to pin PB7
            MODIFY_REG(GPIOB->OTYPER, 0b1 << 7, 0b0 << 7);
            // GPIOB MODER: Set alternate mode = 0b10 to pin PB7
            MODIFY_REG(GPIOB->MODER, 0b11 << 14, 0b10 << 14);
            break;

          default:
            Error_Handler();
        }
      }

      if ((init_config->function == PERIPH_TX_ONLY) || (init_config->function == PERIPH_TX_RX))
      {
        switch(init_config->pin_tx)
        {
          case H7UART_PIN_LPUART1_TX_PA9:
            __HAL_RCC_GPIOA_CLK_ENABLE();

            // GPIOA AFRL: Set alternate function LPUART1 = 7 = 0b0111 (see datasheet chapt 5) to pin PA9 (LPUART1_TX)
            MODIFY_REG(GPIOA->AFR[1], 0b1111 << 4, 0b0111 << 4);
            // GPIOA OSPEEDR: Set very high speed = 0b11 to pin PA9
            MODIFY_REG(GPIOA->OSPEEDR, 0b11 << 20, 0b11 << 20);
            // GPIOA PUPDR: Set pull-up = 0b01 to pin PA9
            MODIFY_REG(GPIOA->PUPDR, 0b11 << 20, 0b01 << 20);
            // GPIOA OTYPED: Set no open drain = 0b0 to pin PA9
            MODIFY_REG(GPIOA->OTYPER, 0b1 << 9, 0b0 << 9);
            // GPIOA MODER: Set alternate mode = 0b10 to pin PA9
            MODIFY_REG(GPIOA->MODER, 0b11 << 18, 0b10 << 18);
            break;

          case H7UART_PIN_LPUART1_TX_PB6:
            __HAL_RCC_GPIOB_CLK_ENABLE();

            // GPIOB AFRL: Set alternate function LPUART1 = 7 = 0b1000 (see datasheet chapt 5) to pin PB6 (LPUART1_RX)
            MODIFY_REG(GPIOB->AFR[0], 0b1111 << 24, 0b1000 << 24);
            // GPIOB OSPEEDR: Set very high speed = 0b11 to pin PB6
            MODIFY_REG(GPIOB->OSPEEDR, 0b11 << 12, 0b11 << 12);
            // GPIOB PUPDR: Set pull-up = 0b01 to pin PB6
            MODIFY_REG(GPIOB->PUPDR, 0b11 << 12, 0b01 << 12);
            // GPIOB OTYPED: Set no open drain = 0b0 to pin PB6
            MODIFY_REG(GPIOB->OTYPER, 0b1 << 6, 0b0 << 6);
            // GPIOB MODER: Set alternate mode = 0b10 to pin PB6
            MODIFY_REG(GPIOB->MODER, 0b11 << 12, 0b10 << 12);
            break;

          default:
            Error_Handler();
        }
      }

      //
      // *** INTERRUPT CALLBACK CONFIG.
      //

      //- Global peripheral callback
      h7uart_state_lpuart1.irq_callback = init_config->irq_callback;

      //- Reception callback
      h7uart_state_lpuart1.rx_callback  = init_config->rx_callback;

      //
      // *** UART SETUP ***
      //
      __HAL_RCC_C1_LPUART1_CLK_ENABLE();

      h7uart_uart_reset_peripheral_full(H7UART_LPUART1);

      // Set the Config Register 1
      MODIFY_REG(LPUART1->CR1,
            USART_CR1_M1             | USART_CR1_EOBIE  | USART_CR1_RTOIE
          | USART_CR1_DEAT           | USART_CR1_DEDT   | USART_CR1_OVER8  | USART_CR1_CMIE | USART_CR1_MME           | USART_CR1_M0
          | USART_CR1_WAKE           | USART_CR1_PCE    | USART_CR1_PS     | USART_CR1_PEIE | USART_CR1_TXEIE_TXFNFIE | USART_CR1_TCIE
          | USART_CR1_RXNEIE_RXFNEIE | USART_CR1_IDLEIE | USART_CR1_TE     | USART_CR1_RE   | USART_CR1_UESM          | USART_CR1_UE,
          cr1_value);

      // Set the Config Register 2
      MODIFY_REG(LPUART1->CR2,
            USART_CR2_ADD     | USART_CR2_RTOEN | USART_CR2_ABRMODE | USART_CR2_ABREN | USART_CR2_MSBFIRST | USART_CR2_DATAINV
          | USART_CR2_TXINV   | USART_CR2_RXINV | USART_CR2_SWAP    | USART_CR2_LINEN | USART_CR2_STOP     | USART_CR2_CLKEN
          | USART_CR2_CPOL    | USART_CR2_CPHA  | USART_CR2_LBCL    | USART_CR2_LBDIE | USART_CR2_LBDL     | USART_CR2_ADDM7
          | USART_CR2_DIS_NSS | USART_CR2_SLVEN,
          cr2_value);

      // Set the Config Register 3
      MODIFY_REG(LPUART1->CR3,
          USART_CR3_TCBGTIE | USART_CR3_WUFIE
        | USART_CR3_WUS     | USART_CR3_SCARCNT | USART_CR3_DEP     | USART_CR3_DEM     | USART_CR3_DDRE   | USART_CR3_OVRDIS
        | USART_CR3_ONEBIT  | USART_CR3_CTSIE   | USART_CR3_CTSE    | USART_CR3_RTSE    | USART_CR3_DMAT   | USART_CR3_DMAR
        | USART_CR3_SCEN    | USART_CR3_NACK    | USART_CR3_HDSEL   | USART_CR3_IRLP    | USART_CR3_IREN   | USART_CR3_EIE,
        cr3_value);

      // Set the Guard Time and Prescaler Register
      // ( It is related to Smartcard function which is not implemented by the present driver )
      MODIFY_REG(LPUART1->GTPR,USART_GTPR_GT | USART_GTPR_PSC,0UL);

      // Set Receiver Timeout Register
      // - BLEN: It is used only in Smartcard mode which is not implmented by the present driver
      // - RTO: Receive timeout value it is not used in the present driver.
      MODIFY_REG(LPUART1->RTOR,USART_RTOR_BLEN | USART_RTOR_RTO,0UL);

      // Set Baud Rate Configuration

      // - Kernel Clock after prescaler
      usart_ker_ckpres = USART_CLOCK_KER_USART_2345678 / presc_value;

      // - Compute baud rate register value
      brr_value = (256 *  usart_ker_ckpres) / init_config->baud_rate;

      // Checks if the resulting brr values fits in 16-bits
      if ( brr_value != ( brr_value & 0x0000FFFF ) )
        Error_Handler();

      // Set Prescaler Register
      MODIFY_REG(USART1->PRESC,USART_PRESC_PRESCALER,presc_value);

      // Set Baud Rate Register
      MODIFY_REG(USART1->BRR,USART_BRR_DIV_FRACTION | USART_BRR_DIV_MANTISSA,brr_value);

      //
      //  *** NVIC SETUP ***
      //
      HAL_NVIC_SetPriority(LPUART1_IRQn, H7UART_IRQ_LPUART1_PRI, H7UART_IRQ_LPUART1_SUBPRI);
      HAL_NVIC_EnableIRQ(LPUART1_IRQn);

      // Peripheral enable
      SET_BIT(LPUART1->CR1,USART_CR1_UE);
      READ_REG(LPUART1->CR1);

      // Update driver state
      h7uart_state_lpuart1.fsm_state = H7UART_FSM_STATE_IDLE;

      break;
#endif
      default:
        return H7UART_RET_CODE_UNMANAGED_BY_DRIVER;
  };

  return H7UART_RET_CODE_OK;
}

h7uart_uart_ret_code_t h7uart_clear_error_state(h7uart_periph_t peripheral)
{
  uint32_t const timeout = 100;

  h7uart_driver_instance_state_t* instance = h7uart_get_driver_instance(peripheral);

  if (!instance)
    return H7UART_RET_CODE_UNMANAGED_BY_DRIVER;

  if (h7uart_uart_mutex_lock(peripheral, timeout) == H7UART_RET_CODE_OK)
  {
    switch(instance->fsm_state)
    {
      case H7UART_FSM_STATE_ERROR_ABRE:   // Auto baud-rate error
      case H7UART_FSM_STATE_ERROR_UDR:    // SPI slave underrun error flag
      case H7UART_FSM_STATE_ERROR_ORE:    // Overrun error
      case H7UART_FSM_STATE_ERROR_NE:     // Noise detection flag
      case H7UART_FSM_STATE_ERROR_FE:     // Framing error
      case H7UART_FSM_STATE_ERROR_PE:     // Parity error
        instance->fsm_state = H7UART_FSM_STATE_IDLE;
        h7uart_uart_mutex_release(peripheral);
        h7uart_uart_reset_peripheral_soft(peripheral);
        return H7UART_RET_CODE_OK;
      default:
        h7uart_uart_mutex_release(peripheral);
        h7uart_uart_reset_peripheral_soft(peripheral);
        return H7UART_RET_CODE_CLEARED_A_NONERROR_STATE;
    }
  }
  return H7UART_RET_CODE_BUSY;
}

void H7UART_IRQHandler_Impl(h7uart_periph_t peripheral)
{
  h7uart_driver_instance_state_t* instance = h7uart_get_driver_instance(peripheral);
  USART_TypeDef* hardware = (USART_TypeDef*) instance->uart_base;

  // Get Interrupt and Status Register.
  uint32_t const isr = hardware->ISR;

  // Auxiliary Interrupt Flag Clear Register
  uint32_t icr = 0UL;

  // Auxiliary Request Register
  uint32_t rqr = 0UL;

  // TXFT: TXFIFO threshold flag
  // This bit is set by hardware when the TXFIFO reaches the threshold programmed.
  if ( READ_BIT(isr,USART_ISR_TXFT) != 0 )
  {
    // TODO: Not used in the present driver.
  }

  // RXFT: RXFIFO threshold flag
  // This bit is set by hardware when the threshold programmed.
  if ( READ_BIT(isr,USART_ISR_RXFT) != 0 )
  {
    instance->cont_rx = 0;
    do
    {
      instance->data_rx[instance->cont_rx] = (uint8_t) hardware->RDR;
      instance->cont_rx++;

    } while(READ_BIT(hardware->ISR,USART_ISR_RXFT) != 0);

    instance->rx_callback(instance->data_rx,instance->cont_rx);
  }

  // TCBGT: Transmission complete before guard time flag.
  // This bit is set when the last data written in the USART_TDR has been transmitted correctly out of the shift register.
  if ( READ_BIT(isr,USART_ISR_TCBGT) != 0 )
  {
    // Clear interrupt flag
    icr |= USART_ICR_TCBGTCF;
  }

  // RXFF: RXFIFO full
  // This bit is set by hardware when the number of received data corresponds to RXFIFO size + 1 (RXFIFO full + 1 data in the USART_RDR reg.)
  if ( READ_BIT(isr, USART_ISR_RXFF))
  {
    instance->cont_rx = 0;
    do
    {
      instance->data_rx[instance->cont_rx] = (uint8_t) hardware->RDR;
      instance->cont_rx++;

    } while(READ_BIT(hardware->ISR,USART_ISR_RXFF) != 0);

    instance->rx_callback(instance->data_rx,instance->cont_rx);
  }

  // TXFE: TXFIFO empty
  // This bit is set by hardware when TXFIFO is empty. When the TXFIFO contains at least one data, this flag is cleared.
  if ( READ_BIT(isr, USART_ISR_TXFE))
  {
    // Clear interrupt flag
    icr |= USART_ICR_TXFECF;
  }

  // REACK: Receive enable acknowledge flag
  if ( READ_BIT(isr,USART_ISR_REACK) != 0 )
  {
    // TODO: It is not an interrupt
  }

  // TEACK: Transmit enable acknowledge flag
  if ( READ_BIT(isr,USART_ISR_TEACK) != 0 )
  {
    // TODO: Not used in this driver
  }

  // WUF: Wakeup from low-power mode flag
  // This bit is set by hardware, when a wakeup event is detected.
  if ( READ_BIT(isr,USART_ISR_WUF) != 0 )
  {
    // Clear interrupt flag
    icr |= USART_ICR_WUCF;
  }

  // RWU: Receiver wakeup from Mute mode
  if ( READ_BIT(isr,USART_ISR_RWU) != 0 )
  {
    // TODO: Not used in this driver
  }

  // SBKF: Send break flag
  // This bit indicated that a send break character was requested. It is set by software.
  if ( READ_BIT(isr,USART_ISR_SBKF) != 0 )
  {
    // TODO: Not used in this driver
  }

  // CMF: Character match flag
  // This bit is set by hardware, when a the character defined by ADD[7:0] is received.
  if ( READ_BIT(isr,USART_ISR_SBKF) != 0 )
  {
    // Clear interrupt flag
    icr |= USART_ICR_CMCF;
  }

  // BUSY: Busy flag
  // This bit is set and reset by hardware. It is active when a communication is ongoing on the RX line (successful start bit detected).
  // It is reset at the end of the reception (successful or not).
  if ( READ_BIT(isr,USART_ISR_BUSY) != 0 )
  {
    // TODO: Not used in this driver
  }

  // ABRF: Auto baud rate flag
  // This bit is set by hardware when the automatic baud rate has been set or when the auto baud rate operations was completed without success.
  if ( READ_BIT(isr,USART_ISR_ABRF) != 0 )
  {
    // TODO: Not used in this driver
  }

  // ABRE: Auto baud rate error
  // This bit is set by hardware if the baud rate measurement failed.
  if ( READ_BIT(isr,USART_ISR_ABRE) != 0 )
  {
    instance->fsm_state = H7UART_FSM_STATE_ERROR_ABRE;
  }

  // UDR: SPI slave underrun error flag.
  if ( READ_BIT(isr,USART_ISR_UDR) != 0 )
  {
    instance->fsm_state = H7UART_FSM_STATE_ERROR_UDR;
    // Clear interrupt flag
    icr |= USART_ICR_UDRCF;
  }

  // EOBF: End of block flag
  // (if Smartcard mode is not supported, this bit is reserverd and kept at reset value.)
  if ( READ_BIT(isr,USART_ISR_UDR) != 0 )
  {
    // Clear interrupt flag
    icr |= USART_ICR_EOBCF;
  }

  // RTOF: Receiver timeout
  // This bit is set by hardware when the timeout value, programmed in the RTOR register has lapsed, without any communication.
  if ( READ_BIT(isr,USART_ISR_RTOF) != 0 )
  {
    // Clear interrupt flag
    icr |= USART_ICR_RTOCF;
  }

  // CTS: CTS flag
  // This bit is set/reset by hardware. It is an inverted copy of the status of the nCTS input pin.
  if ( READ_BIT(isr,USART_ISR_CTS) != 0 )
  {
    // TODO: Not used in this driver
  }

  // CTSIF: CTS interrupt flag
  // This bit is set by hardware when the nCTS input toggles, if the CTSE bit is set.
  if ( READ_BIT(isr,USART_ISR_CTSIF))
  {
    // Clear interrupt flag
    icr |= USART_ICR_CTSCF;
  }

  // LBDF: LIN break detection flag
  if ( READ_BIT(isr,USART_ISR_LBDF))
  {
    // Clear interrupt flag
    icr |= USART_ICR_LBDCF;
  }

  // FIFO MODE: TXFNF - TXFIFO no full
  // TXFNF is set by hardware when TXFIFO is not full meaning that data can be written in the USART_TDR.
  // NO FIFO MODE: TXE - Transmit data register empty
  // TXE is set by hardware when the content of the USART_TDR register has been transferred into the shift register.
  if ( READ_BIT(isr,USART_ISR_TXE_TXFNF) != 0)
  {
    if( instance->fsm_state == H7UART_FSM_STATE_TRASFERING )
    {
      do
      {
        if( instance->cont_tx < instance->len_tx )
        {
          hardware->TDR = ( 0x000000FF & instance->data_tx[instance->cont_tx] );
          instance->cont_tx++;
        }
      } while(READ_BIT(hardware->ISR,USART_ISR_TXE_TXFNF) != 0);
    }
  }

  // TC: Transmission complete
  // This bit indicates that the last data written in the USART_TDR has been transmitted out of the shift register.
  if ( READ_BIT(isr,USART_ISR_TC) != 0)
  {
    if ( instance->fsm_state == H7UART_FSM_STATE_TRASFERING )
    {
      if( instance->cont_tx >= instance->len_tx )
      {
        instance->fsm_state = H7UART_FSM_STATE_IDLE;
      }
    }
    // Clear interrupt flag
    icr |= USART_ICR_TCCF;
  }

  // FIFO MODE: RXFNE - RXFIFO not empty
  // RXFNE bit is set by hardware when the RXFIFO is not empty, meaning that data can be read from the USART_RDR register.
  // NO FIFO MODE: RXNE - Read data register not empty
  // RXNE bit is set by hardware when the content of the USART_RDR shift register has been transferred to the USART_RDR
  if ( READ_BIT(isr,USART_ISR_RXNE_RXFNE))
  {
    if ((READ_BIT(hardware->CR1,USART_CR1_FIFOEN) != 0) && (READ_BIT(hardware->CR1,USART_CR1_RXFFIE) != 0) )
    {
      instance->cont_rx = 0;
      do
      {
        instance->data_rx[instance->cont_rx] = (uint8_t) hardware->RDR;
        instance->cont_rx++;

      } while(READ_BIT(hardware->ISR,USART_ISR_RXNE_RXFNE) != 0);

      if (instance->rx_callback != NULL )
        instance->rx_callback(instance->data_rx,instance->cont_rx);
    }
  }

  // IDLE: idle line detected
  // This bit is set by hardware when an Idle Line is detected. An interrupt is generated if IDLEIE=1 in the USART_CR1 register.
  if ( READ_BIT(isr,USART_ISR_IDLE) != 0)
  {
    // Clear interrupt flag
    icr |= USART_ICR_IDLECF;
  }

  // ORE: Overrun error
  // This bit is set by hardware when the data currently being received in the shift register is ready to be transferred
  // into the USART_RDR register while RXFF = 1.
  if ( READ_BIT(isr,USART_ISR_ORE))
  {
    instance->fsm_state = H7UART_FSM_STATE_ERROR_ORE;
    // Clear interrupt flag
    icr |= USART_ICR_ORECF;
  }

  // NE: Noise detection flag
  // This bit is set by hardware when noise is detected on a received frame. It is cleared by software, wirting 1 to the
  // NECCF bit in the USART_ICR register.
  if ( READ_BIT(isr,USART_ISR_NE))
  {
    instance->fsm_state = H7UART_FSM_STATE_ERROR_NE;
    // Clear interrupt flag
    icr |= USART_ICR_NECF;
  }

  // FE: Framing error
  // This bit is set by hardware when a de-synchronization, excessive noise or a break character is detected.
  if ( READ_BIT(isr,USART_ISR_FE))
  {
    instance->fsm_state = H7UART_FSM_STATE_ERROR_FE;
    // Clear interrupt flag
    icr |= USART_ICR_FECF;
  }

  // PE: Parity error
  // This bit is set by hardware when a parity error occurs in receiver mode.
  if ( READ_BIT(isr,USART_ISR_PE))
  {
    instance->fsm_state = H7UART_FSM_STATE_ERROR_PE;
    // Clear interrupt flag
    icr |= USART_ICR_PECF;
  }

  // Execute callback
  if(instance->irq_callback != NULL)
    instance->irq_callback(isr);

  // Update interrupt flag clear register
  hardware->ICR = icr;
  READ_REG(hardware->ICR);

  // Check fsm_state to update mutex
  if (instance->fsm_state == H7UART_FSM_STATE_IDLE)
  {
    h7uart_uart_mutex_release_fromISR(peripheral);
  }
}

static int h7uart_uart_pre_transaction_check(h7uart_periph_t peripheral, uint32_t timeout)
{
  h7uart_driver_instance_state_t* instance = h7uart_get_driver_instance(peripheral);

  if(!instance)
    return H7UART_RET_CODE_UNMANAGED_BY_DRIVER;

  // Do not run if the driver is in error state
  switch(instance->fsm_state)
  {
    case H7UART_FSM_STATE_ERROR_ABRE:
    case H7UART_FSM_STATE_ERROR_UDR:
    case H7UART_FSM_STATE_ERROR_ORE:
    case H7UART_FSM_STATE_ERROR_NE:
    case H7UART_FSM_STATE_ERROR_FE:
    case H7UART_FSM_STATE_ERROR_PE:
      return H7UART_RET_CODE_PERIPH_IN_ERR_STATE;
    default:
      break;
  }

  // Lazy initialization. This branch usually should happen just once
  if (instance->fsm_state == H7UART_FSM_STATE_UNINITIALIZED)
    h7uart_uart_init(peripheral);

  instance->fsm_state = H7UART_FSM_STATE_SETUP_TRANSFER;

  return H7UART_RET_CODE_OK;
}

static int h7uart_uart_tx(h7uart_periph_t peripheral, uint8_t *data, uint16_t len, uint32_t timeout)
{
  if((len == 0UL) || (data == NULL))
    return H7UART_RET_CODE_INVALID_ARGS;

  h7uart_driver_instance_state_t* instance = h7uart_get_driver_instance(peripheral);

  if(!instance)
    return H7UART_RET_CODE_UNMANAGED_BY_DRIVER;

  if(h7uart_uart_mutex_lock(peripheral, timeout) != H7UART_RET_CODE_OK)
    return H7UART_RET_CODE_BUSY;

  int const check_ret_val = h7uart_uart_pre_transaction_check(peripheral,timeout);

  if ( check_ret_val != H7UART_RET_CODE_OK)
  {
    h7uart_uart_mutex_release(peripheral);
    return check_ret_val;
  }

  instance->timestart  = HAL_GetTick();
  instance->timeout    = timeout;

  instance->len_tx     = len;
  instance->cont_tx    = 0;
  instance->data_tx    = data;

  // Update FSM state
  instance->fsm_state = H7UART_FSM_STATE_TRASFERING;
}


