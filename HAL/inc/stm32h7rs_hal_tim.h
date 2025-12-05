#ifndef STM32H7RS_HAL_TIM_H
#define STM32H7RS_HAL_TIM_H

#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_rcc.h"
#include "stm32h7rsxx.h"
#include "stm32h7s3xx.h"
#include <stdint.h>
#include <sys/timespec.h>

typedef struct {
    uint32_t Prescaler;
    uint32_t CounterMode;
    uint32_t Period;
    uint32_t ClockDivision;
    uint32_t RepetitionCounter;
    uint32_t AutoReloadPreload;
} TIM_Init;

typedef enum {
    HAL_TIM_STATE_RESET   = 0x00U, /*!< Peripheral is not initialized */
    HAL_TIM_STATE_READY   = 0x01U, /*!< Peripheral Initialized and ready for use       */
    HAL_TIM_STATE_BUSY    = 0x02U, /*!< an internal process is ongoing    */
    HAL_TIM_STATE_TIMEOUT = 0x03U, /*!< Timeout state */
    HAL_TIM_STATE_ERROR   = 0x04U  /*!< Error    */
} HAL_TIM_State;

typedef enum {
    HAL_TIM_CHANNEL_STATE_RESET = 0x00U, /*!< TIM Channel initial state                         */
    HAL_TIM_CHANNEL_STATE_READY = 0x01U, /*!< TIM Channel ready for use                         */
    HAL_TIM_CHANNEL_STATE_BUSY  = 0x02U, /*!< An internal process is ongoing on the TIM channel */
} HAL_TIM_ChannelState;

typedef enum {
    HAL_TIM_ACTIVE_CHANNEL_1       = 0x01U, /*!< The active channel is 1     */
    HAL_TIM_ACTIVE_CHANNEL_2       = 0x02U, /*!< The active channel is 2     */
    HAL_TIM_ACTIVE_CHANNEL_3       = 0x04U, /*!< The active channel is 3     */
    HAL_TIM_ACTIVE_CHANNEL_4       = 0x08U, /*!< The active channel is 4     */
    HAL_TIM_ACTIVE_CHANNEL_5       = 0x10U, /*!< The active channel is 5     */
    HAL_TIM_ACTIVE_CHANNEL_6       = 0x20U, /*!< The active channel is 6     */
    HAL_TIM_ACTIVE_CHANNEL_CLEARED = 0x00U  /*!< All active channels cleared */
} HAL_TIM_ActiveChannel;

/* The counter of a timer instance is disabled only if all the CCx and CCxN
   channels have been disabled */
#define TIM_CCER_CCxE_MASK  ((uint32_t) (TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E | TIM_CCER_CC5E | TIM_CCER_CC6E))
#define TIM_CCER_CCxNE_MASK ((uint32_t) (TIM_CCER_CC1NE | TIM_CCER_CC2NE | TIM_CCER_CC3NE | TIM_CCER_CC4NE))

typedef struct {
    TIM_TypeDef          *Instance;
    TIM_Init              Init;
    HAL_TIM_ActiveChannel Channel;

    HAL_TIM_ChannelState ChannelState[6];
    HAL_TIM_ChannelState ChannelNState[4];
    // TODO: hdma handle
    HAL_TIM_State State;
} TIM_Handle;

HAL_Status HAL_TIM_Init(TIM_Handle *handle);
HAL_Status HAL_TIM_DeInit(TIM_Handle *handle);

HAL_Status HAL_TIM_Start_IT(TIM_Handle *handle);
HAL_Status HAL_TIM_Stop_IT(TIM_Handle *handle);

// input capture
// output compare
// PWM generation
// one-pulse mode output
// encoder interface

void HAL_TIM_IRQHandler(TIM_Handle *handle);

void TIM_SetConfig(TIM_TypeDef *TIMx, const TIM_Init *Structure);

#endif // STM32H7RS_HAL_TIM_H
