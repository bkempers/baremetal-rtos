#include "stm32h7rs_hal_tim.h"
#include "stm32h7s3xx.h"

HAL_Status HAL_TIM_Init(TIM_Handle *handle)
{
    if (handle == NULL) {
        return HAL_ERROR;
    }

    // register callback/reset

    handle->State = HAL_TIM_STATE_BUSY;

    TIM_SetConfig(handle->Instance, &handle->Init);

    handle->ChannelState[0] = HAL_TIM_CHANNEL_STATE_READY;
    handle->ChannelState[1] = HAL_TIM_CHANNEL_STATE_READY;
    handle->ChannelState[2] = HAL_TIM_CHANNEL_STATE_READY;
    handle->ChannelState[3] = HAL_TIM_CHANNEL_STATE_READY;
    handle->ChannelState[4] = HAL_TIM_CHANNEL_STATE_READY;
    handle->ChannelState[5] = HAL_TIM_CHANNEL_STATE_READY;

    handle->ChannelNState[0] = HAL_TIM_CHANNEL_STATE_READY;
    handle->ChannelNState[1] = HAL_TIM_CHANNEL_STATE_READY;
    handle->ChannelNState[2] = HAL_TIM_CHANNEL_STATE_READY;
    handle->ChannelNState[3] = HAL_TIM_CHANNEL_STATE_READY;

    handle->State = HAL_TIM_STATE_READY;
    return HAL_OK;
}

HAL_Status HAL_TIM_DeInit(TIM_Handle *handle)
{
    // TODO: implement
    return HAL_OK;
}

void TIM_SetConfig(TIM_TypeDef *TIMx, const TIM_Init *Structure)
{
    uint32_t tim_cr1;
    tim_cr1 = TIMx->CR1;

    if (IS_TIM_COUNTER_MODE_SELECT_INSTANCE(TIMx)) {
        /* Select the Counter Mode */
        tim_cr1 &= ~(TIM_CR1_DIR | TIM_CR1_CMS);
        tim_cr1 |= Structure->CounterMode;
    }

    if (IS_TIM_CLOCK_DIVISION_INSTANCE(TIMx)) {
        /* Set the clock division */
        tim_cr1 &= ~(TIM_CR1_CKD);
        tim_cr1 |= (uint32_t) Structure->ClockDivision;
    }

    /* Set the auto-reload preload */
    MODIFY_REG(tim_cr1, TIM_CR1_ARPE, Structure->AutoReloadPreload);

    /* Set the Autoreload value */
    TIMx->ARR = (uint32_t) Structure->Period;

    /* Set the Prescaler value */
    TIMx->PSC = Structure->Prescaler;

    if (IS_TIM_REPETITION_COUNTER_INSTANCE(TIMx)) {
        /* Set the Repetition Counter value */
        TIMx->RCR = Structure->RepetitionCounter;
    }

    /* Disable Update Event (UEV) with Update Generation (UG)
    by changing Update Request Source (URS) to avoid Update flag (UIF) */
    SET_BIT(TIMx->CR1, TIM_CR1_URS);

    /* Generate an update event to reload the Prescaler
    and the repetition counter (only for advanced timer) value immediately */
    TIMx->EGR = TIM_EGR_UG;

    TIMx->CR1 = tim_cr1;
}

// HAL_TIM_START (polling)
// HAL_TIM_STOP

HAL_Status HAL_TIM_Start_IT(TIM_Handle *handle)
{
    if (handle->State != HAL_TIM_STATE_READY) {
        return HAL_ERROR;
    }

    handle->State = HAL_TIM_STATE_BUSY;

    /* enable update interrupt */
    handle->Instance->DIER |= TIM_DIER_UIE;

    /* enable counter */
    handle->Instance->CR1 |= TIM_CR1_CEN;

    return HAL_OK;
}

HAL_Status HAL_TIM_Stop_IT(TIM_Handle *handle)
{
    /* disable TIM interrupt */
    handle->Instance->DIER &= ~TIM_DIER_UIE;

    if ((handle->Instance->CCER & TIM_CCER_CCxE_MASK) == 0UL) {
        if ((handle->Instance->CCER & TIM_CCER_CCxNE_MASK) == 0UL) {
            handle->Instance->CR1 &= ~TIM_CR1_CEN;
        }
    }

    handle->State = HAL_TIM_STATE_READY;

    return HAL_OK;
}

void HAL_TIM_IRQHandler(TIM_Handle *handle)
{
    uint32_t itsource = handle->Instance->DIER;
    uint32_t itflag   = handle->Instance->SR;

    /* Capture compare 1 event */
    if ((itflag & (TIM_SR_CC1IF)) == (TIM_SR_CC1IF)) {
        if ((itsource & (TIM_DIER_CC1IE)) == (TIM_DIER_CC1IE)) {
            handle->Instance->SR = ~TIM_SR_CC1IF;
            handle->Channel      = HAL_TIM_ACTIVE_CHANNEL_1;

            /* Input capture event */
            if ((handle->Instance->CCMR1 & TIM_CCMR1_CC1S) != 0x00U) {
                // HAL_TIM_IC_CaptureCallback(htim);
            }
            /* Output compare event */
            else {
                // HAL_TIM_OC_DelayElapsedCallback(htim);
                // HAL_TIM_PWM_PulseFinishedCallback(htim);
            }
            handle->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
        }
    }
    /* Capture compare 2 event */
    if ((itflag & (TIM_SR_CC2IF)) == (TIM_SR_CC2IF)) {
        if ((itsource & (TIM_DIER_CC2IE)) == (TIM_DIER_CC2IE)) {
            handle->Instance->SR = ~TIM_SR_CC2IF;
            handle->Channel      = HAL_TIM_ACTIVE_CHANNEL_2;

            /* Input capture event */
            if ((handle->Instance->CCMR1 & TIM_CCMR1_CC2S) != 0x00U) {
                // HAL_TIM_IC_CaptureCallback(htim);
            }
            /* Output compare event */
            else {
                // HAL_TIM_OC_DelayElapsedCallback(htim);
                // HAL_TIM_PWM_PulseFinishedCallback(htim);
            }
            handle->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
        }
    }
    /* Capture compare 3 event */
    if ((itflag & (TIM_SR_CC3IF)) == (TIM_SR_CC3IF)) {
        if ((itsource & (TIM_DIER_CC3IE)) == (TIM_DIER_CC3IE)) {
            handle->Instance->SR = ~TIM_SR_CC3IF;
            handle->Channel      = HAL_TIM_ACTIVE_CHANNEL_3;
            /* Input capture event */
            if ((handle->Instance->CCMR2 & TIM_CCMR2_CC3S) != 0x00U) {
                // HAL_TIM_IC_CaptureCallback(htim);
            }
            /* Output compare event */
            else {
                // HAL_TIM_OC_DelayElapsedCallback(htim);
                // HAL_TIM_PWM_PulseFinishedCallback(htim);
            }
            handle->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
        }
    }
    /* Capture compare 4 event */
    if ((itflag & (TIM_SR_CC4IF)) == (TIM_SR_CC4IF)) {
        if ((itsource & (TIM_DIER_CC4IE)) == (TIM_DIER_CC4IE)) {
            handle->Instance->SR = ~TIM_SR_CC4IF;
            handle->Channel      = HAL_TIM_ACTIVE_CHANNEL_4;
            /* Input capture event */
            if ((handle->Instance->CCMR2 & TIM_CCMR2_CC4S) != 0x00U) {
                // HAL_TIM_IC_CaptureCallback(htim);
            }
            /* Output compare event */
            else {
                // HAL_TIM_OC_DelayElapsedCallback(htim);
                // HAL_TIM_PWM_PulseFinishedCallback(htim);
            }
            handle->Channel = HAL_TIM_ACTIVE_CHANNEL_CLEARED;
        }
    }
    /* TIM Update event */
    if ((itflag & (TIM_SR_UIF)) == (TIM_SR_UIF)) {
        if ((itsource & (TIM_DIER_UIE)) == (TIM_DIER_UIE)) {
            handle->Instance->SR &= ~TIM_SR_UIF;
            handle->periodElapsedCallback(handle);
        }
    }
    /* TIM Break input event */
    if (((itflag & (TIM_SR_BIF)) == (TIM_SR_BIF)) || ((itflag & (TIM_SR_SBIF)) == (TIM_SR_SBIF))) {
        if ((itsource & (TIM_DIER_BIE)) == (TIM_DIER_BIE)) {
            handle->Instance->SR = ~(TIM_SR_BIF | TIM_SR_SBIF);
            // HAL_TIMEx_BreakCallback(htim);
        }
    }
    /* TIM Break2 input event */
    if ((itflag & (TIM_SR_B2IF)) == (TIM_SR_B2IF)) {
        if ((itsource & (TIM_SR_SBIF)) == (TIM_SR_SBIF)) {
            handle->Instance->SR = ~TIM_SR_B2IF;
            // HAL_TIMEx_Break2Callback(htim);
        }
    }
    /* TIM Trigger detection event */
    if ((itflag & (TIM_SR_TIF)) == (TIM_SR_TIF)) {
        if ((itsource & (TIM_DIER_TIE)) == (TIM_DIER_TIE)) {
            handle->Instance->SR = ~TIM_SR_TIF;
            // HAL_TIM_TriggerCallback(htim);
        }
    }
    /* TIM commutation event */
    if ((itflag & (TIM_SR_COMIF)) == (TIM_SR_COMIF)) {
        if ((itsource & (TIM_DIER_COMIE)) == (TIM_DIER_COMIE)) {
            handle->Instance->SR = ~TIM_SR_COMIF;
            // HAL_TIMEx_CommutCallback(htim);
        }
    }
    /* TIM Encoder index event */
    if ((itflag & (TIM_SR_IDXF)) == (TIM_SR_IDXF)) {
        if ((itsource & (TIM_DIER_IDXIE)) == (TIM_DIER_IDXIE)) {
            handle->Instance->SR = ~TIM_SR_IDXF;
            // HAL_TIMEx_EncoderIndexCallback(htim);
        }
    }
    /* TIM Direction change event */
    if ((itflag & (TIM_SR_DIRF)) == (TIM_SR_DIRF)) {
        if ((itsource & (TIM_DIER_DIRIE)) == (TIM_DIER_DIRIE)) {
            handle->Instance->SR = ~TIM_SR_DIRF;
            // HAL_TIMEx_DirectionChangeCallback(htim);
        }
    }
    /* TIM Index error event */
    if ((itflag & (TIM_SR_IERRF)) == (TIM_SR_IERRF)) {
        if ((itsource & (TIM_DIER_IERRIE)) == (TIM_DIER_IERRIE)) {
            handle->Instance->SR = ~TIM_SR_IERRF;
            // HAL_TIMEx_IndexErrorCallback(htim);
        }
    }
    /* TIM Transition error event */
    if ((itflag & (TIM_SR_TERRF)) == (TIM_SR_TERRF)) {
        if ((itsource & (TIM_DIER_TERRIE)) == (TIM_DIER_TERRIE)) {
            handle->Instance->SR = ~TIM_SR_TERRF;
            // HAL_TIMEx_TransitionErrorCallback(htim);
        }
    }
}

__weak void HAL_TIM_periodElapsedCallback(TIM_Handle *handle)
{
    UNUSED(handle);
}

uint32_t HAL_TIM_GetCounter(TIM_Handle *handle)
{
    if (handle == NULL) {
        return 0;
    }

    return handle->Instance->CNT;
}

void HAL_TIM_SetCounter(TIM_Handle *handle, uint32_t counter)
{
    if (handle == NULL) {
        return;
    }

    handle->Instance->CNT = counter;
}
