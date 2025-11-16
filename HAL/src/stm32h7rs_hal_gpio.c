#include "stm32h7rs_hal_gpio.h"

void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, const GPIO_Init *GPIO_InitStruct)
{
    uint32_t position = 0;
    uint32_t temp = 0;
    
    while ((GPIO_InitStruct->Pin >> position) != 0) {
        if ((GPIO_InitStruct->Pin & (1U << position)) != 0) {
            // Configure mode
            temp = GPIOx->MODER;
            temp &= ~(3U << (position * 2));  // Clear 2 bits
            temp |= (GPIO_InitStruct->Mode << (position * 2));
            GPIOx->MODER = temp;
            
            // Configure output type (push-pull or open-drain)
            if (GPIO_InitStruct->Mode == GPIO_MODE_OUTPUT_PP || GPIO_InitStruct->Mode == GPIO_MODE_ALT_FUNC_PP) {
                GPIOx->OTYPER &= ~(1U << position);  // Push-pull
            } else if (GPIO_InitStruct->Mode == GPIO_MODE_OUTPUT_OD || GPIO_InitStruct->Mode == GPIO_MODE_ALT_FUNC_OD) {
                GPIOx->OTYPER |= (1U << position);   // Open-drain
            }
            
            // Configure speed
            temp = GPIOx->OSPEEDR;
            temp &= ~(3U << (position * 2));
            temp |= (GPIO_InitStruct->Speed << (position * 2));
            GPIOx->OSPEEDR = temp;
            
            // Configure pull-up/pull-down
            temp = GPIOx->PUPDR;
            temp &= ~(3U << (position * 2));
            temp |= (GPIO_InitStruct->Pull << (position * 2));
            GPIOx->PUPDR = temp;
            
            // Configure alternate function
            if (GPIO_InitStruct->Mode == GPIO_MODE_ALT_FUNC_PP || GPIO_InitStruct->Mode == GPIO_MODE_ALT_FUNC_OD) {
                temp = (position < 8) ? GPIOx->AFR[0] : GPIOx->AFR[1];
                uint32_t af_position = (position < 8) ? position : (position - 8);
                temp &= ~(0xFU << (af_position * 4));
                temp |= (GPIO_InitStruct->Alternate << (af_position * 4));
                if (position < 8) {
                    GPIOx->AFR[0] = temp;
                } else {
                    GPIOx->AFR[1] = temp;
                }
            }
        }
        position++;
    }
}

//TODO: ADD LATER
void HAL_GPIO_DeInit(GPIO_TypeDef *GPIOx, uint16_t pin)
{
    return;
}

GPIO_PinState HAL_GPIO_Read(GPIO_TypeDef *GPIOx, uint16_t pin)
{
    GPIO_PinState ret = (GPIOx->ODR & pin) ? GPIO_PIN_SET : GPIO_PIN_RESET;
    return ret;
}

void HAL_GPIO_Write(GPIO_TypeDef *GPIOx, uint16_t pin, GPIO_PinState state)
{
    if (state != GPIO_PIN_RESET) {
        GPIOx->BSRR = (uint32_t)pin;
    } else {
        GPIOx->BRR = (uint32_t)pin;
    }
}

void HAL_GPIO_Toggle(GPIO_TypeDef *GPIOx, uint16_t pin)
{
   GPIOx->ODR ^= pin; 
}

