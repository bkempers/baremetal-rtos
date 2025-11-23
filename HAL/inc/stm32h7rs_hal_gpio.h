#ifndef STM32H7RS_HAL_GPIO_H
#define STM32H7RS_HAL_GPIO_H

#include "stm32h7rs_hal.h"
#include "stm32h7rsxx.h"
#include <stdint.h>

typedef struct {
    uint32_t Pin;
    uint32_t Mode;
    uint32_t Speed;
    uint32_t Pull;
    uint32_t Alternate;
} GPIO_Init;

typedef enum {
    GPIO_PIN_RESET = 0U,
    GPIO_PIN_SET
} GPIO_PinState;

#define GPIO_PIN_0   (1U << 0)
#define GPIO_PIN_1   (1U << 1)
#define GPIO_PIN_2   (1U << 2)
#define GPIO_PIN_3   (1U << 3)
#define GPIO_PIN_4   (1U << 4)
#define GPIO_PIN_5   (1U << 5)
#define GPIO_PIN_6   (1U << 6)
#define GPIO_PIN_7   (1U << 7)
#define GPIO_PIN_8   (1U << 8)
#define GPIO_PIN_9   (1U << 9)
#define GPIO_PIN_10  (1U << 10)
#define GPIO_PIN_11  (1U << 11)
#define GPIO_PIN_12  (1U << 12)
#define GPIO_PIN_13  (1U << 13)
#define GPIO_PIN_14  (1U << 14)
#define GPIO_PIN_15  (1U << 15)
#define GPIO_PIN_ALL 0xFFFF

#define GPIO_MODE_Pos 0u
#define GPIO_MODE     (0x3uL << GPIO_MODE_Pos)
#define MODE_INPUT    0x0uL
#define MODE_OUTPUT   0x1uL
#define MODE_ALT_FUNC 0x2uL
#define MODE_ANALOG   0x3uL

#define GPIO_TYPE_Pos 4u
#define TYPE_PP       0x0uL
#define TYPE_OD       0x1uL

#define GPIO_MODE_INPUT       (MODE_INPUT << GPIO_MODE_Pos)
#define GPIO_MODE_OUTPUT_PP   ((MODE_OUTPUT << GPIO_MODE_Pos) | (TYPE_PP << GPIO_TYPE_Pos))
#define GPIO_MODE_OUTPUT_OD   ((MODE_OUTPUT << GPIO_MODE_Pos) | (TYPE_OD << GPIO_TYPE_Pos))
#define GPIO_MODE_ALT_FUNC_PP ((MODE_ALT_FUNC << GPIO_MODE_Pos) | (TYPE_PP << GPIO_TYPE_Pos))
#define GPIO_MODE_ALT_FUNC_OD ((MODE_ALT_FUNC << GPIO_MODE_Pos) | (TYPE_OD << GPIO_TYPE_Pos))
#define GPIO_MODE_ANALOG      (MODE_ANALOG << GPIO_MODE_Pos)

#define GPIO_SPEED_Pos           8u
#define GPIO_SPEED_FREQ_LOW      0x00u
#define GPIO_SPEED_FREQ_MED      0x01u
#define GPIO_SPEED_FREQ_HIGH     0x02u
#define GPIO_SPEED_FEQ_VERY_HIGH 0x03u

#define GPIO_NOPULL_UP 0x00u
#define GPIO_PULL_UP   0x01u
#define GPIO_PULL_DOWN 0x02u

void HAL_GPIO_Init(GPIO_TypeDef *GPIOx, const GPIO_Init *GPIO_InitStruct);
void HAL_GPIO_DeInit(GPIO_TypeDef *GPIOx, uint16_t pin);

GPIO_PinState HAL_GPIO_Read(GPIO_TypeDef *GPIOx, uint16_t pin);
void          HAL_GPIO_Write(GPIO_TypeDef *GPIOx, uint16_t pin, GPIO_PinState state);
void          HAL_GPIO_Toggle(GPIO_TypeDef *GPIOx, uint16_t pin);

#endif // STM32H7RS_HAL_GPIO_H
