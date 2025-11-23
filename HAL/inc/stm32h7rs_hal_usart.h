#ifndef STM32H7RS_HAL_USART_H
#define STM32H7RS_HAL_USART_H

#include "stm32h7rs_hal.h"
#include "stm32h7rs_hal_rcc.h"
#include "stm32h7rsxx.h"
#include <stdint.h>

typedef struct {
    uint32_t BaudRate;
    uint32_t WordLen;
    uint32_t StopBits;
    uint32_t Parity;
    uint32_t Mode;
    uint32_t CLKPolarity;
    uint32_t CLKPhase;
    uint32_t CLKLastBit;
    uint32_t ClockPrescaler;
} USART_Init;

typedef enum {
    HAL_USART_STATE_RESET      = 0x00U, /*!< Peripheral is not initialized */
    HAL_USART_STATE_READY      = 0x01U, /*!< Peripheral Initialized and ready for use       */
    HAL_USART_STATE_BUSY       = 0x02U, /*!< an internal process is ongoing    */
    HAL_USART_STATE_BUSY_TX    = 0x12U, /*!< Data Transmission process is ongoing */
    HAL_USART_STATE_BUSY_RX    = 0x22U, /*!< Data Reception process is ongoing */
    HAL_USART_STATE_BUSY_TX_RX = 0x32U, /*!< Data Transmission Reception process is ongoing */
    HAL_USART_STATE_TIMEOUT    = 0x03U, /*!< Timeout state */
    HAL_USART_STATE_ERROR      = 0x04U  /*!< Error    */
} HAL_USART_State;

typedef enum {
    USART_CLOCKSOURCE_PCLK1     = 0x00U, /*!< PCLK1 clock source */
    USART_CLOCKSOURCE_PCLK2     = 0x01U, /*!< PCLK2 clock source */
    USART_CLOCKSOURCE_PLL2Q     = 0x02U, /*!< PLL2Q clock source         */
    USART_CLOCKSOURCE_PLL3Q     = 0x04U, /*!< PLL3Q clock source         */
    USART_CLOCKSOURCE_HSI       = 0x08U, /*!< HSI clock source           */
    USART_CLOCKSOURCE_CSI       = 0x10U, /*!< CSI clock source           */
    USART_CLOCKSOURCE_LSE       = 0x20U, /*!< LSE clock source           */
    USART_CLOCKSOURCE_UNDEFINED = 0x80U  /*!< Undefined clock source     */
} USART_ClockSource;

typedef enum {
    USART_FLAG_PE    = USART_ISR_PE,
    USART_FLAG_FE    = USART_ISR_FE,
    USART_FLAG_NE    = USART_ISR_NE,
    USART_FLAG_ORE   = USART_ISR_ORE,
    USART_FLAG_IDLE  = USART_ISR_IDLE,
    USART_FLAG_RXNE  = USART_ISR_RXNE,
    USART_FLAG_TC    = USART_ISR_TC,
    USART_FLAG_TXE   = USART_ISR_TXE,
    USART_FLAG_TXFNF = USART_ISR_TXE_TXFNF,
    USART_FLAG_LBDF  = USART_ISR_LBDF,
    USART_FLAG_CTSIF = USART_ISR_CTSIF,
    USART_FLAG_RTOF  = USART_ISR_RTOF,
    USART_FLAG_EOBF  = USART_ISR_EOBF,
    USART_FLAG_UDR   = USART_ISR_UDR,
    USART_FLAG_ABRE  = USART_ISR_ABRE,
    USART_FLAG_ABRF  = USART_ISR_ABRF,
    USART_FLAG_BUSY  = USART_ISR_BUSY,
    USART_FLAG_CMF   = USART_ISR_CMF,
    USART_FLAG_SBKF  = USART_ISR_SBKF,
    USART_FLAG_RWU   = USART_ISR_RWU,
    USART_FLAG_WUF   = USART_ISR_WUF,
    USART_FLAG_TEACK = USART_ISR_TEACK,
    USART_FLAG_REACK = USART_ISR_REACK,
    USART_FLAG_TCBGT = USART_ISR_TCBGT
} USART_Flag;

#define USART_CLEAR_PEF    USART_ICR_PECF   /*!< Parity Error Clear Flag             */
#define USART_CLEAR_FEF    USART_ICR_FECF   /*!< Framing Error Clear Flag            */
#define USART_CLEAR_NEF    USART_ICR_NECF   /*!< Noise Error detected Clear Flag     */
#define USART_CLEAR_OREF   USART_ICR_ORECF  /*!< OverRun Error Clear Flag            */
#define USART_CLEAR_IDLEF  USART_ICR_IDLECF /*!< IDLE line detected Clear Flag       */
#define USART_CLEAR_TCF    USART_ICR_TCCF   /*!< Transmission Complete Clear Flag    */
#define USART_CLEAR_UDRF   USART_ICR_UDRCF  /*!< SPI slave underrun error Clear Flag */
#define USART_CLEAR_TXFECF USART_ICR_TXFECF /*!< TXFIFO Empty Clear Flag             */
#define USART_CLEAR_RTOF   USART_ICR_RTOCF  /*!< USART receiver timeout clear flag  */

#define USART_CR_MASK 0x0F00 // Extract CR number
#define USART_CR_POS  8      // Shift to get CR number
#define USART_IT_MASK 0x001F // Extract bit position (0-31)

#define USART_IT_PE    0x0028U /*!< USART parity error interruption                 */
#define USART_IT_TXE   0x0727U /*!< USART transmit data register empty interruption */
#define USART_IT_TXFNF 0x0727U /*!< USART TX FIFO not full interruption             */
#define USART_IT_TC    0x0626U /*!< USART transmission complete interruption        */
#define USART_IT_RXNE  0x0525U /*!< USART read data register not empty interruption */
#define USART_IT_RXFNE 0x0525U /*!< USART RXFIFO not empty interruption             */
#define USART_IT_IDLE  0x0424U /*!< USART idle interruption                         */
#define USART_IT_ERR   0x0060U /*!< USART error interruption                        */
#define USART_IT_ORE   0x0300U /*!< USART overrun error interruption                */
#define USART_IT_NE    0x0200U /*!< USART noise error interruption                  */
#define USART_IT_FE    0x0100U /*!< USART frame error interruption                  */
#define USART_IT_RXFF  0x183FU /*!< USART RXFIFO full interruption                  */
#define USART_IT_TXFE  0x173EU /*!< USART TXFIFO empty interruption                 */
#define USART_IT_RXFT  0x1A7CU /*!< USART RXFIFO threshold reached interruption     */
#define USART_IT_TXFT  0x1B77U /*!< USART TXFIFO threshold reached interruption     */

#define USART_PARITY_NONE 0x00000000U
#define USART_PARITY_EVEN USART_CR1_PCE
#define USART_PARITY_ODD  (USART_CR1_PCE | USART_CR1_PS)

#define USART_WORDLENGTH_7B (USART_CR1_M1)
#define USART_WORDLENGTH_8B (0x00000000U)
#define USART_WORDLENGTH_9B (USART_CR1_M0)

#define USART_SLAVEMODE_DISABLE 0x00000000U
#define USART_SLAVEMODE_ENABLE  USART_CR2_SLVEN

#define USART_ERROR_NONE 0x00u

typedef struct __USART_Handle {
    USART_TypeDef    *Instance;
    USART_Init        Init;
    const uint8_t    *txPointer;
    uint16_t          txSize;
    volatile uint16_t txCount;
    uint16_t          txProcessData;
    const uint8_t    *rxPointer;
    uint16_t          rxSize;
    volatile uint16_t rxCount;
    uint16_t          rxProcessData;
    uint16_t          mask;
    uint32_t          slaveMode;
    uint32_t          fifoMode;
    void (*RxISR)(struct __USART_Handle *handle);
    void (*TxISR)(struct __USART_Handle *handle);
    HAL_Lock                 Lock;
    volatile HAL_USART_State State;
    volatile uint16_t        errorCode;

} USART_Handle;

HAL_Status HAL_USART_Init(USART_Handle *handle);
HAL_Status HAL_USART_DeInit(USART_Handle *handle);

HAL_Status HAL_USART_Receive(USART_Handle *handle, uint8_t *rxPointer, uint16_t size, uint32_t timeout);
HAL_Status HAL_USART_Transmit(USART_Handle *handle, const uint8_t *txPointer, uint16_t size, uint32_t timeout);

HAL_Status HAL_USART_Receiver_IT(USART_Handle *handle, uint8_t *rxPointer, uint16_t size);
HAL_Status HAL_USART_Transmit_IT(USART_Handle *handle, const uint8_t *txPointer, uint16_t size);
HAL_Status HAL_USART_Receive_Transmit_IT(USART_Handle *handle, uint8_t *rxPointer, const uint8_t *txPointer, uint16_t size);

// void USART1_IRHandler(void);
// void USART2_IRHandler(void);
void USART3_IRHandler(void);
// void UART4_IRHandler(void);
// void UART5_IRHandler(void);
// void UART6_IRHandler(void);
// void UART7_IRHandler(void);
// void UART8_IRHandler(void);

#endif // STM32H7RS_HAL_USART_H
