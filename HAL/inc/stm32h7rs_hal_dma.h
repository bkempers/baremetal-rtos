#ifndef STM32H7RS_HAL_DMA_H
#define STM32H7RS_HAL_DMA_H

#include "stm32h7rs_hal.h"
#include "stm32h7rsxx.h"
#include <stdint.h>

typedef struct {
    uint32_t Request;
    uint32_t BlockRequest;
    uint32_t Direction;
    uint32_t SrcInc;
    uint32_t DestInc;
    uint32_t SrcDataWidth;
    uint32_t DestDataWidth;
    uint32_t Priority;
    uint32_t SrcBurstLen;
    uint32_t DestBurstLen;
    uint32_t TransferAllocatedPort;
    uint32_t TransferEventMode;
    uint32_t Mode;
} DMA_Init;

typedef struct {
    uint32_t Priority;
    uint32_t LinkStepMode;
    uint32_t LinkAllocatedPort;
    uint32_t TransferEventMode;
    uint32_t LinkedListMode;
} DMA_LinkedList;

typedef enum
{
  HAL_DMA_STATE_RESET   = 0x00U, /*!< DMA not yet initialized or disabled */
  HAL_DMA_STATE_READY   = 0x01U, /*!< DMA initialized and ready for use   */
  HAL_DMA_STATE_BUSY    = 0x02U, /*!< DMA process is ongoing              */
  HAL_DMA_STATE_ERROR   = 0x03U, /*!< DMA error state                     */
  HAL_DMA_STATE_ABORT   = 0x04U, /*!< DMA Abort state                     */
  HAL_DMA_STATE_SUSPEND = 0x05U, /*!< DMA Suspend state                   */
} HAL_DMA_State;

typedef enum
{
  HAL_DMA_FULL_TRANSFER = 0x00U, /*!< Full channel transfer */
  HAL_DMA_HALF_TRANSFER = 0x01U, /*!< Half channel transfer */

} HAL_DMA_LevelComplete;

typedef struct __DMA_Handle {
    DMA_Channel_TypeDef *Instance;
    DMA_Init Init;
    DMA_LinkedList LinkedList;
    uint32_t Mode;
    __IO HAL_DMA_State State;
    __IO uint32_t ErrorCode;
    void *Parent;

  void (* XferCpltCallback)(struct __DMA_Handle *handle);     /*!< DMA transfer complete callback          */
  void (* XferHalfCpltCallback)(struct __DMA_Handle *handle); /*!< DMA half transfer complete callback     */
  void (* XferErrorCallback)(struct __DMA_Handle *handle);    /*!< DMA transfer error callback             */
  void (* XferAbortCallback)(struct __DMA_Handle *handle);    /*!< DMA transfer Abort callback             */
  void (* XferSuspendCallback)(struct __DMA_Handle *handle);  /*!< DMA transfer Suspend callback           */

    // linked list queue?
} DMA_Handle;

#define HAL_DMA_ERROR_NONE             (0x0000U) /*!< No error                      */
#define HAL_DMA_ERROR_DTE              (0x0001U) /*!< Data transfer error           */
#define HAL_DMA_ERROR_ULE              (0x0002U) /*!< Update linked-list item error */
#define HAL_DMA_ERROR_USE              (0x0004U) /*!< User setting error            */
#define HAL_DMA_ERROR_TO               (0x0008U) /*!< Trigger overrun error         */
#define HAL_DMA_ERROR_TIMEOUT          (0x0010U) /*!< Timeout error                 */
#define HAL_DMA_ERROR_NO_XFER          (0x0020U) /*!< No transfer ongoing error     */
#define HAL_DMA_ERROR_BUSY             (0x0040U) /*!< Busy error                    */
#define HAL_DMA_ERROR_INVALID_CALLBACK (0x0080U) /*!< Invalid callback error        */
#define HAL_DMA_ERROR_NOT_SUPPORTED    (0x0100U) /*!< Not supported mode            */

#define DMA_PERIPH_TO_MEMORY 0x00000000U             /*!< Peripheral to memory direction */
#define DMA_MEMORY_TO_PERIPH DMA_CTR2_DREQ           /*!< Memory to peripheral direction */
#define DMA_MEMORY_TO_MEMORY DMA_CTR2_SWREQ          /*!< Memory to memory direction     */


#define HAL_TIMEOUT_DMA_ABORT          (0x00000005U) /* DMA channel abort timeout 5 milli-second */
#define HAL_DMA_CHANNEL_START          (0x00000050U) /* DMA channel offset                       */
#define HAL_DMA_CHANNEL_SIZE           (0x00000080U) /* DMA channel size                         */
#define HAL_DMA_OFFSET_MASK            (0x00000FFFU) /* DMA channel offset mask                  */
#define DMA_CHANNEL_ATTR_PRIV_MASK     (0x01000000U) /* DMA channel privilege               */
#define DMA_CHANNEL_PRIV_VAL_POS                 0U
#define DMA_CHANNEL_BURST_MIN          (0x00000001U) /* DMA channel minimum burst size           */
#define DMA_CHANNEL_BURST_MAX          (0x00000040U) /* DMA channel maximum burst size           */

HAL_Status HAL_DMA_Init(DMA_Handle *handle);
HAL_Status HAL_DMA_DeInit(DMA_Handle *handle);

HAL_Status HAL_DMA_Start(DMA_Handle *handle, uint32_t src, uint32_t dest, uint32_t data_size);
HAL_Status HAL_DMA_Start_IT(DMA_Handle *handle, uint32_t src, uint32_t dest, uint32_t data_size);

HAL_Status HAL_DMA_Abort(DMA_Handle *const handle);
HAL_Status HAL_DMA_Abort_IT(DMA_Handle *const handle);
HAL_Status HAL_DMA_PollForTransfer(DMA_Handle *const handle, HAL_DMA_LevelComplete CompleteLevel, uint32_t Timeout);

void HAL_DMA_IRQHandler(DMA_Handle *const handle);

HAL_DMA_State HAL_DMA_GetState(DMA_Handle const *const handle);
uint32_t HAL_DMA_GetError(DMA_Handle const *const handle);

#define GET_DMA_INSTANCE(__HANDLE__) \
  ((DMA_TypeDef *)((uint32_t)((__HANDLE__)->Instance) & (~HAL_DMA_OFFSET_MASK)))

#define GET_DMA_CHANNEL(__HANDLE__) \
  ((((uint32_t)((__HANDLE__)->Instance) & HAL_DMA_OFFSET_MASK) - HAL_DMA_CHANNEL_START) / HAL_DMA_CHANNEL_SIZE)

#define IS_DMA_GLOBAL_ACTIVE_FLAG(INSTANCE, GLOBAL_FLAG) \
  (((INSTANCE)->MISR & (GLOBAL_FLAG)))


#endif // STM32H7RS_HAL_DMA_H
