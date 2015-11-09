#include "can_api.h"

void initCan(can_t* obj, PinName rd, PinName td, uint32_t prescaler, uint32_t mode);

static can_t* object;
static uint32_t can_irq_id = 0;
static can_irq_handler irq_handler;

void TX_IRQHandler(void);
void RX0_IRQHandler(void);
void RX1_IRQHandler(void);

/**
 * @brief CAN Peripheral Initialization
 * @param obj can_t object
 * @param rd: RX pin name
 * @param td: TX pin name
 * @param prescaler: Value to use for CAN clock prescaler
 * @param mode: CAN Bus mode
 * @retval None
 */
void initCan(can_t* obj, PinName rd, PinName td, uint32_t prescaler, uint32_t mode) {

    //Retain pointer to object for MSP init and deinit
    object = obj;

    obj->handle.Instance = CAN;
    obj->handle.pRxMsg = &(obj->rxMsg);
    obj->handle.pTxMsg = &(obj->txMsg);
    obj->rxPin = rd;
    obj->txPin = td;

    //AHB1 clock = 36MHZ
    //1 + 8 + 3 = 12TQ (75% Sample Point)
    //36MHZ / (12 * 24) = 125kbps (default bit rate)
    //Set the Prescaler to 24
    obj->handle.Init.Prescaler = prescaler;
    //Set resynchronization stretch period to 1 Quanta
    obj->handle.Init.SJW = CAN_SJW_1TQ;
    //Specify Bit Segment 1 length to 8 Time Quanta
    obj->handle.Init.BS1 = CAN_BS1_8TQ;
    //Specify Bit Segment 2 length to 3 Time Quanta
    obj->handle.Init.BS2 = CAN_BS2_3TQ;

    //Disable Time Triggered CAN
    obj->handle.Init.TTCM = DISABLE;
    //Enable automatic bus off recovery
    obj->handle.Init.ABOM = ENABLE;
    //Disable Automatic Wake Up Mode
    obj->handle.Init.AWUM = DISABLE;
    //Disable Non-automatic retransmission
    obj->handle.Init.NART = DISABLE;
    //Disable Recieve FIFO Locked Mode
    //Keeps most recent message in case of Overrun
    obj->handle.Init.RFLM = DISABLE;
    //Disable Transmit FIFO Priority
    obj->handle.Init.TXFP = DISABLE;
    //Startup in Silent Mode
    obj->handle.Init.Mode = mode;

    HAL_CAN_Init(&(obj->handle));
}

void can_init(can_t* obj, PinName rd, PinName td) {
    initCan(obj, rd, td, 24, CAN_MODE_NORMAL);

    NVIC_SetVector(USB_HP_CAN_TX_IRQn, (uint32_t)&TX_IRQHandler);
    NVIC_SetVector(USB_LP_CAN_RX0_IRQn, (uint32_t)&RX0_IRQHandler);
    NVIC_SetVector(CAN_RX1_IRQn, (uint32_t)&RX1_IRQHandler);
    NVIC_EnableIRQ(USB_HP_CAN_TX_IRQn);
    NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
    NVIC_EnableIRQ(CAN_RX1_IRQn);

    __HAL_CAN_ENABLE_IT(&obj->handle, CAN_IT_FMP0);
    __HAL_CAN_ENABLE_IT(&obj->handle, CAN_IT_FMP1);
    __HAL_CAN_ENABLE_IT(&obj->handle, CAN_IT_TME);
}

/**
 * @brief CAN Peripheral Deinitialization
 * @param obj can_t object
 * @retval None
 */
void can_free(can_t* obj) {
    HAL_CAN_DeInit(&(obj->handle));
}

/**
 * @brief CAN MSP Initialization
 * @param hcan: CAN_HandleTypeDef pointer
 * @retval None
 */
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan) {
    GPIO_InitTypeDef gpioInit;
    gpioInit.Mode = GPIO_MODE_AF_PP;
    gpioInit.Speed = GPIO_SPEED_HIGH;
    gpioInit.Pull = GPIO_NOPULL;
    gpioInit.Alternate = GPIO_AF9_CAN;
    /**
     *  RX      TX
     *  PA11    PA12
     *  PB8     PB9
     *  PD0     PD1 (Not Available on LQFP64)
     */

    //Enable the CAN Peripheral Clock
    __CAN_CLK_ENABLE();

    if (object->rxPin == PA_11 && object->txPin == PA_12) {
        //Enable GPIOA Peripheral Clock
        __GPIOA_CLK_ENABLE();
        gpioInit.Pin = GPIO_PIN_11 | GPIO_PIN_12;
        HAL_GPIO_Init(GPIOA, &gpioInit);
    }
    else if (object->rxPin == PB_8 && object->txPin == PB_9) {
        __GPIOB_CLK_ENABLE();
        gpioInit.Pin = GPIO_PIN_8 | GPIO_PIN_9;
        HAL_GPIO_Init(GPIOB, &gpioInit);
    }
    else {
        return;
    }
    //Setup the CAN IRQs
    HAL_NVIC_SetPriority(USB_LP_CAN_RX0_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN_RX0_IRQn);
    HAL_NVIC_SetPriority(USB_HP_CAN_TX_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USB_HP_CAN_TX_IRQn);
}

/**
 * @brief CAN MSP Deinit
 *        De-initializes the CAN Peripheral
 * @param hcan CAN_HandleTypeDef pointer
 * @retval None
 */
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan) {
    //Disable the peripheral clock
    __CAN_CLK_DISABLE();

    if (object->rxPin == PA_11 && object->txPin == PA_12) {
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);
    }
    else if(object->rxPin == PB_8 && object->txPin == PB_9) {
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_8|GPIO_PIN_9);
    }
    //Disable CAN IRQs
    HAL_NVIC_DisableIRQ(USB_LP_CAN_RX0_IRQn);
    HAL_NVIC_DisableIRQ(USB_HP_CAN_TX_IRQn);
}

/**
 * @brief Generic IRQ handler to pass the IRQ up to the handling object
 * @param: obj: Pointer to the can_t object from which the IRQ was triggered
 * @param: type: The CanIrqType for the IRQ recieved
 */
void CAN_IRQHandler(can_t* obj, CanIrqType type) {
    irq_handler(can_irq_id, type);
}

/**
 * @brief Recieves a message from the bus
 * @param hcan: Pointer to the can bus handle to receive from
 * @prarm FIFONumber: The CAN_FIFO to receieve from
 * @retval None
 */
void RX_Message(CAN_HandleTypeDef* hcan, uint32_t FIFONumber) { ;

    hcan->pRxMsg->IDE = (uint8_t)0x04 & hcan->Instance->sFIFOMailBox[FIFONumber].RIR;
    if (hcan->pRxMsg->IDE == CAN_ID_STD)
    {
        hcan->pRxMsg->StdId = (uint32_t)0x000007FF & (hcan->Instance->sFIFOMailBox[FIFONumber].RIR >> 21);
    }
    else
    {
        hcan->pRxMsg->ExtId = (uint32_t)0x1FFFFFFF & (hcan->Instance->sFIFOMailBox[FIFONumber].RIR >> 3);
    }

    hcan->pRxMsg->RTR = (uint8_t)0x02 & hcan->Instance->sFIFOMailBox[FIFONumber].RIR;
    /* Get the DLC */
    hcan->pRxMsg->DLC = (uint8_t)0x0F & hcan->Instance->sFIFOMailBox[FIFONumber].RDTR;
    /* Get the FMI */
    hcan->pRxMsg->FMI = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[FIFONumber].RDTR >> 8);
    /* Get the data field */
    hcan->pRxMsg->Data[0] = (uint8_t)0xFF & hcan->Instance->sFIFOMailBox[FIFONumber].RDLR;
    hcan->pRxMsg->Data[1] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[FIFONumber].RDLR >> 8);
    hcan->pRxMsg->Data[2] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[FIFONumber].RDLR >> 16);
    hcan->pRxMsg->Data[3] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[FIFONumber].RDLR >> 24);
    hcan->pRxMsg->Data[4] = (uint8_t)0xFF & hcan->Instance->sFIFOMailBox[FIFONumber].RDHR;
    hcan->pRxMsg->Data[5] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[FIFONumber].RDHR >> 8);
    hcan->pRxMsg->Data[6] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[FIFONumber].RDHR >> 16);
    hcan->pRxMsg->Data[7] = (uint8_t)0xFF & (hcan->Instance->sFIFOMailBox[FIFONumber].RDHR >> 24);
    /* Release the FIFO */
    __HAL_CAN_FIFO_RELEASE(hcan, FIFONumber);
}

/**
 * @brief Handles CAN1 RX0 Interrupt
 * @param None
 * @retval None
 */
void RX0_IRQHandler(void) {
    RX_Message(&object->handle, CAN_FIFO0);
    object->rxMsgPending = 1;
    CAN_IRQHandler(object, IRQ_RX);
}

/**
 * @brief Handles CAN1 RX1 Interrupt
 * @param None
 * @retval None
 */
void RX1_IRQHandler(void) {
    RX_Message(&object->handle, CAN_FIFO1);
    object->rxMsgPending;
    CAN_IRQHandler(object, IRQ_RX);
}

/**
 * @brief Handles CAN1 TX Interrupts
 * @param None
 * @retval None
 */
void TX_IRQHandler(void) {
   // HAL_CAN_IRQHandler(&(object->handle));
    CAN_HandleTypeDef* hcan = &object->handle;

    //We must explicitly compute the status of each flag
    // or for some reason GCC is optimizing the calc out
    // always assuming its always set
    volatile uint8_t RQCP0 = __HAL_CAN_GET_FLAG(hcan, CAN_FLAG_RQCP0);
    volatile uint8_t RQCP1 = __HAL_CAN_GET_FLAG(hcan, CAN_FLAG_RQCP1);
    volatile uint8_t RQCP2 = __HAL_CAN_GET_FLAG(hcan, CAN_FLAG_RQCP2);
    if (RQCP0) {
        //clear the interrupt flag
        __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RQCP0);
    }
    if (RQCP1) {
        //clear the interrupt flag
        __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RQCP1);
    }
    if (RQCP2) {
        //clear the interrupt flag
        __HAL_CAN_CLEAR_FLAG(hcan, CAN_FLAG_RQCP2);
    }
    CAN_IRQHandler(object, IRQ_TX);
}

/**
 * @brief initialize irq handlers
 * @param obj: can_t object pointer
 * @param handler: can_irq_handler type
 * @param id: unique id of the irq handler object
 * @retval None
 */
void can_irq_init(can_t* obj, can_irq_handler handler, uint32_t id) {
    irq_handler = handler;
    can_irq_id = id;
}

/**
 * @brief free the irq handlers
 * @param: obj: can_t object pointer
 * @retval None
 */
void can_irq_free(can_t* obj) {
    can_irq_id = 0;
}

/**
 * @brief enable can irq handling
 * @param obj: can_t object pointer
 * @param irq: CanIrqType handler
 * @param enable: enable the irq
 */
void can_irq_set(can_t* obj, CanIrqType irq, uint32_t enable) {
    //Not implemented.
    //The higher layer protocol handles calling the actual user handler
    // And we always need the low level interrupt enabled to handle asynchronous tasks
}

/**
 * @brief set the frequency of the CAN Bus
 *          This method should not be called from an interrupt routine
 *          doing so may cause a race condition, resulting in lost messages
 * @param obj: can_t object pointer
 * @param hz: The frequency in hertz
 * @retval returns 1 if successful, else 0
 */
int can_frequency(can_t* obj, int hz) {
    //The bit timing register is protected, so we need to put the device
    //  into init mode again to be able to modify the frequency
    uint32_t prescale;
    switch(hz) {
        case 10000:
            prescale = 300;
            break;
        case 20000:
            prescale = 150;
            break;
        case 50000:
            prescale = 60;
            break;
        case 100000:
            prescale = 30;
            break;
        case 125000:
            prescale = 24;
            break;
        case 250000:
            prescale = 12;
            break;
        case 500000:
            prescale = 6;
            break;
        case 1000000:
            prescale = 3;
            break;
        default:
            //We don't support any other speeds
            return 0;
    }
    //De-init the bus
    can_free(obj);
    //Initialize the bus with the new frequency
    initCan(obj, obj->rxPin, obj->txPin, prescale, obj->handle.Init.Mode);
    return 1;
}

int CAN_Transmit(CAN_HandleTypeDef* hcan) {
    uint32_t transmitmailbox = CAN_TXSTATUS_NOMAILBOX;

    /* Check the parameters */
    assert_param(IS_CAN_IDTYPE(hcan->pTxMsg->IDE));
    assert_param(IS_CAN_RTR(hcan->pTxMsg->RTR));
    assert_param(IS_CAN_DLC(hcan->pTxMsg->DLC));

    if ((hcan->State != HAL_CAN_STATE_RESET) || (hcan->State != HAL_CAN_STATE_ERROR) ||
        (hcan->State != HAL_CAN_STATE_TIMEOUT)) {
        __HAL_LOCK(hcan);
        volatile uint8_t mb0Empty = (hcan->Instance->TSR & CAN_TSR_TME0) == CAN_TSR_TME0;
        volatile uint8_t mb1Empty = (hcan->Instance->TSR & CAN_TSR_TME1) == CAN_TSR_TME1;
        volatile uint8_t mb2Empty = (hcan->Instance->TSR & CAN_TSR_TME2) == CAN_TSR_TME2;

        if (mb0Empty) {
            transmitmailbox = 0;
        }
        else if (mb1Empty) {
            transmitmailbox = 1;
        }
        else if (mb2Empty) {
            transmitmailbox = 2;
        }
        else {
            return 0;
        }
        /* Set up the Id */
        hcan->Instance->sTxMailBox[transmitmailbox].TIR &= CAN_TI0R_TXRQ;
        if(hcan->pTxMsg->IDE == CAN_ID_STD)
        {
            assert_param(IS_CAN_STDID(hcan->pTxMsg->StdId));
            hcan->Instance->sTxMailBox[transmitmailbox].TIR |= ((hcan->pTxMsg->StdId << 21) | \
                                                  hcan->pTxMsg->RTR);
        }
        else
        {
            assert_param(IS_CAN_EXTID(hcan->pTxMsg->ExtId));
            hcan->Instance->sTxMailBox[transmitmailbox].TIR |= ((hcan->pTxMsg->ExtId << 3) | \
                                                  hcan->pTxMsg->IDE | \
                                                  hcan->pTxMsg->RTR);
        }

        /* Set up the DLC */
        hcan->pTxMsg->DLC &= (uint8_t)0x0000000F;
        hcan->Instance->sTxMailBox[transmitmailbox].TDTR &= (uint32_t)0xFFFFFFF0;
        hcan->Instance->sTxMailBox[transmitmailbox].TDTR |= hcan->pTxMsg->DLC;

        /* Set up the data field */
        hcan->Instance->sTxMailBox[transmitmailbox].TDLR = (((uint32_t)hcan->pTxMsg->Data[3] << 24) |
                                                            ((uint32_t)hcan->pTxMsg->Data[2] << 16) |
                                                            ((uint32_t)hcan->pTxMsg->Data[1] << 8) |
                                                            ((uint32_t)hcan->pTxMsg->Data[0]));
        hcan->Instance->sTxMailBox[transmitmailbox].TDHR = (((uint32_t)hcan->pTxMsg->Data[7] << 24) |
                                                            ((uint32_t)hcan->pTxMsg->Data[6] << 16) |
                                                            ((uint32_t)hcan->pTxMsg->Data[5] << 8) |
                                                            ((uint32_t)hcan->pTxMsg->Data[4]));

        /* Set CAN error code to none */
        hcan->ErrorCode = HAL_CAN_ERROR_NONE;

        __HAL_UNLOCK(hcan);

        /* Request transmission */
        hcan->Instance->sTxMailBox[transmitmailbox].TIR |= CAN_TI0R_TXRQ;
        return 1;
    }
    return 0;
}

/**
 * @brief Synchronously write a message on the bus
 * @param obj: can_t object pointer
 * @param msg: CAN_Message,
 * @retval: returns 1 if successful, 0 otherwise
 */
int can_write(can_t* obj, CAN_Message msg, int cc) {
    obj->txMsg.StdId = msg.id;
    if (msg.format == CANExtended || msg.format == CANAny) {
        obj->txMsg.IDE = CAN_ID_EXT;
        obj->txMsg.ExtId = msg.id;
        obj->txMsg.StdId = 0;
    }
    else {
        obj->txMsg.ExtId = CAN_ID_STD;
        obj->txMsg.IDE = 0;
    }
    obj->txMsg.DLC = msg.len;
    obj->txMsg.RTR = msg.type;

    for (uint8_t i = 0; i < msg.len; i++) {
        obj->txMsg.Data[i] = msg.data[i];
    }
    //Clear out any extraneous data
    for (uint8_t i = msg.len; i < 8; i++) {
        obj->txMsg.Data[i] = 0;
    }
    return CAN_Transmit(&obj->handle);
}

/**
 * @brief Read the last Received message from the CAN bus
 * @param obj: can_t object pointer
 * @param msg: Pointer to a CAN_Message struct
 * @retval: returns 1 if successful, 0 otherwise
 */
int can_read(can_t* obj,CAN_Message* msg, int handle) {
    if (obj->rxMsgPending) {
        msg->format = obj->rxMsg.IDE == CAN_ID_EXT ? CANExtended : CANStandard;
        msg->type = obj->rxMsg.RTR ? CANRemote : CANData;
        msg->id = obj->rxMsg.IDE == CAN_ID_EXT ? obj->rxMsg.ExtId : obj->rxMsg.StdId;
        for (int i = 0; i < 8; i++) {
            msg->data[i] = obj->rxMsg.Data[i];
        }
        msg->len = obj->rxMsg.DLC;
        obj->rxMsgPending = 0;
        return 1;
    }
    return 0;
}

/**
 * @brief Change the operational mode of the CAN Bus
 * @param mode: The requested operational mode
 * @retval: returns 1 if successful, 0 otherwise
 */
int can_mode(can_t* obj, CanMode mode) {
    switch (mode) {
        case MODE_RESET:
            if (obj->handle.State != HAL_CAN_STATE_RESET)
                can_free(obj);
            return 1;
            break;
        case MODE_NORMAL:
            if (obj->handle.Init.Mode != CAN_MODE_NORMAL) {
                can_free(obj);
                initCan(obj, obj->rxPin, obj->txPin, obj->handle.Init.Prescaler, CAN_MODE_NORMAL);
            }
            return 1;
            break;
        case MODE_SILENT:
            if (obj->handle.Init.Mode != CAN_MODE_SILENT) {
                can_free(obj);
                initCan(obj, obj->rxPin, obj->txPin, obj->handle.Init.Prescaler, CAN_MODE_SILENT);
            }
            return 1;
            break;
        case MODE_TEST_LOCAL:
            if (obj->handle.Init.Mode != CAN_MODE_SILENT_LOOPBACK) {
                can_free(obj);
                initCan(obj, obj->rxPin, obj->txPin, obj->handle.Init.Prescaler, CAN_MODE_SILENT_LOOPBACK);
            }
            return 1;
            break;
        case MODE_TEST_GLOBAL:
            if (obj->handle.Init.Mode != CAN_MODE_LOOPBACK) {
                can_free(obj);
                initCan(obj, obj->rxPin, obj->txPin, obj->handle.Init.Prescaler, CAN_MODE_LOOPBACK);
            }
            return 1;
            break;
        case MODE_TEST_SILENT:
            if (obj->handle.Init.Mode != CAN_MODE_SILENT_LOOPBACK) {
                can_free(obj);
                initCan(obj, obj->rxPin, obj->txPin, obj->handle.Init.Prescaler, CAN_MODE_SILENT_LOOPBACK);
            }
            return 1;
            break;
        default:
            return 0;
    }
    return 0;
}

/**
 * @brief Configure a Hardware filter
 * @param obj: can_t object pointer
 * @param id: The filter mask id
 * @param mask: The filter mask
 * @param format: The CANFormat (Standard/Extended)
 * @param handle: The filter number to use
 * @retval returns 1 if successful, 0 otherwise
 */
int can_filter(can_t* obj, uint32_t id, uint32_t mask, CANFormat format, int32_t handle) {

    //we only support filter banks #0 - 27
    if (handle >= 0 && handle <= 27) {
        CAN_FilterConfTypeDef filterConf;
        filterConf.FilterIdHigh = id >> 16;
        filterConf.FilterIdLow = id & 0xFFFF;
        filterConf.FilterMaskIdHigh = mask >> 16;
        filterConf.FilterMaskIdLow = mask & 0xFFFF;
        filterConf.FilterFIFOAssignment = CAN_FILTER_FIFO0;
        filterConf.FilterNumber = 0;
        filterConf.FilterMode = CAN_FILTERMODE_IDMASK;
        filterConf.FilterScale = CAN_FILTERSCALE_32BIT;
        filterConf.FilterActivation = ENABLE;
        filterConf.BankNumber = handle;

        HAL_CAN_ConfigFilter(&(obj->handle), &filterConf);
    }
    return 0;
}
/**
 * @brief resets the can bus
 * @param: obj: can_t object pointer to reset
 * @retval: none
 */
void can_reset(can_t* obj) {
    can_free(obj);
    initCan(obj, obj->rxPin, obj->txPin, obj->handle.Init.Prescaler, obj->handle.Init.Mode);
}

/**
 * @brief retrieves the CAN RX Error Count
 * @param obj: can_t object pointer
 * @retval: the integer count of the can bus RX errors
 */
unsigned char can_rderror  (can_t* obj) {
    return (unsigned char)((obj->handle.Instance->ESR & CAN_ESR_REC) >> 24);
}

/**
 * @brief retrieves the CAN TX Error Count
 * @param obj: can_t object pointer
 * @retval: the integer count of the can bus TX errors
 */
unsigned char can_tderror  (can_t *obj) {
   return (unsigned char)((obj->handle.Instance->ESR & CAN_ESR_TEC) >> 16);
}

/**
 * @brief enable silent monitoring mode
 * @param silent: Boolean, true to enable, false to disable
 * @retval: none
 */
void can_monitor(can_t* obj, int silent) {
    if (silent && (obj->handle.Init.Mode != CAN_MODE_LOOPBACK)) {
        can_free(obj);
        initCan(obj, obj->rxPin, obj->txPin, obj->handle.Init.Prescaler, CAN_MODE_LOOPBACK);
    }
    else if (!silent && (obj->handle.Init.Mode != CAN_MODE_NORMAL)) {
        can_free(obj);
        initCan(obj, obj->rxPin, obj->txPin, obj->handle.Init.Prescaler, CAN_MODE_NORMAL);
    }
}
