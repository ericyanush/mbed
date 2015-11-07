#include "can_api.h"

void initCan(can_t* obj, PinName rd, PinName td, uint32_t prescaler, uint32_t mode);

static can_t* object;
static uint32_t can_irq_id = 0;
static can_irq_handler irq_handler;


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
    can_filter(obj, 0, 0, CANStandard, 0);
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
 * @brief Handles CAN1 RX0 Interrupts
 *          Forwards Interrupts to ST HAL
 * @param None
 * @retval None
 */
void USB_LP_CAN1_RX0_IRQHandler(void) {
    HAL_CAN_IRQHandler(&(object->handle));
}

/**
 * @brief Handles CAN1 TX Interrupts
 *          Forwards Interrupts to ST HAL
 * @param None
 * @retval None
 */
void USB_HP_CAN_TX_IRQHandler(void) {
    HAL_CAN_IRQHandler(&(object->handle));
}

/**
 * @brief ST HAL RX Complete Callback Handler
 * @param hcan: CAN_HandleTypeDef pointer
 * @retval None
 */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* hcan) {

}

/**
 * @brief ST HAL TX Complete Callback Handler
 * @param hcan: CAN_HandleTypeDef Pointer
 * @retval None
 */
void HAL_CAN_TxCpltCallback(CAN_HandleTypeDef* hcan) {

}

/**
 * @brief ST HAL Error Callback Handler
 * @param hcan: CAN_HandleTypeDef Pointer
 * @retval None
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef* hcan) {

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
    if (HAL_CAN_Transmit(&(obj->handle), HAL_MAX_DELAY) == HAL_OK) {
        return 1;
    }
    return 0;
}

/**
 * @brief Synchronously read a message from the CAN bus
 * @param obj: can_t object pointer
 * @param msg: Pointer to a CAN_Message struct
 * @retval: returns 1 if successful, 0 otherwise
 */
int can_read(can_t* obj,CAN_Message* msg, int handle) {
    int recieve = HAL_CAN_Receive(&(obj->handle), 0, 0);
    if (recieve == HAL_OK) {
        msg->format = obj->rxMsg.IDE == CAN_ID_EXT ? CANExtended : CANStandard;
        msg->type = obj->rxMsg.RTR ? CANRemote : CANData;
        msg->id = obj->rxMsg.IDE == CAN_ID_EXT ? obj->rxMsg.ExtId : obj->rxMsg.StdId;
        for (int i = 0; i < 8; i++) {
            msg->data[i] = obj->rxMsg.Data[i];
        }
        msg->len = obj->rxMsg.DLC;
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
