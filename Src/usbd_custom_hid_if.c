/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : usbd_custom_hid_if.c
  * @version        : v1.0_Cube
  * @brief          : USB Device Custom HID interface file.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usbd_custom_hid_if.h"

/* USER CODE BEGIN INCLUDE */

/* USER CODE END INCLUDE */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
extern struct rx {
	char IDENTIFIER[16];
	uint16_t HEADER_SIZE;
	uint16_t DATA_SIZE;
	uint8_t DO[8];
	float DAC_VOLTAGE[2];
	uint16_t PWM_FRE[2];
	uint16_t PWM_DUTY[2];
} rx_data; 
struct tx {
	char IDENTIFIER[16];
	uint16_t HEADER_SIZE;
	uint16_t DATA_SIZE;
	uint8_t DI[8];
	float ADC_VOLTAGE[4];
};
extern struct tx tx_data;
extern ADC_HandleTypeDef hadc1;

extern DAC_HandleTypeDef hdac;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;
/* USER CODE END PV */

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @brief Usb device.
  * @{
  */

/** @addtogroup USBD_CUSTOM_HID
  * @{
  */

/** @defgroup USBD_CUSTOM_HID_Private_TypesDefinitions USBD_CUSTOM_HID_Private_TypesDefinitions
  * @brief Private types.
  * @{
  */

/* USER CODE BEGIN PRIVATE_TYPES */

/* USER CODE END PRIVATE_TYPES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Defines USBD_CUSTOM_HID_Private_Defines
  * @brief Private defines.
  * @{
  */

/* USER CODE BEGIN PRIVATE_DEFINES */

/* USER CODE END PRIVATE_DEFINES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Macros USBD_CUSTOM_HID_Private_Macros
  * @brief Private macros.
  * @{
  */

/* USER CODE BEGIN PRIVATE_MACRO */
void USB_RX_Interrupt(void);
/* USER CODE END PRIVATE_MACRO */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_Variables USBD_CUSTOM_HID_Private_Variables
  * @brief Private variables.
  * @{
  */

/** Usb HID report descriptor. */
__ALIGN_BEGIN static uint8_t CUSTOM_HID_ReportDesc_FS[USBD_CUSTOM_HID_REPORT_DESC_SIZE] __ALIGN_END =
{
  /* USER CODE BEGIN 0 */
	0x06, 0x00, 0xFF,      // Usage Page = 0xFF00 (Vendor Defined Page 1)
  0x09, 0x01,             // Usage (Vendor Usage 1)
  0xA1, 0x01,             // Collection (Application)
  // Input report
  0x19, 0x01,             // Usage Minimum
  0x29, 0x40,             // Usage Maximum
  0x15, 0x00,             // Logical Minimum (data bytes in the report may have minimum value = 0x00)
  0x26, 0xFF, 0x00,       // Logical Maximum (data bytes in the report may have maximum value = 0x00FF = unsigned 255)
  0x75, 0x08,             // Report Size: 8-bit field size
  0x95, CUSTOM_HID_EPIN_SIZE,// Report Count
  0x81, 0x02,             // Input (Data, Array, Abs)
  // Output report
  0x19, 0x01,             // Usage Minimum
  0x29, 0x40,              // Usage Maximum
  0x75, 0x08,             // Report Size: 8-bit field size
  0x95, CUSTOM_HID_EPOUT_SIZE,// Report Count
  0x91, 0x02,             // Output (Data, Array, Abs)
  /* USER CODE END 0 */
  0xC0    /*     END_COLLECTION	             */
};

/* USER CODE BEGIN PRIVATE_VARIABLES */

/* USER CODE END PRIVATE_VARIABLES */

/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Exported_Variables USBD_CUSTOM_HID_Exported_Variables
  * @brief Public variables.
  * @{
  */
extern USBD_HandleTypeDef hUsbDeviceFS;

/* USER CODE BEGIN EXPORTED_VARIABLES */

/* USER CODE END EXPORTED_VARIABLES */
/**
  * @}
  */

/** @defgroup USBD_CUSTOM_HID_Private_FunctionPrototypes USBD_CUSTOM_HID_Private_FunctionPrototypes
  * @brief Private functions declaration.
  * @{
  */

static int8_t CUSTOM_HID_Init_FS(void);
static int8_t CUSTOM_HID_DeInit_FS(void);
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state);

/**
  * @}
  */

USBD_CUSTOM_HID_ItfTypeDef USBD_CustomHID_fops_FS =
{
  CUSTOM_HID_ReportDesc_FS,
  CUSTOM_HID_Init_FS,
  CUSTOM_HID_DeInit_FS,
  CUSTOM_HID_OutEvent_FS
};

/** @defgroup USBD_CUSTOM_HID_Private_Functions USBD_CUSTOM_HID_Private_Functions
  * @brief Private functions.
  * @{
  */

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_Init_FS(void)
{
  /* USER CODE BEGIN 4 */
  return (USBD_OK);
  /* USER CODE END 4 */
}

/**
  * @brief  DeInitializes the CUSTOM HID media low layer
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_DeInit_FS(void)
{
  /* USER CODE BEGIN 5 */
  return (USBD_OK);
  /* USER CODE END 5 */
}

/**
  * @brief  Manage the CUSTOM HID class events
  * @param  event_idx: Event index
  * @param  state: Event state
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */
static int8_t CUSTOM_HID_OutEvent_FS(uint8_t event_idx, uint8_t state)
{
  /* USER CODE BEGIN 6 */
  UNUSED(event_idx);
  UNUSED(state);
	USB_RX_Interrupt();
	USBD_CUSTOM_HID_ReceivePacket(&hUsbDeviceFS);
  return (USBD_OK);
  /* USER CODE END 6 */
}

/* USER CODE BEGIN 7 */
/**
  * @brief  Send the report to the Host
  * @param  report: The report to be sent
  * @param  len: The report length
  * @retval USBD_OK if all operations are OK else USBD_FAIL
  */

int8_t USBD_CUSTOM_HID_SendReport_FS(uint8_t *report, uint16_t len)
{
  return USBD_CUSTOM_HID_SendReport(&hUsbDeviceFS, report, len);
}

void USB_RX_Interrupt(void)
{
	USBD_CUSTOM_HID_HandleTypeDef *usb = (USBD_CUSTOM_HID_HandleTypeDef *) hUsbDeviceFS.pClassData;
	memcpy(&rx_data,usb->Report_buf,sizeof(rx_data));
	
	HAL_GPIO_WritePin(DO0_GPIO_Port, DO0_Pin, rx_data.DO[0]);
	HAL_GPIO_WritePin(DO1_GPIO_Port, DO1_Pin, rx_data.DO[1]);
	HAL_GPIO_WritePin(DO2_GPIO_Port, DO2_Pin, rx_data.DO[2]);
	HAL_GPIO_WritePin(DO3_GPIO_Port, DO3_Pin, rx_data.DO[3]);
	HAL_GPIO_WritePin(DO4_GPIO_Port, DO4_Pin, rx_data.DO[4]);
	HAL_GPIO_WritePin(DO5_GPIO_Port, DO5_Pin, rx_data.DO[5]);
	HAL_GPIO_WritePin(DO6_GPIO_Port, DO6_Pin, rx_data.DO[6]);
	HAL_GPIO_WritePin(DO7_GPIO_Port, DO7_Pin, rx_data.DO[7]);
	
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_1, DAC_ALIGN_12B_R,(uint16_t)(rx_data.DAC_VOLTAGE[0]*4095/3.3));
	HAL_DAC_SetValue(&hdac,DAC_CHANNEL_2, DAC_ALIGN_12B_R, (uint16_t)(rx_data.DAC_VOLTAGE[1]*4095/3.3));
	
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
	TIM_OC_InitTypeDef sConfigOC = {0};
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	htim1.Init.Period = 168000000/rx_data.PWM_FRE[0] - 1;
	sConfigOC.Pulse = (htim1.Init.Period+1)*rx_data.PWM_DUTY[0]/100;
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
	htim2.Init.Period = 84000000/rx_data.PWM_FRE[1]- 1;
  sConfigOC.Pulse = (htim2.Init.Period+1)*rx_data.PWM_DUTY[1]/100;
	HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	
	
	
	//usb->Report_buf[1];
	
}
/* USER CODE END 7 */

/* USER CODE BEGIN PRIVATE_FUNCTIONS_IMPLEMENTATION */

/* USER CODE END PRIVATE_FUNCTIONS_IMPLEMENTATION */
/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

