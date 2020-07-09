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
struct rx {
	char header[16];// = {'T','R','I','E','N','L','O','N','G','T','I','N','H','0','0','0'};
	uint16_t DO;
	float dac1;
	float dac2;
	float dac3;
	float dac4;
	uint16_t pwm1_fre;
	uint16_t pwm1_duty;
	uint16_t pwm2_fre;
	uint16_t pwm2_duty;
	uint16_t byte_number;
	uint16_t crc;
};
extern struct rx rx_data;
struct tx {
	char header[16];//  = {'T','R','I','E','N','L','O','N','G','T','I','N','H','0','0','0'}
	uint16_t DI;
	uint16_t adc1;
	uint16_t adc2;
	uint16_t adc3;
	uint16_t adc4;
	uint16_t byte_number;
	uint16_t crc;
};
extern struct tx tx_data;

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
  USB_RX_Interrupt();
  return 0;
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

