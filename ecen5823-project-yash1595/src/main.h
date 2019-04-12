#ifndef		__main_h__
#define 	__main_h__

//***********************************************************************************
// Include files
//***********************************************************************************

/* C Standard Library headers */
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

/* Board headers */
#include "init_mcu.h"
#include "init_board.h"
#include "init_app.h"
#include "ble-configuration.h"
#include "board_features.h"
#include "retargetserial.h"

/* Bluetooth stack headers */
#include "bg_types.h"
#include "native_gecko.h"
#include "gatt_db.h"
#include <gecko_configuration.h>
//#include "mesh_generic_model_capi_types.h"
//#include "mesh_lib.h"
//#include <mesh_sizes.h>


/* Libraries containing default Gecko configuration values */
#include "em_emu.h"
#include "em_cmu.h"
#include "em_core.h"

#include "em_usart.h"

/* Device initialization header */
#include "hal-config.h"


/* Display Interface header */
//#include "lcd_driver.h"

#endif

//***********************************************************************************
// defined files
//***********************************************************************************


//#define Event_Si7021_Temperature_Read_Complete	3

//***********************************************************************************
// global variables
//***********************************************************************************

extern uint16_t Element_Index, Appkey_Index, Response;
extern uint8_t Trans_ID, conn_handle;

extern uint16_t New_Threshold, Old_Threshold, Publisher_Data, Publisher_Address;
extern uint8_t Alarm_Status, Old_Alarm_Status;

extern uint32_t Interrupt_Read;
extern uint8_t FP_Button;
extern char lcd_string[20];


//***********************************************************************************
// function prototypes
//***********************************************************************************
