/*
 * mesh_friend.h
 *
 *  Created on: Nov 24, 2018
 *      Author: poorn
 */

#ifndef MESH_FRIEND_H_
#define MESH_FRIEND_H_

#include "src/main.h"
//#include "src/Custom_Timer0.h"

#define Application_Key_Type					1

#define John_Moisture_LPN_Server_Address		0x03
#define Andrew_Lux_Friend_Server_Address		0x02
#define Khalid_Smoke_LPN_Server_Address			0x01

#define Generic_Level_State_Type				2
#define Generic_Level_State_Length				2

#define OOB_Public_Key					0
#define OOB_Output_Auth_Method			(1 << 3)
#define OOB_Output_Numeric				(1 << 3)
#define OOB_Size						2
#define OOB_Ignore						0

/** Timer Frequency used. */
#define TIMER_CLK_FREQ ((uint32)32768)
/** Convert msec to timer ticks. */
#define TIMER_MS_2_TIMERTICK(ms) ((TIMER_CLK_FREQ * ms) / 1000)

#define TIMER_ID_RESTART    	1
#define TIMER_ID_FACTORY_RESET  2
#define TIMER_Button_Debouncing	3
#define TIMER_Provisioning		4
#define TIMER_Regular_Poll		5

#define Prov_Timeout_Seconds	10

#define Door_Open				0x7FFF

#define Alarm_ON				0x7FFF
#define John_Alarm				0
#define Andrew_Alarm			1
#define Khalid_Alarm			2

#define Flash_Key_Address		0x4000
#define Key_Not_Written			0x0502


void generic_level_client_get(uint16_t s_addr);
void generic_level_client_set(uint16_t s_addr);
void Store_Alarm_to_Flash(void);
void Load_Alarm_from_Flash(void);
void Alarm_Update(void);
void set_device_name(bd_addr *pAddr);
void initiate_factory_reset(void);
void friend_node_init(void);


#endif /* MESH_FRIEND_H_ */
