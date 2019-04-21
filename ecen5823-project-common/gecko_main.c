/***************************************************************************//**
 * @file
 * @brief Silicon Labs BT Mesh Empty Example Project
 * This example demonstrates the bare minimum needed for a Blue Gecko BT Mesh C application.
 * The application starts unprovisioned Beaconing after boot
 *******************************************************************************
 * # License
 * <b>Copyright 2018 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include "src/gpio.h"
#include "native_gecko.h"
#include "src/display.h"
#include "src/LETIMER.h"
#include "src/Global_Defines.h"
#include "src/servo.h"
#include "src/adc.h"

uint8_t tmp_state = 0;


const char* ButtonState[] = {"Button Pressed","Button Released"};

// bluetooth stack heap
#define MAX_CONNECTIONS 2

uint8_t bluetooth_stack_heap[DEFAULT_BLUETOOTH_HEAP(MAX_CONNECTIONS) + BTMESH_HEAP_SIZE + 1760];

// Bluetooth advertisement set configuration
//
// At minimum the following is required:
// * One advertisement set for Bluetooth LE stack (handle number 0)
// * One advertisement set for Mesh data (handle number 1)
// * One advertisement set for Mesh unprovisioned beacons (handle number 2)
// * One advertisement set for Mesh unprovisioned URI (handle number 3)
// * N advertisement sets for Mesh GATT service advertisements
// (one for each network key, handle numbers 4 .. N+3)
//
#define MAX_ADVERTISERS (4 + MESH_CFG_MAX_NETKEYS)  //Change ?

static gecko_bluetooth_ll_priorities linklayer_priorities = GECKO_BLUETOOTH_PRIORITIES_DEFAULT;

// bluetooth stack configuration
extern const struct bg_gattdb_def bg_gattdb_data;

// Flag for indicating DFU Reset must be performed
uint8_t boot_to_dfu = 0;

const gecko_configuration_t config =
{
  .bluetooth.max_connections = MAX_CONNECTIONS,
  .bluetooth.max_advertisers = MAX_ADVERTISERS,
  .bluetooth.heap = bluetooth_stack_heap,
  .bluetooth.heap_size = sizeof(bluetooth_stack_heap) - BTMESH_HEAP_SIZE,
  .bluetooth.sleep_clock_accuracy = 100,
  .bluetooth.linklayer_priorities = &linklayer_priorities,
  .gattdb = &bg_gattdb_data,
  .btmesh_heap_size = BTMESH_HEAP_SIZE,
#if (HAL_PA_ENABLE)
  .pa.config_enable = 1, // Set this to be a valid PA config
#if defined(FEATURE_PA_INPUT_FROM_VBAT)
  .pa.input = GECKO_RADIO_PA_INPUT_VBAT, // Configure PA input to VBAT
#else
  .pa.input = GECKO_RADIO_PA_INPUT_DCDC,
#endif // defined(FEATURE_PA_INPUT_FROM_VBAT)
#endif // (HAL_PA_ENABLE)
  .max_timers = 16,
};

/**
 * See light switch app.c file definition
 */
void gecko_bgapi_classes_init_server_friend(void)
{
  gecko_bgapi_class_dfu_init();
  gecko_bgapi_class_system_init();
  gecko_bgapi_class_le_gap_init();
  gecko_bgapi_class_le_connection_init();
  gecko_bgapi_class_gatt_init();        //Additional
  gecko_bgapi_class_gatt_server_init();
//  gecko_bgapi_class_endpoint_init();//not found in native_gecko.h
  gecko_bgapi_class_hardware_init();
  gecko_bgapi_class_flash_init();
  gecko_bgapi_class_test_init();
  //gecko_bgapi_class_sm_init();//for oob
  mesh_native_bgapi_init();         //Additional
  gecko_bgapi_class_mesh_node_init();
  //gecko_bgapi_class_mesh_prov_init();
  gecko_bgapi_class_mesh_proxy_init();
  gecko_bgapi_class_mesh_proxy_server_init();
  //gecko_bgapi_class_mesh_proxy_client_init();
  gecko_bgapi_class_mesh_generic_client_init();
  gecko_bgapi_class_mesh_generic_server_init();
  //gecko_bgapi_class_mesh_vendor_model_init();
  //gecko_bgapi_class_mesh_health_client_init();
  //gecko_bgapi_class_mesh_health_server_init();
  //gecko_bgapi_class_mesh_test_init();
  //gecko_bgapi_class_mesh_lpn_init();
  gecko_bgapi_class_mesh_friend_init();
}


/**
 * See main function list in soc-btmesh-switch project file
 */
void gecko_bgapi_classes_init_client_lpn(void)
{
  gecko_bgapi_class_dfu_init();
  gecko_bgapi_class_system_init();
  gecko_bgapi_class_le_gap_init();
  gecko_bgapi_class_le_connection_init();
  gecko_bgapi_class_gatt_init();//Additional
  gecko_bgapi_class_gatt_server_init();
  //  gecko_bgapi_class_endpoint_init();//not found in native_gecko.h
  gecko_bgapi_class_hardware_init();
  gecko_bgapi_class_flash_init();
  gecko_bgapi_class_test_init();
  //gecko_bgapi_class_sm_init();//for oob
  mesh_native_bgapi_init();     //Additional
  gecko_bgapi_class_mesh_node_init();
  //gecko_bgapi_class_mesh_prov_init();
  gecko_bgapi_class_mesh_proxy_init();
  gecko_bgapi_class_mesh_proxy_server_init();
  //gecko_bgapi_class_mesh_proxy_client_init();
//  gecko_bgapi_class_mesh_generic_client_init();
  gecko_bgapi_class_mesh_generic_server_init();
  //gecko_bgapi_class_mesh_vendor_model_init();
  //gecko_bgapi_class_mesh_health_client_init();
  //gecko_bgapi_class_mesh_health_server_init();
  //gecko_bgapi_class_mesh_test_init();
  gecko_bgapi_class_mesh_lpn_init();
  //gecko_bgapi_class_mesh_friend_init();
}


void initiate_factory_reset(void)
{
  LOG_INFO("Factory Reset:[TIME]%f\n",Logging());
  displayPrintf(DISPLAY_ROW_ACTION,"Factory Reset");

  if (conn_handle != 0xFF) {
    gecko_cmd_le_connection_close(conn_handle);
  }

  gecko_cmd_flash_ps_erase_all();
  gecko_cmd_hardware_set_soft_timer(RESET_TIME, factoryID, 1);
}


void set_device_name(bd_addr *DeviceAddress)
{

//#if DEVICE_IS_ONOFF_PUBLISHER
  // sprintf(StringToDisplay, "Publisher %x:%x", DeviceAddress->addr[1], DeviceAddress->addr[0]);
  // displayPrintf(DISPLAY_ROW_NAME,"Publisher");
  // displayPrintf(DISPLAY_ROW_BTADDR2,"5823Pub%02x:%02x",DeviceAddress->addr[1],DeviceAddress->addr[0]);

//#else
 sprintf(StringToDisplay, "Subscriber: %x:%x", DeviceAddress->addr[1], DeviceAddress->addr[0]);
 displayPrintf(DISPLAY_ROW_NAME,"Subscriber");
 displayPrintf(DISPLAY_ROW_BTADDR2,"5823Sub%02x:%02x",DeviceAddress->addr[1],DeviceAddress->addr[0]);
////#endif
 resp = gecko_cmd_gatt_server_write_attribute_value(gattdb_device_name, 0, strlen(StringToDisplay), (uint8 *)StringToDisplay)->result;
 if (resp) {
    LOG_INFO("Write attribute failed,[ERROR]%x:[TIME]%f\n",resp,Logging());
 }
 LOG_INFO("Device: '%s':[TIME]%f\n", StringToDisplay,Logging());
}

////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////

void lpn_init(void)
{
    struct gecko_msg_mesh_lpn_init_rsp_t *pLPNInit = gecko_cmd_mesh_lpn_init();//friend_node_init();
    if(pLPNInit->result)
    LOG_INFO("Error Init LPN node\n");
    else
    LOG_INFO("Init LPN node Success\n");

    struct gecko_msg_mesh_lpn_configure_rsp_t *pLPNConfig = gecko_cmd_mesh_lpn_configure(2,5000);
    if(pLPNConfig->result)
    LOG_INFO("Error configuring LPN node\n");
    else
    LOG_INFO("configuring LPN node Success\n");
    struct gecko_msg_mesh_lpn_establish_friendship_rsp_t *pLPNest=gecko_cmd_mesh_lpn_establish_friendship(0);
    if(pLPNest->result)
    LOG_INFO("Finding friend command error\n");
    else
    LOG_INFO(" LPN node Success\n");
}

void gecko_main_init()
{

  initMcu();

  initBoard();

  initApp();

  initDefines();

  EnableGPIOInterrupts();

  initADC();

  //ServoInit();	//NEW

  linklayer_priorities.scan_max = linklayer_priorities.adv_min + 1;

  gecko_stack_init(&config);

//  if( DeviceUsesClientModel() ) {
//    gecko_bgapi_classes_init_client_lpn();
//  } else {
//    gecko_bgapi_classes_init_server_friend();
//  }

//  gecko_bgapi_classes_init_server_friend();
  gecko_bgapi_classes_init_client_lpn();
  gecko_initCoexHAL();


}

void handle_gecko_event(uint32_t evt_id, struct gecko_cmd_packet *evt)
{
  //Additional
    struct gecko_msg_mesh_node_provisioning_failed_evt_t  *prov_fail_evt;
//
//    if (NULL == evt) {
//    return;
//    }


  switch (evt_id) {
    case gecko_evt_system_boot_id:
      if(!(GPIO_PinInGet(PortB1,PinB1))|| !(GPIO_PinInGet(PortB0,PinB0)))
    {
      initiate_factory_reset();
    }
       else
       {
      LOG_INFO("System Boot ID\n");
      struct gecko_msg_system_get_bt_address_rsp_t *Device_Addr_Struct = gecko_cmd_system_get_bt_address();
      set_device_name(&Device_Addr_Struct->address);
      resp = gecko_cmd_mesh_node_init()->result;
      if(resp != 0)
      LOG_INFO("Error: 0x%x:[TIME]%f\n", resp,Logging());
       }
      break;

/////////////////////////////////////////////////////////////////////////////////
  case gecko_evt_hardware_soft_timer_id:

        switch(evt->data.evt_hardware_soft_timer.handle)
        {
          case factoryID:
            gecko_cmd_system_reset(0);
            break;

          case debounceID:
            NVIC_EnableIRQ(GPIO_ODD_IRQn);
            NVIC_EnableIRQ(GPIO_EVEN_IRQn);
            break;

          case restartID:
            gecko_cmd_system_reset(0);
            break;

          case  provisionID:
            LOG_INFO("\n\r________________OPERATION-START________________\n:[TIME]%f\n\r",Logging());
//#if DEVICE_IS_ONOFF_PUBLISHER
//            NVIC_EnableIRQ(GPIO_ODD_IRQn);
//            NVIC_EnableIRQ(GPIO_EVEN_IRQn);
//#endif
            break;
          case FriendNotFoundID:
            LOG_INFO("retrying\n");
            lpn_init();
            break;

          default:
            break;
        }
        break;



/////////////////////////////////////////////////////////////////
case gecko_evt_mesh_node_initialized_id:
        LOG_INFO("node initialized:[TIME]%f\n",Logging());
        resp = gecko_cmd_mesh_generic_server_init()->result;
    if(resp)
    {
      LOG_INFO("Client Initialization Failed:[TIME]%f\n",Logging());
      LOG_INFO("[ERROR]CODE: %x:[TIME]%f\n", resp,Logging());
    }

        struct gecko_msg_mesh_node_initialized_evt_t *PtrData = (struct gecko_msg_mesh_node_initialized_evt_t *)&(evt->data);
        if(PtrData->provisioned)
        {
          LOG_INFO("Node Provisioned with Address:%x, ivi:%ld:[TIME]%f\n", PtrData->address, PtrData->ivi,Logging());
          displayPrintf(DISPLAY_ROW_CONNECTION,"Provisioned");
          primary_address = PtrData->address;
          ElementID = 0;
          mesh_lib_init(malloc, free, 8);
         lpn_init();
        }


        else
        {
          LOG_INFO("Node Unprovisioned:[TIME]%f\n",Logging());
          LOG_INFO("Unprovisioned Beaconing Started...:[TIME]%f\n",Logging());
          gecko_cmd_mesh_node_start_unprov_beaconing(0x3);   // (3)enable ADV and GATT provisioning bearer
        }
        break;

////////////////////////////////////////////////////////////////////////////

    case gecko_evt_mesh_node_provisioning_started_id:
        LOG_INFO("Provisioning Started:[TIME]%f\n",Logging());
        displayPrintf(DISPLAY_ROW_CONNECTION,"Provisioning");
        break;

case gecko_evt_mesh_friend_friendship_established_id:
    LOG_INFO("FRIENDSHIP ESTABLISHED\n");
    break;

case gecko_evt_mesh_lpn_friendship_failed_id:
      LOG_INFO("FRIENDSHIP FAILED\n");
      //struct gecko_msg_mesh_lpn_friendship_failed_evt_t
      gecko_cmd_hardware_set_soft_timer(RESET_TIME,FriendNotFoundID,1);
  break;

case gecko_evt_mesh_lpn_friendship_established_id:
  LOG_INFO("FRIENDSHIP ESTABLISHED-LPN\n");
  displayPrintf(DISPLAY_ROW_CONNECTION,"LowPower-Node2");
  break;

    case gecko_evt_mesh_node_provisioned_id:

        ElementID = 0;
//#if DEVICE_IS_ONOFF_PUBLISHER
//        mesh_lib_init(malloc, free, 8);
//        displayPrintf(DISPLAY_ROW_ACTION,"%s",ButtonState[ButtonToggle]);

        mesh_lib_init(malloc, free, 8);
        LOG_INFO("Node provisioned with Address:%x:[TIME]%f\n", evt->data.evt_mesh_node_provisioned.address,Logging());
        displayPrintf(DISPLAY_ROW_CONNECTION,"Provisioned");
        lpn_init();
        break;


//////////////////////////////////////////////////////////////
    case gecko_evt_mesh_node_provisioning_failed_id:

        prov_fail_evt = (struct gecko_msg_mesh_node_provisioning_failed_evt_t  *)&(evt->data);
        LOG_INFO("provisioning failed, code %x:[TIME]%f\n", prov_fail_evt->result,Logging());
        displayPrintf(DISPLAY_ROW_CONNECTION,"Provision Fail");
        gecko_cmd_hardware_set_soft_timer(RESET_TIME, restartID, 1);
        break;


//    case gecko_evt_mesh_node_key_added_id:
//
//        LOG_INFO("New Key");
//        if(evt->data.evt_mesh_node_key_added.type)
//          LOG_INFO("Application");
//        else
//          LOG_INFO("Network");
//        LOG_INFO("is %f\r\n",evt->data.evt_mesh_node_key_added.index,Logging());
//
//        if(evt->data.evt_mesh_node_key_added.type == Application_Key_Type)
//          appkeyindex = evt->data.evt_mesh_node_key_added.index;
//        break;

///////////////////////////////////////////////////////////////////////////////////////////////

//    case gecko_evt_mesh_node_model_config_changed_id:
//    {
//        LOG_INFO("Model Configuration Changed:[TIME]%f\n",Logging());
//        gecko_cmd_hardware_set_soft_timer((32768 * ProvisionTimeout), provisionID, 1);
//        break;
//    }
//////////////////////////////////////////////////////////////////////////////////////////////
  case gecko_evt_le_connection_opened_id:
      LOG_INFO("Connection_Opened_ID:[TIME]%f\n",Logging());
      NumOfConn++;
      conn_handle = evt->data.evt_le_connection_opened.connection;
      break;
//////////////////////////////////////////////////////////////////////////////////////////////
  case gecko_evt_le_connection_parameters_id: //Usual
      LOG_INFO("Connection_Parameters_ID:[TIME]%f\n",Logging());
      break;
//////////////////////////////////////////////////////////////////////////////////////////////
  case gecko_evt_mesh_generic_server_client_request_id:
  {
	  //	  LOG_INFO("");
	        uint8_t  Variables = 0;
	        Variables = evt->data.evt_mesh_generic_server_client_request.parameters.data[0];
	  	  if(Variables == 0)
	  	  {
				displayPrintf(DISPLAY_ROW_ACTION, "Override");
				LOG_INFO("Server_Client_Request_ID:[TIME]%f\n",Logging());
				LOG_INFO("Override Detected\n");
				CORE_DECLARE_IRQ_STATE;
				CORE_ENTER_CRITICAL();
					//UnlockDoor();	//NEW
					//Buzzer Off();
					//millivolts = 2500;
				CORE_EXIT_CRITICAL();

	  	  }
	      break;
  }
/////////////////////////////////////////////////////////////////////////////////////////////
//  case gecko_evt_mesh_generic_server_state_changed_id:
//        mesh_lib_generic_server_event_handler(evt);
//        break;

  case gecko_evt_le_connection_closed_id:
      if (boot_to_dfu)
        gecko_cmd_system_reset(2);
      break;

//  case gecko_evt_gatt_server_user_write_request_id:
//      if (evt->data.evt_gatt_server_user_write_request.characteristic == gattdb_ota_control) {
//        boot_to_dfu = 1;
//        gecko_cmd_gatt_server_send_user_write_response(
//          evt->data.evt_gatt_server_user_write_request.connection,
//          gattdb_ota_control,
//          bg_err_success);
//        gecko_cmd_le_connection_close(evt->data.evt_gatt_server_user_write_request.connection);
//      }
//      break;
//////////////////////////////////////////////////////////////////////////////
  case gecko_evt_system_external_signal_id:
      DetectEvent = evt->data.evt_system_external_signal.extsignals;
       if(DetectEvent & update_display)
      {
        CORE_DECLARE_IRQ_STATE;
        CORE_ENTER_CRITICAL();
        mask &= ~update_display;        //Clear the Event Mask
        CORE_EXIT_CRITICAL();
        displayUpdate();
        if(clear_count%5==0)
		{
			clearDisplay();
			clear_count=0;
			ADCSample();
			LOG_INFO("Val:%d",millivolts);
//
		}
		else
			clear_count+=1;

      //  LOG_INFO("Updated Display:[TIME]%f\n",Logging());

      }

      if(DetectEvent & button_event)
      {
      CORE_DECLARE_IRQ_STATE;
      CORE_ENTER_CRITICAL();
        mask &= ~button_event;
      CORE_EXIT_CRITICAL();
        LOG_INFO("%sPublishing:[TIME]%f\n",ButtonState[ButtonToggle],Logging());  //display printf state.
      displayPrintf(DISPLAY_ROW_TEMPVALUE,"%s",ButtonState[ButtonToggle]);
      transaction_id+=1;
      //generic_level_update_publish(ButtonToggle);

	struct mesh_generic_state current;
	errorcode_t response_update, response_publish;

	current.kind = mesh_generic_state_on_off;
	current.on_off.on = 1;//ButtonToggle;

//	transaction_id += 1;
//	struct mesh_generic_request custom;	//mesh_generic_state
//	custom.kind = mesh_generic_request_on_off;
//	custom.on_off = ButtonToggle;

	response_update = mesh_lib_generic_server_update(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,0,&current,NULL,0);

     if (response_update)
     {
    	 LOG_INFO("gecko_cmd_mesh_generic_client_publish failed,code %d:TimeStamp=>%f\n", response_update,Logging());

     }
	  else
	  {
		 LOG_INFO("Server Updated:TimeStamp=>%f\n",Logging());
		 response_publish=mesh_lib_generic_server_publish(MESH_GENERIC_ON_OFF_SERVER_MODEL_ID,
				 0, mesh_generic_state_on_off);
		 if(response_publish)
			 LOG_INFO("Not pubished,code:%x,Time:%f\n",response_publish,Logging());
		 else
			 LOG_INFO("Server Published:%f\n",Logging());
      }

    }

    break;

  default:
      break;
  }
}

void initDefines(void)
{
    ElementID = 0xffff;
    appkeyindex = 0;
    transaction_id = 0;
    conn_handle = 0xFF;      /* handle of the last opened LE connection */

    StringToDisplay[20];

    primary_address = 0;    /* Address of the Primary Element of the Node */
    NumOfConn = 0;     		/* number of active Bluetooth connections */
}

void EnableGPIOInterrupts(void)
{
    NVIC_EnableIRQ(GPIO_EVEN_IRQn);
    NVIC_EnableIRQ(GPIO_ODD_IRQn);
}
