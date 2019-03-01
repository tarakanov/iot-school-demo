/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file    MKW41Z512xxx4_OT.cpp
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MKW41Z4.h"
#include "fsl_debug_console.h"

#include <FreeRTOS.h>
#include <task.h>


#include <assert.h>
#include <openthread-core-config.h>
#include <openthread/config.h>

#include <openthread/cli.h>
#include <openthread/diag.h>
#include <openthread/openthread.h>
#include <openthread/platform/logging.h>

#include "platform.h"

#include <app_timer.h>
#include <nrf_error.h>
#include <nrf_log.h>
#include <mqttsn_client.h>

#if ENABLE_RTT_CONSOLE
#include "SEGGER_RTT/SEGGER_RTT.h"
#endif
//
// Defines
//
#define SEARCH_GATEWAY_TIMEOUT      5                               /**< MQTT-SN Gateway discovery procedure timeout in [s]. */

#define main_task_PRIORITY          (tskIDLE_PRIORITY + 1)

#undef NRF_LOG_INFO
#undef NRF_LOG_ERROR
#define NRF_LOG_INFO(...)           DbgConsole_Printf(__VA_ARGS__)
#define NRF_LOG_ERROR(...)          DbgConsole_Printf(__VA_ARGS__)

//
// Type definitions
//
typedef enum
{
    MQTTState_Init,
    MQTTState_SearchGW,
    MQTTState_Connect,
    MQTTState_Register,
    MQTTState_Publish,
    MQTTState_Done,
} MQTTState_t;

//
// Function prototypes
//
void mqttsn_evt_handler(mqttsn_client_t * p_client, mqttsn_event_t * p_event);
static void main_task(void *pvParameters);

//
// Static variables
//
static mqttsn_client_t      m_client;                                       /**< An MQTT-SN client instance. */
static mqttsn_remote_t      m_gateway_addr;                                 /**< A gateway address. */
static uint8_t              m_gateway_id;                                   /**< A gateway ID. */
static mqttsn_connect_opt_t m_connect_opt;                                  /**< Connect options for the MQTT-SN client. */
//static uint8_t              m_led_state        = 0;                         /**< Previously sent BSP_LED_2 command. */
static uint16_t             m_msg_id           = 0;                         /**< Message ID thrown with MQTTSN_EVENT_TIMEOUT. */
static char                 m_client_id[]      = "test_mqtt_sn";           /**< The MQTT-SN Client's ID. */
static char                 m_topic_name[]     = "channels/2/messages";      /**< Name of the topic corresponding to subscriber's BSP_LED_2. */
static mqttsn_topic_t       m_topic            =                            /**< Topic corresponding to subscriber's BSP_LED_2. */
{
    .p_topic_name = (unsigned char *)m_topic_name,
    .topic_id     = 0,
};

static MQTTState_t m_mqtt_state = MQTTState_Init;


/***************************************************************************************************
 * @section Thread utils
 **************************************************************************************************/

#if OPENTHREAD_ENABLE_MULTIPLE_INSTANCES
void *otPlatCAlloc(size_t aNum, size_t aSize)
{
    return calloc(aNum, aSize);
}

void otPlatFree(void *aPtr)
{
    free(aPtr);
}
#endif

void otTaskletsSignalPending(otInstance *aInstance)
{
    (void)aInstance;
}

/***************************************************************************************************
 * @section MQTT-SN
 **************************************************************************************************/

/**@brief Initializes MQTT-SN client's connection options.
 */
static void connect_opt_init(void)
{
    m_connect_opt.alive_duration = MQTTSN_DEFAULT_ALIVE_DURATION,
    m_connect_opt.clean_session  = MQTTSN_DEFAULT_CLEAN_SESSION_FLAG,
    m_connect_opt.will_flag      = MQTTSN_DEFAULT_WILL_FLAG,
    m_connect_opt.client_id_len  = strlen(m_client_id),

    memcpy(m_connect_opt.p_client_id, (unsigned char *)m_client_id, m_connect_opt.client_id_len);
}


/**@brief Processes GWINFO message from a gateway.
 *
 * @details This function updates MQTT-SN Gateway information.
 *
 * @param[in]    p_event  Pointer to MQTT-SN event.
 */
static void gateway_info_callback(mqttsn_event_t * p_event)
{
    m_gateway_addr = *(p_event->event_data.connected.p_gateway_addr);
    m_gateway_id   = p_event->event_data.connected.gateway_id;

    m_mqtt_state = MQTTState_Connect;
}


/**@brief Processes CONNACK message from a gateway.
 *
 * @details This function launches the topic registration procedure if necessary.
 */
static void connected_callback(void)
{
    // Switch to register state
    m_mqtt_state = MQTTState_Register;
}


/**@brief Processes DISCONNECT message from a gateway. */
static void disconnected_callback(bool byGateway)
{
    (void)byGateway;

    // Reset state
    m_client.client_state = MQTTSN_CLIENT_DISCONNECTED;
    m_client.evt_handler  = mqttsn_evt_handler;

    m_mqtt_state = MQTTState_SearchGW;
}


/**@brief Processes REGACK message from a gateway.
 *
 * @param[in] p_event Pointer to MQTT-SN event.
 */
static void regack_callback(mqttsn_event_t * p_event)
{
    m_topic.topic_id = p_event->event_data.registered.packet.topic.topic_id;
    NRF_LOG_INFO("MQTT-SN event: Topic has been registered with ID: %d.\r\n",
                 p_event->event_data.registered.packet.topic.topic_id);
    m_mqtt_state = MQTTState_Publish;
}


/**@brief Processes retransmission limit reached event. */
static void timeout_callback(mqttsn_event_t * p_event)
{
    NRF_LOG_INFO("MQTT-SN event: Timed-out message: %d. Message ID: %d.\r\n",
                  p_event->event_data.error.msg_type,
                  p_event->event_data.error.msg_id);
    // try to reconnect in this case
    if (m_mqtt_state > MQTTState_SearchGW)
        m_mqtt_state = MQTTState_Connect;
}


/**@brief Processes results of gateway discovery procedure. */
static void searchgw_timeout_callback(mqttsn_event_t * p_event)
{
    NRF_LOG_INFO("MQTT-SN event: Gateway discovery result: 0x%x.\r\n", p_event->event_data.discovery);
}


/**@brief Function for handling MQTT-SN events. */
void mqttsn_evt_handler(mqttsn_client_t * p_client, mqttsn_event_t * p_event)
{
    switch(p_event->event_id)
    {
        case MQTTSN_EVENT_GATEWAY_FOUND:
            NRF_LOG_INFO("MQTT-SN event: Client has found an active gateway.\r\n");
            gateway_info_callback(p_event);
            break;

        case MQTTSN_EVENT_CONNECTED:
            NRF_LOG_INFO("MQTT-SN event: Client connected.\r\n");
            connected_callback();
            break;

        case MQTTSN_EVENT_DISCONNECTED:
            NRF_LOG_INFO("MQTT-SN event: Client disconnected by gateway.\r\n");
            disconnected_callback(true);
            break;

        case MQTTSN_EVENT_DISCONNECT_PERMIT:
            NRF_LOG_INFO("MQTT-SN event: Client disconnected.\r\n");
            disconnected_callback(false);
            break;

        case MQTTSN_EVENT_REGISTERED:
            NRF_LOG_INFO("MQTT-SN event: Client registered topic.\r\n");
            regack_callback(p_event);
            break;

        case MQTTSN_EVENT_PUBLISHED:
            NRF_LOG_INFO("MQTT-SN event: Client has successfully published content.\r\n");
            m_mqtt_state = MQTTState_Done;
            break;

        case MQTTSN_EVENT_TIMEOUT:
            NRF_LOG_INFO("MQTT-SN event: Retransmission retries limit has been reached.\r\n");
            timeout_callback(p_event);
            break;

        case MQTTSN_EVENT_SEARCHGW_TIMEOUT:
            NRF_LOG_INFO("MQTT-SN event: Gateway discovery procedure has finished.\r\n");
            searchgw_timeout_callback(p_event);
            break;

        default:
            break;
    }
}


/*
 * @brief MQTT-SN Process
 * @param sInstance - pointer to Thread instance
 */
static void MqttProcess(otInstance *sInstance)
{
    if (otThreadGetDeviceRole(sInstance) < OT_DEVICE_ROLE_CHILD )
        return;

    static TickType_t s_ticks = xTaskGetTickCount(); // in milliseconds

    switch (m_mqtt_state)
    {
        case MQTTState_Init:
        {
            DbgConsole_Printf("Init MQTT...");
            app_timer_init();

            mqttsn_client_init(&m_client,
                               MQTTSN_DEFAULT_CLIENT_PORT,
                               mqttsn_evt_handler,
                               sInstance);

            connect_opt_init();

            // switch to the next state
            m_mqtt_state = MQTTState_SearchGW;
            break;
        }
        case MQTTState_SearchGW:
        {
            uint32_t err_code = mqttsn_client_search_gateway(&m_client, SEARCH_GATEWAY_TIMEOUT);
            if (err_code != NRF_SUCCESS)
            {
                if (err_code != NRF_ERROR_BUSY)
                    NRF_LOG_ERROR("SEARCH GATEWAY message could not be sent. Error: 0x%x\r\n", err_code);
            }
            break;
        }

        case MQTTState_Connect:
        {
            uint32_t err_code;

            if (mqttsn_client_state_get(&m_client) == MQTTSN_CLIENT_CONNECTED)
            {
                err_code = mqttsn_client_disconnect(&m_client);
                if (err_code != NRF_SUCCESS)
                {
                    NRF_LOG_ERROR("DISCONNECT message could not be sent. Error: 0x%x\r\n", err_code);
                }
            }
            else
            {
#if 0
                // for testing purposes
                m_gateway_addr.port_number = MQTTSN_DEFAULT_CLIENT_PORT;
                m_gateway_addr.addr[0] = 0xfd;
                m_gateway_addr.addr[1] = 0x11;

                m_gateway_addr.addr[2] = 0x11;
                m_gateway_addr.addr[3] = 0x11;

                m_gateway_addr.addr[4] = 0x11;
                m_gateway_addr.addr[5] = 0x22;

                m_gateway_addr.addr[6] = 0x0;
                m_gateway_addr.addr[7] = 0x0;

                m_gateway_addr.addr[8] = 0xd6;
                m_gateway_addr.addr[9] = 0x3d;

                m_gateway_addr.addr[10] = 0xcb;
                m_gateway_addr.addr[11] = 0x9a;

                m_gateway_addr.addr[12] = 0x58;
                m_gateway_addr.addr[13] = 0xb8;

                m_gateway_addr.addr[14] = 0x6;
                m_gateway_addr.addr[15] = 0x78;
                m_gateway_id = 1;
#endif

                err_code = mqttsn_client_connect(&m_client, &m_gateway_addr, m_gateway_id, &m_connect_opt);
                if (err_code != NRF_SUCCESS)
                {
                    if (err_code != NRF_ERROR_INVALID_STATE)
                        NRF_LOG_ERROR("CONNECT message could not be sent. Error: 0x%x\r\n", err_code);
                }
            }
            break;
        }

        case MQTTState_Register:
        {
            uint32_t err_code = mqttsn_client_topic_register(&m_client,
                                                             m_topic.p_topic_name,
                                                             strlen(m_topic_name),
                                                             &m_msg_id);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("REGISTER message could not be sent. Error code: 0x%x\r\n", err_code);
            }
            break;
        }

        case MQTTState_Publish:
        {
            static const char data[] = "[{'Hard Version':'1','Soft Version':1,'Ticks':%u}]";
            static char message[sizeof(data) + 16];
            snprintf(message, sizeof(message)-1, data, (unsigned int)s_ticks);
            uint32_t err_code = mqttsn_client_publish(&m_client, m_topic.topic_id, (const uint8_t*)message, strlen(message), &m_msg_id);
            if (err_code != NRF_SUCCESS)
            {
                NRF_LOG_ERROR("PUBLISH message could not be sent. Error code: 0x%x\r\n", err_code);
            }
            break;
        }

        case MQTTState_Done:
        {
            if (xTaskGetTickCount() - s_ticks > 10000)
            {
                s_ticks = xTaskGetTickCount();

                // Publish new value
                m_mqtt_state = MQTTState_Publish;
            }
            break;
        }

        default:
            break;
    }
}

/*
 * @brief   Application entry point.
 */
int main(void) {

    xTaskCreate(main_task, "Main", configMINIMAL_STACK_SIZE + 512 + 256, NULL, main_task_PRIORITY, NULL);
    vTaskStartScheduler();
    for (;;)
        ;
    return 0;
}

#include <openthread/include/openthread/platform/uart.h>
#include <openthread/src/core/common/logging.hpp>

#if ENABLE_RTT_CONSOLE
#define DOWN_BUFFER_SIZE 16
#endif
/*!
 * @brief Main task
 */
static void main_task(void *pvParameters)
{
    int argc = 0;
    char *argv[] = {(char*)"", NULL};

#if ENABLE_RTT_CONSOLE
    char input[DOWN_BUFFER_SIZE];
    size_t index = 0;
	memset(input, 0x00, DOWN_BUFFER_SIZE);
#endif
    otInstance *sInstance;

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    //================================


    DbgConsole_Printf("Initializing\r\n");

#if OPENTHREAD_ENABLE_MULTIPLE_INSTANCES
    size_t   otInstanceBufferLength = 0;
    uint8_t *otInstanceBuffer       = NULL;
#endif

pseudo_reset:

    PlatformInit(argc, argv);

    DbgConsole_Printf("Init instance\r\n");
#if OPENTHREAD_ENABLE_MULTIPLE_INSTANCES
    // Call to query the buffer size
    (void)otInstanceInit(NULL, &otInstanceBufferLength);

    // Call to allocate the buffer
    otInstanceBuffer = (uint8_t *)malloc(otInstanceBufferLength);
    assert(otInstanceBuffer);

    // Initialize OpenThread with the buffer
    sInstance = otInstanceInit(otInstanceBuffer, &otInstanceBufferLength);
#else
    sInstance = otInstanceInitSingle();
#endif
    assert(sInstance);
    otLogInfo(sInstance, OT_LOG_REGION_PLATFORM, "InstanceInit");

    DbgConsole_Printf("Init OT uart\r\n");
    otCliUartInit(sInstance);

#if OPENTHREAD_ENABLE_DIAG
    DbgConsole_Printf("Init diag\r\n");
    otDiagInit(sInstance);
#endif

    otIp6SetEnabled(sInstance, true);

    DbgConsole_Printf("Start...\r\n");
    while (!PlatformPseudoResetWasRequested())
    {
        otTaskletsProcess(sInstance);
        PlatformProcessDrivers(sInstance);

        MqttProcess(sInstance);
#if (ENABLE_RTT_CONSOLE)
        if (SEGGER_RTT_HasKey())
        {
        	input[index] = (char)SEGGER_RTT_GetKey();
        	if (input[index] == '\0') continue;
        	if (input[index] == '\n')
        	{
            	otLogInfo(sInstance, OT_LOG_REGION_PLATFORM, "%s", input);

            	otPlatUartReceived((uint8_t*)input, strlen(input));
            	index = 0;
            	memset(input, 0x00, DOWN_BUFFER_SIZE);
        	} else        	index++;
        }
#endif
        taskYIELD();
    }

    DbgConsole_Printf("Finalize...\r\n");
    otInstanceFinalize(sInstance);
#if OPENTHREAD_ENABLE_MULTIPLE_INSTANCES
    free(otInstanceBuffer);
#endif

    goto pseudo_reset;
}

/*
 * Provide, if required an "otPlatLog()" function
 */
#if OPENTHREAD_CONFIG_LOG_OUTPUT == OPENTHREAD_CONFIG_LOG_OUTPUT_APP
void otPlatLog(otLogLevel aLogLevel, otLogRegion aLogRegion, const char *aFormat, ...)
{
    OT_UNUSED_VARIABLE(aLogLevel);
    OT_UNUSED_VARIABLE(aLogRegion);
    OT_UNUSED_VARIABLE(aFormat);

    va_list ap;
    va_start(ap, aFormat);
    otCliPlatLogv(aLogLevel, aLogRegion, aFormat, ap);
    va_end(ap);
}
#endif
