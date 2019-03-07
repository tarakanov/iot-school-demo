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
#include "fsl_adc16.h"
#include "fsl_debug_console.h"
#include <FreeRTOS.h>
#include <task.h>
#include <assert.h>
#include <openthread-core-config.h>
#include <openthread/config.h>
#include <openthread/cli.h>
#include <openthread/diag.h>
#include <openthread/tasklet.h>
#include <openthread/thread.h>
#include <openthread/platform/logging.h>
#include "platform.h"
#include <app_timer.h>
#include <nrf_error.h>
#include <nrf_log.h>
#include <mqttsn_client.h>
#if ENABLE_RTT_CONSOLE
#include "SEGGER_RTT/SEGGER_RTT.h"
#endif
#include <openthread/include/openthread/coap.h>
#include <openthread/include/openthread/error.h>
#include <openthread/src/core/common/code_utils.hpp>
#include <openthread/src/core/common/logging.hpp>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "fsl_adc16.h"
#include <openthread/include/openthread/platform/uart.h>
#include <openthread/src/core/common/logging.hpp>

//
// Defines
//

#define main_task_PRIORITY          (tskIDLE_PRIORITY + 1)
#define BOARD_LED_GPIO BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_GPIO_PIN
#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 4U /* PTB18, ADC0_SE4 */
#if ENABLE_RTT_CONSOLE
#define DOWN_BUFFER_SIZE 100
#endif

//
// Type definitions
//
typedef enum
{
	appMode_Init,
	appMode_Manual,
	appMode_Single,
	appMode_Multiple
} appMode_t;

typedef enum
{
	light_goes_down,
	light_no_change,
	light_goes_up
} lightTrigger_t;

//
// Function prototypes
//
static void main_task(void *pvParameters);

void coap_handler_test ( void * aContext,  otMessage * aMessage, const otMessageInfo *aMessageInfo);
void coap_handler_led_on  ( void * aContext,  otMessage * aMessage, const otMessageInfo *aMessageInfo);
void coap_handler_lux  ( void * aContext,  otMessage * aMessage, const otMessageInfo *aMessageInfo);
void coapAppInit();
void getLightLevel();
static void coapAppProcess(otInstance *sInstance);

//
//  Variables

static appMode_t app_mode = appMode_Init;
static otInstance *sInstance;
adc16_config_t adc16ConfigStruct;
adc16_channel_config_t adc16ChannelConfigStruct;
otCoapResource cr_1; // coap resource 1
otCoapResource cr_led; // coap resource for led
otCoapResource cr_lux; // coap resource for lux
otCoapResource cr_mode; // coap resource for mode
otError error = OT_ERROR_NONE;
uint32_t 	light_level_adc; // current light level in adc readings
uint32_t	light_index = 0; // index of a device
otIp6Address allCoapAddr;
const char *allCoapAddrStr = "FF0A::FD";
otError      error1 = OT_ERROR_NONE;
uint32_t light_lvl_trigger = 500;
uint32_t light_lvl_gyst = 200;
uint32_t light_lvl_last;
lightTrigger_t light_trigger = light_no_change;
TickType_t  ticks, prev_ticks;

static void coapAppProcess(otInstance *sInstance)
{
	getLightLevel();
	switch(app_mode)
	{
	case appMode_Init:
	{
		ticks = xTaskGetTickCount();
		prev_ticks = xTaskGetTickCount();
		break;
	}
	case appMode_Manual:
	{
		break;
	}
	case appMode_Single:
	{
		ticks = xTaskGetTickCount();
		if (ticks - prev_ticks > pdMS_TO_TICKS(3000)) // every 3000 ms
		{
			otLogInfoPlat("App action\r\n");
			prev_ticks = xTaskGetTickCount();
			switch (light_trigger)
			{
			case light_goes_up:
			{
				GPIO_ClearPinsOutput(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
				break;
			}
			case light_goes_down:
			{
				GPIO_SetPinsOutput(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
				break;
			}
			default:
				break;
			}
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

  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
    BOARD_InitDebugConsole();
    gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput, 0,
    };
    GPIO_PinInit(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN, &led_config);
    ADC16_GetDefaultConfig(&adc16ConfigStruct);
    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */
    adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;

pseudo_reset:

    PlatformInit(argc, argv);
    otLogInfoPlat("Init instance\r\n");
    sInstance = otInstanceInitSingle();
    assert(sInstance);
    otCliUartInit(sInstance);
    otIp6SetEnabled(sInstance, true);

    coapAppInit(); // init COAP resources
    error1 = otIp6AddressFromString(allCoapAddrStr, &allCoapAddr);
    error1= otIp6SubscribeMulticastAddress(sInstance, &allCoapAddr);

    while (!PlatformPseudoResetWasRequested())
    {
        otTaskletsProcess(sInstance);
        PlatformProcessDrivers(sInstance);
        coapAppProcess(sInstance);

#if (ENABLE_RTT_CONSOLE)
        if (SEGGER_RTT_HasKey())
        {
        	input[index] = (char)SEGGER_RTT_GetKey();
        	if (input[index] == '\0') continue;
        	if (input[index] == '\n')
        	{
        		otLogInfoPlat( "%s", input);
            	otPlatUartReceived((uint8_t*)input, strlen(input));
            	index = 0;
            	memset(input, 0x00, DOWN_BUFFER_SIZE);
        	} else        	index++;
        }
#endif
        taskYIELD();
    }
    otInstanceFinalize(sInstance);
    goto pseudo_reset;
}

void coap_handler_test ( void * aContext, otMessage * aMessage, const otMessageInfo *aMessageInfo)
{
    otError      error = OT_ERROR_NONE;
    otMessage *  responseMessage;
    otCoapCode   responseCode    = OT_COAP_CODE_EMPTY;
    char         responseContent[] = "hello\r\n";

    if (otCoapMessageGetType(aMessage) == OT_COAP_TYPE_CONFIRMABLE ||
        otCoapMessageGetCode(aMessage) == OT_COAP_CODE_GET)
    {
        if (otCoapMessageGetCode(aMessage) == OT_COAP_CODE_GET)
        {
            responseCode = OT_COAP_CODE_CONTENT;
        }
        else
        {
            responseCode = OT_COAP_CODE_VALID;
        }

        responseMessage = otCoapNewMessage(sInstance, NULL);
        VerifyOrExit(responseMessage != NULL, error = OT_ERROR_NO_BUFS);

        otCoapMessageInit(responseMessage, OT_COAP_TYPE_ACKNOWLEDGMENT, responseCode);
        otCoapMessageSetMessageId(responseMessage, otCoapMessageGetMessageId(aMessage));
        otCoapMessageSetToken(responseMessage, otCoapMessageGetToken(aMessage), otCoapMessageGetTokenLength(aMessage));

        if (otCoapMessageGetCode(aMessage) == OT_COAP_CODE_GET)
        {
            otCoapMessageSetPayloadMarker(responseMessage);
            SuccessOrExit(error = otMessageAppend(responseMessage, &responseContent, sizeof(responseContent)));
        }

        SuccessOrExit(error = otCoapSendResponse(sInstance, responseMessage, aMessageInfo));
    }

exit:

    if (error != OT_ERROR_NONE)
    {
        if (responseMessage != NULL)
        {
        	otLogInfoPlat("coap send response error %d: %s\r\n", error,
                                               otThreadErrorToString(error));
            otMessageFree(responseMessage);
        }
    }
    else if (responseCode >= OT_COAP_CODE_RESPONSE_MIN)
    {
    	otLogInfoPlat("coap response sent successfully!\r\n");
    }
}

void led_process_query(const char *option_value, uint16_t option_length)
{
	otLogInfoPlat("Processing query %.*s\r\n", option_length, option_value);

	if (strncmp(option_value, "toggle", option_length) == 0)
	{
		otLogInfoPlat("toggle action");
        GPIO_TogglePinsOutput(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
	};
	if (strncmp(option_value, "on", option_length) == 0)
	{
		otLogInfoPlat("on action");
        GPIO_SetPinsOutput(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
	};
	if (strncmp(option_value, "off", option_length) == 0)
	{
		otLogInfoPlat("off action");
        GPIO_ClearPinsOutput(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);
	};
}

void coap_handler_led  ( void * aContext, otMessage * aMessage, const otMessageInfo *aMessageInfo)
{
    otError      error = OT_ERROR_NONE;
    otMessage *  responseMessage;
    otCoapCode   responseCode    = OT_COAP_CODE_EMPTY;
    char         responseContent[14];
    uint32_t     led_value;

	const otCoapOption *option;
	char option_value[100];

	option = otCoapMessageGetFirstOption(aMessage);
	while (error == OT_ERROR_NONE)
	{
		error = otCoapMessageGetOptionValue(aMessage, &option_value);
		if (option->mNumber == OT_COAP_OPTION_URI_QUERY) led_process_query(option_value, option->mLength);
	    option = otCoapMessageGetNextOption(aMessage);
	    if (option->mLength > 255) break;
	}

    if (otCoapMessageGetType(aMessage) == OT_COAP_TYPE_CONFIRMABLE ||
        otCoapMessageGetCode(aMessage) == OT_COAP_CODE_GET)
    {
        if (otCoapMessageGetCode(aMessage) == OT_COAP_CODE_GET)
        {
            responseCode = OT_COAP_CODE_CONTENT;
        }
        else
        {
            responseCode = OT_COAP_CODE_VALID;
        }

        responseMessage = otCoapNewMessage(sInstance, NULL);
        VerifyOrExit(responseMessage != NULL, error = OT_ERROR_NO_BUFS);

        otCoapMessageInit(responseMessage, OT_COAP_TYPE_ACKNOWLEDGMENT, responseCode);
        otCoapMessageSetMessageId(responseMessage, otCoapMessageGetMessageId(aMessage));
        otCoapMessageSetToken(responseMessage, otCoapMessageGetToken(aMessage), otCoapMessageGetTokenLength(aMessage));

        if (otCoapMessageGetCode(aMessage) == OT_COAP_CODE_GET)
        {
            otCoapMessageSetPayloadMarker(responseMessage);
            led_value = GPIO_ReadPinInput(BOARD_LED_GPIO, BOARD_LED_GPIO_PIN);
            sprintf(responseContent, "led now is %s", led_value ? " on": "off");
            SuccessOrExit(error = otMessageAppend(responseMessage, &responseContent, sizeof(responseContent)));
        }

        SuccessOrExit(error = otCoapSendResponse(sInstance, responseMessage, aMessageInfo));
    }

exit:

    if (error != OT_ERROR_NONE)
    {
        if (responseMessage != NULL)
        {
        	otLogInfoPlat("coap send response error %d: %s\r\n", error,
                                               otThreadErrorToString(error));
            otMessageFree(responseMessage);
        }
    }
    else if (responseCode >= OT_COAP_CODE_RESPONSE_MIN)
    {
    	otLogInfoPlat("coap response sent successfully!\r\n");
    }
}

void lux_process_query(const char *option_value, uint16_t option_length, char *responseContent)
{
	char query[option_length+1];
	strncpy(query, option_value, option_length);
	query[option_length] = '\0';
	otLogInfoPlat("Processing query %s\r\n", query);

//	char *key;
//	char *value;
//	char *st = query;
//
//	key = strsep(&st, "=");
//	value = st;


	if (strncmp(query, "lvl=", 4) == 0)
	{
		uint32_t trigger_lvl = strtol(&query[4], NULL, 10);
		if (trigger_lvl != 0) {
			light_lvl_trigger = trigger_lvl;
			otLogInfoPlat("set trigger level to %lu", light_lvl_trigger);
		    sprintf(responseContent, "lvl: %lu", light_lvl_trigger);
		}
	};
	if (strncmp(query, "dz=", 3) == 0)
	{
		otLogInfoPlat("get deadzone");
		uint32_t deadzone_val = strtol(&query[3], NULL, 10);
		if (deadzone_val != 0) {
			light_lvl_gyst = deadzone_val;
			otLogInfoPlat("set deadzone value to %lu", light_lvl_gyst);
		    sprintf(responseContent, "dz: %lu", light_lvl_gyst);
		}
	};

	if (strncmp(query, "raw", option_length) == 0)
	{
	    otLogInfoPlat("LUX ADC  Value: %d\r\n", light_level_adc);
	    sprintf(responseContent, "adc: %lu", light_level_adc);
	};

	if (strncmp(query, "lvl", option_length) == 0)
	{
	    otLogInfoPlat("LUX trigger  Value: %d\r\n", light_lvl_trigger);
	    sprintf(responseContent, "lvl: %lu", light_lvl_trigger);
	};

	if (strncmp(query, "dz", option_length) == 0)
	{
	    otLogInfoPlat("LUX Deadzone  Value: %d\r\n", light_lvl_gyst);
	    sprintf(responseContent, "dz: %lu", light_lvl_gyst);
	};

}

void coap_handler_lux  ( void * aContext, otMessage * aMessage, const otMessageInfo *aMessageInfo)
{
    otError      error = OT_ERROR_NONE;
    otMessage *  responseMessage;
    otCoapCode   responseCode    = OT_COAP_CODE_EMPTY;
    char         responseContent[30] = "0";

    //parse options

	const otCoapOption *option;
	char option_value[100];


	option = otCoapMessageGetFirstOption(aMessage);
	while (error == OT_ERROR_NONE)
	{
		error = otCoapMessageGetOptionValue(aMessage, &option_value);
		 if (option->mNumber == OT_COAP_OPTION_URI_QUERY)
		{
			otLogInfoPlat("URI query is %.*s\r\n", option->mLength, &option_value);
			lux_process_query(option_value, option->mLength, responseContent);
		}
	    option = otCoapMessageGetNextOption(aMessage);
	    if (option->mLength > 255) break;
	}

    if (otCoapMessageGetType(aMessage) == OT_COAP_TYPE_CONFIRMABLE ||
        otCoapMessageGetCode(aMessage) == OT_COAP_CODE_GET)
    {
        if (otCoapMessageGetCode(aMessage) == OT_COAP_CODE_GET)
        {
            responseCode = OT_COAP_CODE_CONTENT;
        }
        else
        {
            responseCode = OT_COAP_CODE_VALID;
        }

        responseMessage = otCoapNewMessage(sInstance, NULL);
        VerifyOrExit(responseMessage != NULL, error = OT_ERROR_NO_BUFS);

        otCoapMessageInit(responseMessage, OT_COAP_TYPE_ACKNOWLEDGMENT, responseCode);
        otCoapMessageSetMessageId(responseMessage, otCoapMessageGetMessageId(aMessage));
        otCoapMessageSetToken(responseMessage, otCoapMessageGetToken(aMessage), otCoapMessageGetTokenLength(aMessage));

        if (otCoapMessageGetCode(aMessage) == OT_COAP_CODE_GET)
        {
            otCoapMessageSetPayloadMarker(responseMessage);
            SuccessOrExit(error = otMessageAppend(responseMessage, &responseContent, sizeof(responseContent)));
        }

        SuccessOrExit(error = otCoapSendResponse(sInstance, responseMessage, aMessageInfo));
    }

exit:

    if (error != OT_ERROR_NONE)
    {
        if (responseMessage != NULL)
        {
        	otLogInfoPlat("coap send response error %d: %s\r\n", error,
                                               otThreadErrorToString(error));
            otMessageFree(responseMessage);
        }
    }
    else if (responseCode >= OT_COAP_CODE_RESPONSE_MIN)
    {
    	otLogInfoPlat("coap response sent successfully!\r\n");
    }
}

void mode_process_query(const char *option_value, uint16_t option_length, char *responseContent)
{
	char query[option_length+1];
	strncpy(query, option_value, option_length);
	query[option_length] = '\0';
	otLogInfoPlat("Processing query %s\r\n", query);

	if (strncmp(query, "mode", option_length) == 0)
	{
		switch(app_mode)
		{
		case appMode_Manual:
		{
		    sprintf(responseContent, "mode: off");
			break;
		}
		case appMode_Single:
		{
		    sprintf(responseContent, "mode: single");
			break;
		}
		case appMode_Multiple:
		{
		    sprintf(responseContent, "mode: multi");
			break;
		}
		default:
			break;
		}
	};

	if (strncmp(query, "index", option_length) == 0)
	{
	    sprintf(responseContent, "index: %lu", light_index);
	};


	if (strncmp(query, "index=", 6) == 0)
	{
		uint32_t number = strtol(&query[6], NULL, 10);
		if (number != 0) {
			light_index = number;
			otLogInfoPlat("set device index to %lu", light_index);
		    sprintf(responseContent, "index: %lu", light_index);
		}
	};

	if (strncmp(query, "mode=", 5) == 0)
	{
		char mode_str[sizeof(query) - sizeof("mode=")+1];
		strcpy(mode_str, &query[5]);
		mode_str[sizeof(query) - sizeof("mode=")] = '\0';
		otLogInfoPlat("mode_str %s\r\n", mode_str);
		if (strcmp(mode_str, "off") == 0)
		{
			app_mode = appMode_Manual;
			otLogInfoPlat("set device mode to off");
			sprintf(responseContent, "mode: off");
		}
		else if (strcmp(mode_str, "single") == 0)
		{
			app_mode = appMode_Single;
			otLogInfoPlat("set device mode to single");
			sprintf(responseContent, "mode: single");
		}
		else if (strcmp(mode_str, "multi") == 0)
		{
			app_mode = appMode_Multiple;
			otLogInfoPlat("set device mode to multi");
			sprintf(responseContent, "mode: multi");
		}
	}
}

void coap_handler_mode  ( void * aContext, otMessage * aMessage, const otMessageInfo *aMessageInfo)
{
    otError      error = OT_ERROR_NONE;
    otMessage *  responseMessage;
    otCoapCode   responseCode    = OT_COAP_CODE_EMPTY;
    char         responseContent[30] = "0";

    //parse options

	const otCoapOption *option;
	char option_value[100];


	option = otCoapMessageGetFirstOption(aMessage);
	while (error == OT_ERROR_NONE)
	{
		error = otCoapMessageGetOptionValue(aMessage, &option_value);
		if (option->mNumber == OT_COAP_OPTION_URI_QUERY)
		{
			otLogInfoPlat("URI query is %.*s\r\n", option->mLength, &option_value);
			mode_process_query(option_value, option->mLength, responseContent);
			break;
		}
	    option = otCoapMessageGetNextOption(aMessage);
	    if (option->mLength > 255) break;
	}

    if (otCoapMessageGetType(aMessage) == OT_COAP_TYPE_CONFIRMABLE ||
        otCoapMessageGetCode(aMessage) == OT_COAP_CODE_GET)
    {
        if (otCoapMessageGetCode(aMessage) == OT_COAP_CODE_GET)
        {
            responseCode = OT_COAP_CODE_CONTENT;
        }
        else
        {
            responseCode = OT_COAP_CODE_VALID;
        }

        responseMessage = otCoapNewMessage(sInstance, NULL);
        VerifyOrExit(responseMessage != NULL, error = OT_ERROR_NO_BUFS);

        otCoapMessageInit(responseMessage, OT_COAP_TYPE_ACKNOWLEDGMENT, responseCode);
        otCoapMessageSetMessageId(responseMessage, otCoapMessageGetMessageId(aMessage));
        otCoapMessageSetToken(responseMessage, otCoapMessageGetToken(aMessage), otCoapMessageGetTokenLength(aMessage));

        if (otCoapMessageGetCode(aMessage) == OT_COAP_CODE_GET)
        {
            otCoapMessageSetPayloadMarker(responseMessage);
            SuccessOrExit(error = otMessageAppend(responseMessage, &responseContent, sizeof(responseContent)));
        }

        SuccessOrExit(error = otCoapSendResponse(sInstance, responseMessage, aMessageInfo));
    }

exit:

    if (error != OT_ERROR_NONE)
    {
        if (responseMessage != NULL)
        {
        	otLogInfoPlat("coap send response error %d: %s\r\n", error,
                                               otThreadErrorToString(error));
            otMessageFree(responseMessage);
        }
    }
    else if (responseCode >= OT_COAP_CODE_RESPONSE_MIN)
    {
    	otLogInfoPlat("coap response sent successfully!\r\n");
    }
}

void coapAppInit()
{
	//    error = otCoapStart(sInstance, OT_DEFAULT_COAP_PORT);
	    cr_1.mUriPath = "test";
	    cr_1.mHandler = &coap_handler_test;
	    error = otCoapAddResource(sInstance, &cr_1);

	    cr_led.mUriPath = "led";
	    cr_led.mHandler = &coap_handler_led;
	    error = otCoapAddResource(sInstance, &cr_led);

	    cr_lux.mUriPath = "lux";
	    cr_lux.mHandler = &coap_handler_lux;
	    error = otCoapAddResource(sInstance, &cr_lux);

	    cr_mode.mUriPath = "device";
	    cr_mode.mHandler = &coap_handler_mode;
	    error = otCoapAddResource(sInstance, &cr_mode);
}

void getLightLevel()
{
	light_lvl_last = light_level_adc;
    ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
    while (0U == (kADC16_ChannelConversionDoneFlag &
                  ADC16_GetChannelStatusFlags(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP)))
    {
    }
	light_level_adc = ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP);
	if (light_level_adc > (light_lvl_trigger + light_lvl_gyst))	light_trigger = light_goes_up;
	if (light_level_adc < (light_lvl_trigger - light_lvl_gyst))	light_trigger = light_goes_down;
}
