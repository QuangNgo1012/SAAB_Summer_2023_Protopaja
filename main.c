/**
 * Copyright (c) 2014 - 2017, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "app_uart.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "bsp.h"
#include "boards.h"
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "app_pwm.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf.h"

#define UART_TX_BUF_SIZE            512    /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE            2048   /**< UART RX buffer size. */
#define NEW_PACKET_BYTE             255
#define RSSI_NO_SIGNAL              127    /**< Minimum value of RSSISAMPLE */                    /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define buzzer_pin 2
#define motor_pin 26
#define led_pin 17
#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          0x17                               /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                               /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                               /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                         /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                         /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */
#define DEVICE_NAME                      "DroneDetected"
static uint8_t min_channel        = 0;     /**< Lowest scanned channel if adv_channels_en = true */
static uint8_t max_channel        = 80;    /**< highest scanned channel if adv_channels_en = true */
static uint32_t sweep_delay       = 100;
static uint32_t scan_repeat_times = 1;
static bool uart_error = false;
static bool uart_send = false;
static bool scan_ble_adv = false;

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */
static uint8_t              m_enc_srdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX]; //modified
static volatile bool ready_flag;            // A flag indicating PWM status.
APP_PWM_INSTANCE(PWM1,1);  
void pwm_ready_callback(uint32_t pwm_id)    // PWM callback function
{
    ready_flag = true;
}

/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = m_enc_srdata, //modified
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX //modified

    }
};


static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this
                         // implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the
                         // manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in
                         // this implementation.
};

/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_advdata_t advdata;
    ble_advdata_t srdata;
    uint8_t       flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

    ble_advdata_manuf_data_t manuf_specific_data;

    manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;

    manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
    manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;
     
    memset(&advdata, 0, sizeof(advdata));
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;
    

    memset(&srdata, 0, sizeof(srdata)); 
    srdata.name_type             = BLE_ADVDATA_FULL_NAME;
    err_code = ble_advdata_encode(&srdata, m_adv_data.scan_rsp_data.p_data, &m_adv_data.scan_rsp_data.len);
    APP_ERROR_CHECK(err_code);
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL;    
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration        = 0;      

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);
    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
}

/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}
static void advertising_stop(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_stop(m_adv_handle);
    APP_ERROR_CHECK(err_code);

}

/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    if (err_code != NRF_SUCCESS)
    {
      NVIC_SystemReset();
    } 
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);

    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                      (const uint8_t *)DEVICE_NAME,
                                      strlen(DEVICE_NAME));
}

static void reset_ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_disable_request();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing LEDs. */
static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}

void set_uart_send_enable(bool enable)
{
	uart_send = enable;
}

void uart_puts(char *string)
{
  while(*string)
  {
    app_uart_put(*string);
    ++string;
  }
}

void set_scan_ble_adv(bool enable) {
	scan_ble_adv = enable;
}

void rssi_measurer_configure_radio(void)
{
	NRF_RADIO->POWER  = 1;
	NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk;
	NVIC_EnableIRQ(RADIO_IRQn);

	NRF_CLOCK->TASKS_HFCLKSTART = 1;
	while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}

void reset_rssi_measurer_configure_radio(void)
{
	NRF_RADIO->POWER  = 0;
	NVIC_DisableIRQ(RADIO_IRQn);
	NRF_CLOCK->TASKS_HFCLKSTART = 0;
}
#define WAIT_FOR( m ) do { while (!m); m = 0; } while(0)

uint8_t rssi_measurer_scan_channel(uint8_t channel_number)  
{
	uint8_t sample;

	NRF_RADIO->FREQUENCY  = channel_number;
	NRF_RADIO->TASKS_RXEN = 1;

	WAIT_FOR(NRF_RADIO->EVENTS_READY);
	NRF_RADIO->TASKS_RSSISTART = 1;
	WAIT_FOR(NRF_RADIO->EVENTS_RSSIEND);

	sample = 127 & NRF_RADIO->RSSISAMPLE;

	NRF_RADIO->TASKS_DISABLE = 1;
	WAIT_FOR(NRF_RADIO->EVENTS_DISABLED);

	return sample;
}

uint8_t rssi_measurer_scan_channel_repeat(uint8_t channel_number)
{
	uint8_t sample;
	uint8_t max = RSSI_NO_SIGNAL;
	for (int i = 0; i <= scan_repeat_times; ++i) {
		sample = rssi_measurer_scan_channel(channel_number);
		max = MIN(sample, max);
	}
	return max;
}

void uart_error_handle(app_uart_evt_t * p_event)
{
	if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
	{
		APP_ERROR_HANDLER(p_event->data.error_communication);
	}
	else if (p_event->evt_type == APP_UART_FIFO_ERROR)
	{
		APP_ERROR_HANDLER(p_event->data.error_code);
	}
}
uint8_t counter = 0 ;
uint8_t avr1 = 0 ;
uint8_t avr2 = 0 ;
void uart_loopback()
{
	uint8_t sample;
        uint32_t value;
        value = 50;
        uint8_t count1 = 0 ;
        
                uint8_t buf[81], maxvalue=0, minvalue=127, diffvalue;
		for (uint8_t i = min_channel; i <= max_channel; ++i)
		{
			sample = rssi_measurer_scan_channel_repeat(i);
                        buf[i]=sample;
                        maxvalue=(sample>maxvalue)?sample:maxvalue;
                        minvalue=(sample<minvalue)?sample:minvalue;
		}
                char *symbols[5]={" ", ".", ":", "|", "X"};
                diffvalue=maxvalue-minvalue;
                for (uint8_t i = min_channel; i <= max_channel; ++i)
                {
                  uint8_t choice=(4*((uint32_t)buf[i]-(uint32_t)minvalue))/(uint32_t)diffvalue;
                  uart_puts(symbols[choice]);
                  if (buf[i]<50)
                  {
                    ++count1;
                  }
                }
                if (count1>10)
                {
                  
                    reset_rssi_measurer_configure_radio();
                    nrf_sdh_disable_request();
                    nrf_sdh_disable_request();
                    ble_stack_init();
                    advertising_init();
                    advertising_start();
                    nrf_gpio_pin_set(motor_pin);
                    nrf_delay_ms(sweep_delay);
                    app_pwm_enable(&PWM1);
                    APP_ERROR_CHECK(app_pwm_channel_duty_set(&PWM1, 1, value));
                    nrf_delay_ms(sweep_delay);
                    nrf_gpio_pin_set(led_pin);
                    NRF_LOG_INFO("Drone detected.");
                    nrf_delay_ms(1000);
                    nrf_sdh_disable_request();
                    nrf_sdh_disable_request();
                    rssi_measurer_configure_radio();
                    app_pwm_disable(&PWM1);
                }else{
                  
                  nrf_gpio_pin_clear(led_pin);
                  nrf_gpio_pin_clear(motor_pin);
                }

                uart_puts("\r\n");
                nrf_delay_ms(sweep_delay);
                

	if (uart_error) {
		nrf_delay_ms(MAX(sweep_delay, 500));
		uart_error = false;
		set_uart_send_enable(uart_send);
	}
        
}

int main(void)
{
        
	uint32_t err_code;
        nrf_gpio_cfg_output(motor_pin);
        /* 2-channel PWM, 200Hz, output on DK LED pins. */
        app_pwm_config_t pwm1_cfg = APP_PWM_DEFAULT_CONFIG_2CH(238L, 2,17);

        /* Switch the polarity of the second channel. */
        pwm1_cfg.pin_polarity[1] = APP_PWM_POLARITY_ACTIVE_HIGH;

        /* Initialize and enable PWM. */
        err_code = app_pwm_init(&PWM1,&pwm1_cfg,pwm_ready_callback);
        APP_ERROR_CHECK(err_code);
	const app_uart_comm_params_t   comm_params = {
		RX_PIN_NUMBER,
		TX_PIN_NUMBER,
		RTS_PIN_NUMBER,
		CTS_PIN_NUMBER,
		APP_UART_FLOW_CONTROL_DISABLED,
		false,
		UART_BAUDRATE_BAUDRATE_Baud115200
	};

	APP_UART_FIFO_INIT(
		&comm_params,
		UART_RX_BUF_SIZE,
		UART_TX_BUF_SIZE,
		uart_error_handle,
		APP_IRQ_PRIORITY_LOWEST,
		err_code
	);

	APP_ERROR_CHECK(err_code);
	app_uart_flush();      
        log_init();
        timers_init();
        power_management_init();
        NRF_LOG_INFO("Project started.");
        rssi_measurer_configure_radio();
	while (true)
	{
		uart_loopback();
                idle_state_handle();

	}
}


/** @} */