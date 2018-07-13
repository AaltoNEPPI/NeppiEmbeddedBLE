/*
 * Created by Pekka Nikander and others at Aalto University in 2018.
 *
 * This code has been placed in public domain.
 */

#include <stdio.h>
#include <inttypes.h>

#include <stdbool.h>
#include <stdint.h>
#include "ble_advdata.h"
#include "ble_conn_params.h"
#include "board.h"
#include "thread.h"
#include "msg.h"
#include "nrf_ble_gatt.h"
#include "nrf_sdh_ble.h"
#include "app_error.h"
#include "net/gnrc/netif.h"

#include "ble-core.h"

#include "xtimer.h"
#include "board.h"

#include "periph/gpio.h"
#define ENABLE_DEBUG (1)
#include "debug.h"

#define LED_CONNECTED_ON  LED1_ON
#define LED_CONNECTED_OFF LED1_OFF

/**
 * RIOT thread priority for the BLE handler.
 * We use the same thread priority as for TCP/IP network interfaces.
 */
#define BLE_THREAD_PRIO (GNRC_NETIF_PRIO)

/**
 * Advertised device name
 */
#ifndef DEVICE_NAME
#define DEVICE_NAME "NeppiT"
#endif // DEVICE_NAME

/**
 * Application's BLE observer priority. You shouldn't need to modify this value.
 * Used in NRF_SDH_BLE_OBSERVER.
 * By default there are a maximum of four levels, 0-3.
 * By default, the application priority should be lowest (highest number).
 */
#define APP_BLE_OBSERVER_PRIO 3 /* (NRF_SDH_BLE_OBSERVER_PRIO_LEVELS-1) */

/**
 * A tag for the SoftDevice BLE configuration.
 *
 * This is basically any small integer.  By convention,
 * one is used for generic applications.
 */
#define APP_BLE_CONN_CFG_TAG 1

/**************
 * BLE Advertisement parameters
 **************/

/**
 * The advertising interval (in units of 0.625 ms).
 * This value can vary between 100ms to 10.24s).
 */
#define APP_ADV_INTERVAL MSEC_TO_UNITS(50, UNIT_0_625_MS)

/**************
 * BLE GAP parameters
 **************/

/**
 * Minimum acceptable connection interval (0.5 seconds).
 */
#define MIN_CONN_INTERVAL MSEC_TO_UNITS(100, UNIT_1_25_MS)

/**
 * Maximum acceptable connection interval (1 second).
 */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(200, UNIT_1_25_MS)

/**
 * Slave latency.
 */
#define SLAVE_LATENCY 0

/**
 * Connection supervisory time-out (4 seconds).
 */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)

/*************
 * Messages from the BLE thread to the main thread
 *************/

#define UPDATE_ACC    1
#define UPDATE_ENERGY 2

/*************
 * UUIDs used for the service and characteristics
 *************/

// 128-bit base UUID
#define BLE_UUID_OUR_BASE_UUID \
{{ 0x8D, 0x19, 0x7F, 0x81, \
   0x08, 0x08, 0x12, 0xE0, \
   0x2B, 0x14, 0x95, 0x71, \
   0x05, 0x06, 0x31, 0xB1, \
}}

// Just a random, but recognizable values, for the service and characteristics
#define BLE_UUID_OUR_SERVICE                             0xABDC
#define BLE_UUID_ENERGY_CHARACTERISTIC                 	 0xBBCF
#define BLE_UUID_CONTROLS_CHARACTERISTIC                 0xBBD0

typedef struct {
    /**
     * Handle of the current connection (as provided by the BLE stack,
     * is BLE_CONN_HANDLE_INVALID if not in a connection).
     */
    uint16_t conn_handle;
    /**
     * Handle of Our Service (as provided by the BLE stack).
     */
    uint16_t service_handle;
    /**
     * Handle of characteristic (as provided by the BLE stack).
     */
    ble_gatts_char_handles_t char_handles[2];
} ble_os_t;

static ble_os_t our_service;

/**************
 * RIOT thread IDs
 **************/

static kernel_pid_t main_pid;
static kernel_pid_t ble_thread_pid;

static void ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context);
static void add_characteristic(ble_os_t* p_our_service, uint16_t  characteristic, uint8_t char_index);

/**
 * Function called by the NRF_ASSERT macro when an assertion
 * fails somewhere in the NRF SDK.
 *
 * For now, we just print an error message and enter a busy
 * loop, waiting for a developer with a debugger.
 *
 * Note that this is NOT good for a production version.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    core_panic(PANIC_ASSERT_FAIL, "RIOT: NRF Assertion failed.\n");
}

/**
 * A local RIOT-nice error handler replacing NRF's APP_ERROR_CHECK
 */
#define NRF_APP_ERROR_CHECK(err_code)                                           \
do {                                                                            \
  if (NRF_SUCCESS != err_code) {                                                \
    DEBUG("NRF call failed: err=%lu, %s#%d\n", err_code, __FILE__, __LINE__);   \
    return;                                                                     \
  }                                                                             \
} while(0)

static ble_uuid_t adv_uuids[] = {{BLE_UUID_OUR_SERVICE, 0 /* Filled dynamically */}};

static const ble_context_t ble_context = {
    .conn_cfg_tag = APP_BLE_CONN_CFG_TAG,
    .name = DEVICE_NAME,
    .adv_uuids = adv_uuids,
    .adv_uuid_cnt = sizeof(adv_uuids) / sizeof(adv_uuids[0]),
    .app_adv_interval = APP_ADV_INTERVAL,
};

// Register a handler for BLE events.
NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);

static void services_init(const ble_context_t* p_ble_context)
{
    ble_uuid_t *p_service_uuid = p_ble_context->adv_uuids;
    ble_os_t*   p_our_service = &our_service;

    uint32_t      err_code;
    ble_uuid128_t base_uuid = BLE_UUID_OUR_BASE_UUID;

    /* service_uuid_p->uuid already filled in */
    err_code = sd_ble_uuid_vs_add(&base_uuid, &p_service_uuid->type);
    NRF_APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
					p_service_uuid,
					&p_our_service->service_handle);
    NRF_APP_ERROR_CHECK(err_code);

    p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;

    add_characteristic(p_our_service, BLE_UUID_CONTROLS_CHARACTERISTIC, 0);
    add_characteristic(p_our_service, BLE_UUID_ENERGY_CHARACTERISTIC, 1);
}

/**
 * Function for the GAP initialization.
 *
 * This function sets up all the necessary GAP (Generic Access
 * Profile) parameters of the including the device name, appearance,
 * and the preferred connection parameters.
 */
static void gap_params_init(const ble_context_t* p_ble_context)
{
    ret_code_t              err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
					  (uint8_t*)p_ble_context->name,
                                          strlen(p_ble_context->name));
    NRF_APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    NRF_APP_ERROR_CHECK(err_code);
}


/**
 * Function for initializing the GATT module.
 */
static void gatt_init(void)
{
    NRF_BLE_GATT_DEF(m_gatt);

    ret_code_t err_code = nrf_ble_gatt_init(&m_gatt, NULL);
    NRF_APP_ERROR_CHECK(err_code);
}

static void add_characteristic(ble_os_t* p_our_service, uint16_t  characteristic, uint8_t char_index)
{
    ble_uuid128_t base_uuid = BLE_UUID_OUR_BASE_UUID;
    uint32_t      err_code = 0;
    ble_uuid_t    char_uuid;

    char_uuid.uuid = characteristic;
    sd_ble_uuid_vs_add(&base_uuid, &char_uuid.type);
    NRF_APP_ERROR_CHECK(err_code);

    // Add read/write properties to our characteristic
    ble_gatts_char_md_t char_md;
    memset(&char_md, 0, sizeof(char_md));
    char_md.char_props.read = 1;
    char_md.char_props.write = 1;

    // Configuring Client Characteristic Configuration Descriptor metadata and
    // add to char_md structure
    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;
    char_md.p_cccd_md = &cccd_md;
    char_md.char_props.notify = 1;

    // Configure the attribute metadata
    ble_gatts_attr_md_t attr_md;
    memset(&attr_md, 0, sizeof(attr_md));
    attr_md.vloc = BLE_GATTS_VLOC_STACK;

    // Set read/write security levels to our characteristic
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);

    // Configure the characteristic value attribute
    ble_gatts_attr_t    attr_char_value;
    memset(&attr_char_value, 0, sizeof(attr_char_value));
    attr_char_value.p_uuid = &char_uuid;
    attr_char_value.p_attr_md = &attr_md;

	// Set characteristic length in number of bytes
    attr_char_value.max_len = 1;
    attr_char_value.init_len = 1;
    uint8_t value[1] = { 0x12 };
    attr_char_value.p_value = value;

    // Add our new characteristic to the service
    err_code = sd_ble_gatts_characteristic_add(p_our_service->service_handle,
            &char_md,
            &attr_char_value,
            &p_our_service->char_handles[char_index]);
    NRF_APP_ERROR_CHECK(err_code);
}

static void on_ble_evt(ble_os_t * p_our_service, ble_evt_t const * p_ble_evt)
{
    ret_code_t err_code;

    switch (p_ble_evt->header.evt_id) {

    case BLE_GATTS_EVT_SYS_ATTR_MISSING:
	// No system attributes have been stored.
	err_code = sd_ble_gatts_sys_attr_set(our_service.conn_handle, NULL, 0, 0);
	NRF_APP_ERROR_CHECK(err_code);
	break;

    case BLE_GATTC_EVT_TIMEOUT:
	// Disconnect on GATT Client timeout event.
	DEBUG("GATT Client Timeout.\n");
	err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
					 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	NRF_APP_ERROR_CHECK(err_code);
	break;

    case BLE_GATTS_EVT_TIMEOUT:
	// Disconnect on GATT Server timeout event.
	DEBUG("GATT Server Timeout.\n");
	err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
					 BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
	NRF_APP_ERROR_CHECK(err_code);
	break;

    default:
	break;
    }
}

static void on_ble_write(ble_os_t * p_our_service, ble_evt_t const * p_ble_evt)
{
    // Declare buffer variable to hold received data. The data can only be 32 bit long.
    uint8_t data_buffer;
    // Populate ble_gatts_value_t structure to hold received data and metadata.
    ble_gatts_value_t rx_data;
    rx_data.len = sizeof(uint8_t);
    rx_data.offset = 0;
    rx_data.p_value = &data_buffer;

    // Check if write event is performed on our characteristic or the CCCD
    for (uint8_t i = 0; i < 2; i++) {
	if (p_ble_evt->evt.gatts_evt.params.write.handle
	    == p_our_service->char_handles[i].value_handle) {
	    // Get data
	    sd_ble_gatts_value_get(p_our_service->conn_handle,
				   p_our_service->char_handles[i].value_handle, &rx_data);
	    DEBUG("Value CCCD recv\n");

	    //Pyry was here
	    if(((unsigned int)data_buffer) == 0x00000000){LED1_OFF;LED2_OFF;LED3_OFF;}
	    else if(((unsigned int)data_buffer) == 0x00000002){LED1_ON;LED2_OFF;LED3_OFF;}
	    else if(((unsigned int)data_buffer) == 0x00000003){LED1_OFF;LED2_ON;LED3_OFF;}
	    else if(((unsigned int)data_buffer) == 0x00000004){LED1_OFF;LED2_OFF;LED3_ON;}

	} else if (p_ble_evt->evt.gatts_evt.params.write.handle
		   == p_our_service->char_handles[i].cccd_handle) {
	    DEBUG("Value recv\n");
	    // Get data
	    sd_ble_gatts_value_get(p_our_service->conn_handle,
				   p_our_service->char_handles[i].cccd_handle, &rx_data);
	}
    }
}

void ble_our_service_on_ble_evt(ble_os_t * p_our_service, ble_evt_t const * p_ble_evt)
{
    // Implement switch case handling BLE events related to our service.
    switch (p_ble_evt->header.evt_id) {
    case BLE_GAP_EVT_CONNECTED:
	LED_CONNECTED_ON;
	p_our_service->conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
	break;
    case BLE_GAP_EVT_DISCONNECTED:
	LED_CONNECTED_OFF;
	p_our_service->conn_handle = BLE_CONN_HANDLE_INVALID;
	break;
    case BLE_GATTS_EVT_WRITE:
	on_ble_write(p_our_service, p_ble_evt);
	break;
    default:
	// No implementation needed.
	break;
    }
}

void acc_characteristic_update(ble_os_t *p_our_service, uint32_t *acc_value, uint8_t char_index)
{
    if (p_our_service->conn_handle != BLE_CONN_HANDLE_INVALID) {
	uint16_t               len = 1;
	ble_gatts_hvx_params_t hvx_params;
	memset(&hvx_params, 0, sizeof(hvx_params));

	hvx_params.handle = p_our_service->char_handles[char_index].value_handle;
	hvx_params.type   = BLE_GATT_HVX_NOTIFICATION;
	hvx_params.offset = 0;
	hvx_params.p_len  = &len;
	hvx_params.p_data = (uint8_t*)acc_value;

	sd_ble_gatts_hvx(p_our_service->conn_handle, &hvx_params);
    } else {
	uint16_t          len = 1;
	ble_gatts_value_t tx_data;
	tx_data.len     = len;
	tx_data.offset  = 0;
	tx_data.p_value = (uint8_t*)acc_value;
	sd_ble_gatts_value_set(p_our_service->conn_handle,
			       p_our_service->char_handles[char_index].value_handle,
			       &tx_data);
    }
}

static void
ble_evt_handler(ble_evt_t const * p_ble_evt, void * p_context)
{
    //ble_conn_params_on_ble_evt(p_ble_evt);
    on_ble_evt(&our_service, p_ble_evt);
    ble_our_service_on_ble_evt(&our_service, p_ble_evt);
}

void *ble_thread(void *arg)
{
    (void)arg;

    DEBUG("2nd thread started, pid: %" PRIkernel_pid "\n", thread_getpid());

    // Start execution.
    ble_advertising_start(&ble_context);

    for (;;) {
	msg_t m;
	msg_receive(&m);
	// DEBUG("message received: type=%d\n", m.type);
	switch (m.type) {
	case UPDATE_ACC:
	    acc_characteristic_update(&our_service, &m.content.value, 0);
	    break;
	case UPDATE_ENERGY:
	    acc_characteristic_update(&our_service, &m.content.value, 1);
	    break;
	default:
	    break;

	}
    }
}

static char ble_thread_stack[(THREAD_STACKSIZE_DEFAULT*2)];

int main(void)
{
    DEBUG("Entering main function.\n");

    ble_init(&ble_context);
    gap_params_init(&ble_context);
    gatt_init();
    services_init(&ble_context);
    ble_advertising_init(&ble_context);

    main_pid = thread_getpid();
    msg_t main_message;

    gpio_init(BTN0_PIN, BTN0_MODE);
    gpio_init(BTN1_PIN, BTN1_MODE);
    gpio_init(BTN2_PIN, BTN2_MODE);
    gpio_init(BTN3_PIN, BTN3_MODE);

    ble_thread_pid = thread_create(ble_thread_stack, sizeof(ble_thread_stack),
				   BLE_THREAD_PRIO, 0/*THREAD_CREATE_STACKTEST*/,
				   ble_thread, NULL, "BLE");
    LED0_TOGGLE;
	
    DEBUG("Entering main loop.\n");

    uint8_t state = 0;

    for (;;) {
	uint8_t new_state = 0;

	new_state |= (!gpio_read(BTN0_PIN)) << 0;
	new_state |= (!gpio_read(BTN1_PIN)) << 1;
	new_state |= (!gpio_read(BTN2_PIN)) << 2;
	new_state |= (!gpio_read(BTN3_PIN)) << 3;

	if (new_state != state) {
	    state = new_state;
	    main_message.type = UPDATE_ACC;
	    main_message.content.value = state;
	    msg_send(&main_message, ble_thread_pid);
	    DEBUG("New button state: %d\n", state);
	}
	xtimer_usleep(100000);
    }
}
