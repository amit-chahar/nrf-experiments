#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#include "softdevice_handler.h"
#include "ble.h"
#include "bsp.h"
#include "bsp_btn_ble.h"
#include "nrf_delay.h"
#include "SEGGER_RTT.h"
#include "ble_advdata.h"
#include "ble_advertising.h"

#define CENTRAL_LINK_COUNT              0                                           /**< Number of central links used by the application. When changing this number remember to adjust the RAM settings*/
#define PERIPHERAL_LINK_COUNT           1                                           /**< Number of peripheral links used by the application. When changing this number remember to adjust the RAM settings*/

#define DEAD_BEEF                       0xDEADBEEF                                  /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#define APP_ADV_INTERVAL                1000                                        /**< The advertising interval (in units of 0.625 ms. 300 value corresponds to 187.5 ms). */
#define APP_ADV_TIMEOUT_IN_SECONDS      0                                           /**< The advertising timeout in units of seconds. 0 means unlimited. */
#define DEVICE_NAME                     "Hello world"                               /**< Name of device. Will be included in the advertising data. */
#define RPA_REFRESH_INTERVAL						10																					/**< RPA refresh interval in seconds. */

#define APP_PRAND_LEN (3)
#define APP_HASH_LEN (3)
#define APP_ADDR_LEN (6)

uint32_t AES_ECB_BLOCK_SIZE = 16;
static const uint8_t IRK[16] = { 'N', 'o', 't', 'a', 'g', 'o', 'o', 'd', 'p', 'a', 's', 's', 'w', 'o', 'r', 'd' };

// Function to generate hash for the RPA
// Uses AES ECB encryption
void generate_hash(uint8_t const * p_k, uint8_t const * p_r, uint8_t * p_hash)
{
    nrf_ecb_hal_data_t ecb_hal_data;

    for (uint32_t i = 0; i < SOC_ECB_KEY_LENGTH; i++)
    {
        ecb_hal_data.key[i] = p_k[SOC_ECB_KEY_LENGTH - 1 - i];
    }

    memset(ecb_hal_data.cleartext, 0, SOC_ECB_KEY_LENGTH - APP_PRAND_LEN);

    for (uint32_t i = 0; i < APP_PRAND_LEN; i++)
    {
        ecb_hal_data.cleartext[SOC_ECB_KEY_LENGTH - 1 - i] = p_r[i];
    }

    // Can only return NRF_SUCCESS.
    (void) sd_ecb_block_encrypt(&ecb_hal_data);

    for (uint32_t i = 0; i < APP_HASH_LEN; i++)
    {
        p_hash[i] = ecb_hal_data.ciphertext[SOC_ECB_KEY_LENGTH - 1 - i];
    }
}


void RPA_generation_and_set(){
	uint32_t err_code;
	
	// Random number generator's pool capacity is 32 bytes. Random bytes are used for prand.
	//uint8_t pool_capacity;
	//sd_rand_application_pool_capacity_get(&pool_capacity);
	//SEGGER_RTT_printf(0, "Random pool capacity: %u\n", pool_capacity);
	
	uint8_t bytes_available, MAX_DELAY, total_delay;
	MAX_DELAY = 100;
	total_delay = 0;

	// If application is unable to get random bytes in 100 ms , this function will return
	// Only 3 bytes are needed for prand
	while(1){
			sd_rand_application_bytes_available_get(&bytes_available);
			if(bytes_available > 2){
				break;
			} else {
				if(total_delay == MAX_DELAY){
					SEGGER_RTT_printf(0, "Unable to get enough random numbers\n");
					return;
				}
				nrf_delay_ms(2);
				total_delay++;
			}
	}
	
	uint8_t prand[APP_PRAND_LEN];
	err_code = sd_rand_application_vector_get(prand, APP_PRAND_LEN);
	APP_ERROR_CHECK(err_code);
	
	//change to little endian format
	uint8_t prand_le[APP_PRAND_LEN];
	for(uint32_t i = 0; i < APP_PRAND_LEN; i++){
		prand_le[i] = prand[3 - 1 - i];
	}
	
	//set two most significant bits to 0 and 1 for RPA. Most significant bit will be first bit of last byte in little endian format.
	prand_le[2] &= 0x7f;
	prand_le[2] |= 0x40;
	
	uint8_t hash[APP_HASH_LEN];
	generate_hash(IRK, prand_le, hash);
	
	//merge hash and prand to generate address
	uint8_t rpa[APP_ADDR_LEN];
	memcpy(rpa, hash, APP_HASH_LEN);
	memcpy(&rpa[APP_HASH_LEN], prand_le, APP_PRAND_LEN);

	SEGGER_RTT_printf(0, "Generated RPA: %02x%02x%02x%02x%02x%02x\n", rpa[0], rpa[1], rpa[2], rpa[3], rpa[4], rpa[5]);
	
	//set the newly generated address
	ble_gap_addr_t new_add ;
	new_add.addr_type=BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE;
	for(uint32_t j = 0; j < BLE_GAP_ADDR_LEN; j++){  
		new_add.addr[j] = rpa[j];
	}
	err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_NONE, &new_add);
	APP_ERROR_CHECK(err_code);
}

// Callback function for asserts in the SoftDevice.
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

// This function is currently not used.
// This function set IRK and address refresh interval in the softdevice
// so that softdevice itself handle RPA generation and setting.
void RPA_config(){
	uint16_t RPA_refresh_intvl_s = 10;
	uint32_t err_code;
	ble_gap_irk_t m_irk;
	memcpy(m_irk.irk, IRK, BLE_GAP_SEC_KEY_LEN);
	
	ble_opt_t opt;
	ble_gap_addr_t gap_address;
	gap_address.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE;

	err_code = sd_ble_gap_address_set(BLE_GAP_ADDR_CYCLE_MODE_AUTO, &gap_address);
	APP_ERROR_CHECK(err_code);
	opt.gap_opt.privacy.p_irk = &m_irk;
	opt.gap_opt.privacy.interval_s = RPA_refresh_intvl_s;
	sd_ble_opt_set(BLE_GAP_OPT_PRIVACY, &opt);	
}

// Function for initializing the softdevice.
static void ble_stack_init(void)
{
    uint32_t err_code;

    nrf_clock_lf_cfg_t clock_lf_cfg = NRF_CLOCK_LFCLKSRC;

    // Initialize the SoftDevice handler module.
    SOFTDEVICE_HANDLER_INIT(&clock_lf_cfg, NULL);

    ble_enable_params_t ble_enable_params;
    err_code = softdevice_enable_get_default_config(CENTRAL_LINK_COUNT,
                                                    PERIPHERAL_LINK_COUNT,
                                                    &ble_enable_params);
    APP_ERROR_CHECK(err_code);

    // Check the ram settings against the used number of links
    CHECK_RAM_START_ADDR(CENTRAL_LINK_COUNT, PERIPHERAL_LINK_COUNT);

    err_code = softdevice_enable(&ble_enable_params);
    APP_ERROR_CHECK(err_code);
}

// This function sets up all the necessary GAP (Generic Access Profile) parameters of the 
// device. Here it is used to set the "Device Name" to be used in advertisements.
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
}

// Function for putting the chip into sleep mode.
static void sleep_mode_enter(void)
{
		uint32_t err_code;

    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

// This function will be called for advertising events which are passed to the application.
static void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{	
    switch (ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            SEGGER_RTT_printf(0, "Advertising started.");
            break;
				
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;

        default:
            break;
    }
}

// Function for initializing the Advertising functionality.
static void advertising_init(void)
{
    uint32_t               err_code;
    ble_advdata_t          advdata;
    ble_adv_modes_config_t options;

    memset(&advdata, 0, sizeof(advdata));

    advdata.name_type               = BLE_ADVDATA_FULL_NAME;
    advdata.flags                   = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;

    memset(&options, 0, sizeof(options));
    options.ble_adv_fast_enabled  = true;
    options.ble_adv_fast_interval = APP_ADV_INTERVAL;
    options.ble_adv_fast_timeout  = APP_ADV_TIMEOUT_IN_SECONDS;
	
    err_code = ble_advertising_init(&advdata, NULL, &options, on_adv_evt, NULL);
    APP_ERROR_CHECK(err_code);
}


//Function for application main entry.
int main(void)
{
		uint32_t err_code;
		ble_stack_init();
		
		gap_params_init();
		advertising_init();
		err_code = ble_advertising_start(BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
	
		while(true){
			RPA_generation_and_set();
			nrf_delay_ms(RPA_REFRESH_INTERVAL * 1000);
		}
}
