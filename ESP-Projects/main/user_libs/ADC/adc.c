#include "adc.h"
#include "utils.h"
#include <esp_adc/adc_continuous.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include <driver/adc.h>
#include "esp_err.h"




    

// read a decimal value between 0 and 1 indicating position of potentiometer
float readPot(Pot* pot) {
    
    esp_err_t ret;
    uint8_t result[256];          // this line makes a new array called 'result' of size 256 bytes
    uint32_t ret_num = 0;          // this variable will hold the number of bytes read from the ADC
    uint32_t data_samples = 0;
    memset(result, 0xcc, 256);    // set all elements at results buffer to value 0xcc

     ret = adc_continuous_read(pot->handle, result, 256, &ret_num, 0);

            if (ret == ESP_OK){
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
                {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
                    data_samples = p->type2.data;
                    printf("Value:\t%ld\n", data_samples);
                }
                vTaskDelay(pdMS_TO_TICKS(100));     // Add delay between reads
     
	return map(pot->minPos, pot->maxPos, data_samples + pot->actuatorOffset); }
}


// this function initializes a Potentiometer struct and its ADC handle 
Pot PotInit(adc_continuous_handle_t *handle) {
	Pot pot = {
		.handle = handle,
		.read = readPot,
		.minPos = 1190,
		.maxPos = 3153,
	};

	adc_continuous_start(pot->handle);

    // this is how you intialize  the ADC Continous Driver mode 
    adc_continuous_handle_t handle = NULL;
    adc_continuous_handle_cfg_t adc_config = {
    .max_store_buf_size = 1024,
    .conv_frame_size = 256,
};  

	return pot;
}


// Deinitialize the Potentiometer's ADC handle --> Helpers to avoid memory leaks
void PotDeInit(Pot* pot) {
    if (pot != NULL && pot->handle != NULL) {
        adc_continuous_stop(pot->handle);       // Stop ADC continuous reading
        adc_continuous_delete_handle(pot->handle);      // Delete ADC handle to allow free memory
        pot->handle = NULL;
        ESP_LOGI(TAG, "Potentiometer ADC Deinitialized.");
    }
}