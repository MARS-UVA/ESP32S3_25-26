#include "adc.h"
#include "utils.h"
#include <esp_adc/adc_continuous.h>
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include <driver/adc.h>



#define POT_CALIBRATION_COUNT 20            // change this to the ESP equivalent

esp_err_t ret;
    uint32_t ret_num = 0;
    uint8_t result[256] = {0};
    memset(result, 0xcc, 256);
    

// read a decimal value between 0 and 1 indicating position of potentiometer
float readPot(Pot* pot) {
	adc_continuous_start(pot->handle);
	// adc_continuous_read(pot->handle, 20);
    
     ret = adc_continuous_read(handle, result, 256, &ret_num, 0);
     uint32_t data;

            if (ret == ESP_OK){
                for (int i = 0; i < ret_num; i += SOC_ADC_DIGI_RESULT_BYTES)
                {
                    adc_digi_output_data_t *p = (adc_digi_output_data_t *)&result[i];
                    data = p->type2.data
                    printf("Value:\t%ld\n", data);
                }
                vTaskDelay(pdMS_TO_TICKS(100));     // Add delay between reads
    
    // 
	return map(pot->minPos, pot->maxPos, data + pot->actuatorOffset); }
}


# 
Pot PotInit(ADC_HandleTypeDef *handle) {
	Pot pot = {
		.handle = handle,
		.read = readPot,
		.minPos = 1190,
		.maxPos = 3153,
	};

	adc_continuous_handle_start(handle);

    // this is how you intialize  the ADC Continous Driver mode 
    adc_continuous_handle_t handle = NULL;
    adc_continuous_handle_cfg_t adc_config = {
    .max_store_buf_size = 1024,
    .conv_frame_size = 256,
};  

	return pot;
}