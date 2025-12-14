#include "main.h"
#include "utils.h"
#include "adc.h"
#include`<esp_adc/adc_continuous.h>




#define ADC_H


typedef struct pot {
	adc_continuous_handle_t *handle;
	float(*read)(struct pot*);
	float(*readCm)(struct pot*);
	uint32_t actuatorOffset;
	int minPos;				// minimum ADC reading
	int maxPos;				// maximum ADC reading
} Pot;

Pot PotInit(void);

float readPot(Pot* pot);

void calibrateYourMom(Pot *leftPot, Pot *rightPot);

void PotDeInit(Pot* pot);


#endif
