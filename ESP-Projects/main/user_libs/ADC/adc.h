#include "main.h"
#include "utils.h"




typedef struct pot {
	ADC_HandleTypeDef *handle;
	float(*read)(struct pot*);
	float(*readCm)(struct pot*);
	uint32_t actuatorOffset;
	int minPos;
	int maxPos;
} Pot;

Pot PotInit();

void calibrateYourMom(Pot *leftPot, Pot *rightPot);

#endif