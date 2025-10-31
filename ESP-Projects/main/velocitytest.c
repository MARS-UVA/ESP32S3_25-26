#include "CL_CAN.h"

void app_main()
{
canSetup();

while(1)
{
setTargetVelocity(30);
}
}
