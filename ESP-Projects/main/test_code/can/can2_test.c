#include "utils.h"
#include "can2.h"

void app_main()
{
    TalonFX fx = talonFXInit(27);
    TalonSRX srx = talonSRXInit(4, false);
    printf("Can Setup\n");
    canSetup();
    // ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_hdl, &user_cbs, NULL));
    printf("Can Setup Done\n");
    while (1)
    {
        // setTargetFX(&fx, 100);
        // setSRX(&srx, .5);
    }
}