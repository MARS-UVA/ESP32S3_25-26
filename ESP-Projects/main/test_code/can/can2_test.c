#include "utils.h"
#include "can2.h"

void app_main()
{
    twai_node_handle_t node_hdl = NULL;
    TalonFX fx = talonFXInit(&node_hdl, 27);
    TalonSRX srx = talonSRXInit(&node_hdl, 4, false);
    printf("Can Setup\n");
    canSetup(&node_hdl);
    // ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_hdl, &user_cbs, NULL));
    printf("Can Setup Done\n");
    for (;;)
    {
        sendEn(&node_hdl);
        setSRX(&srx, 0);
    }
}