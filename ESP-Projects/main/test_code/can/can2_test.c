#include "utils.h"
#include "can2.h"

void app_main()
{
    twai_node_handle_t *node_hdl = malloc(sizeof(twai_node_handle_t));
    TalonFX fx = talonFXInit(27);
    TalonSRX srx = TalonSRXInit(4, false);
    printf("Can Setup\n");
    canSetup(node_hdl);
    // ESP_ERROR_CHECK(twai_node_register_event_callbacks(node_hdl, &user_cbs, NULL));
    printf("Can Setup Done\n");
    for (;;)
    {
        sendEn(node_hdl);
        setSRX(node_hdl, &srx, 0);
    }
}