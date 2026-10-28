//
// Created by ASUS on 2026/7/30.
//

#ifndef JIUXIAN_SP_CHASSIS_XPP_DRV_ROBOMASTER_STANDARD_LINK_H
#define JIUXIAN_SP_CHASSIS_XPP_DRV_ROBOMASTER_STANDARD_LINK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "../../Referee_Info_Define/Inc/RoboMaster_Referee_Info_Define.h"
#include "../../Referee_Info_Define/Inc/RoboMaster_CRC.h"

    void Standard_Link_Init(void);

    void RoboMaster_Interaction_UI_Design_One_Graphic(interaction_figure_t *figure);

    void RoboMaster_Standard_Link_Rx_Data_Process(Referee_uint8 *data, Referee_uint16 len);
    void Standard_Link_Rx_Data_Parse(void);

#ifdef __cplusplus
}
#endif

#endif //JIUXIAN_SP_CHASSIS_XPP_DRV_ROBOMASTER_STANDARD_LINK_H