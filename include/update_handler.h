#ifndef _BOREALIS_FUSION_UPDATE_HANDLER_H_
#define _BOREALIS_FUSION_UPDATE_HANDLER_H_

#include <iostream>
#include <vector>
#include <string>
#include <queue>

#include <ros/ros.h>

#include "state.h"

class UpdateHandler
{
private:
    State state_buffer_[N_STATE_BUFFER];

    uint16_t idx_current_state_;
    uint16_t idx_state_cov_;
    uint16_t idx_measurement_update_;

    bool new_imu_msg_;
    bool new_measurement_msg_;

    
public:
    UpdateHandler();
    ~UpdateHandler();

    void PropagateStates();
    void UpdateStates();




};



#endif // _BOREALIS_FUSION_UPDATE_HANDLER_H_