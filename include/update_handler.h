#ifndef _BOREALIS_FUSION_UPDATE_HANDLER_H_
#define _BOREALIS_FUSION_UPDATE_HANDLER_H_

#include <iostream>
#include <vector>
#include <string>

class UpdateHandler
{
private:

    uint16_t idx_current_state_;
    uint16_t idx_state_cov_;
    uint16_t idx_measurement_update_;

    bool new_imu_msg_;
    bool new_measurement_msg_;
    
public:

    UpdateHandler();
    ~UpdateHandler();

    uint16_t getCurrentStateIdx() const;
    uint16_t getLastMeaStateIdx() const;

    bool IsImuMsg() const;
    bool IsMeaMsg() const;

    void setIsImuMsg(bool& is_imu);
    void setIsMeaMsg(bool& is_measurement);

    void setCurrentStateIdx(const uint16_t& idx);
    void setLastMeaStateIdx(const uint16_t& idx);

};

#endif // _BOREALIS_FUSION_UPDATE_HANDLER_H_