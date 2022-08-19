#include "update_handler.h"

UpdateHandler::UpdateHandler():idx_current_state_{0},
                               idx_state_cov_{0},
                               idx_measurement_state_{0},
                               new_imu_msg_{false},
                               new_measurement_msg_{false}
{ }

UpdateHandler::~UpdateHandler()
{ }

uint16_t UpdateHandler::getCurrentStateIdx() const
{
    return idx_current_state_;
}

uint16_t UpdateHandler::getLastMeaStateIdx() const
{
    return idx_measurement_state_;
}

bool UpdateHandler::IsImuMsg() const
{
    return new_imu_msg_;
}

bool UpdateHandler::IsMeaMsg() const
{
    return new_measurement_msg_;
}

void UpdateHandler::setIsImuMsg(bool& is_imu)
{
    new_imu_msg_ = is_imu;
}

void UpdateHandler::setIsMeaMsg(bool& is_measurement)
{
    new_measurement_msg_ = is_measurement;
}

void UpdateHandler::setCurrentStateIdx(const uint16_t& idx)
{
    idx_current_state_ = idx;
}

void UpdateHandler::setLastMeaStateIdx(const uint16_t& idx)
{
    idx_measurement_state_ = idx;
}