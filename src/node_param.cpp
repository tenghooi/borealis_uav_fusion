#include "node_param.h"

NodeParams::NodeParams() {}

NodeParams::~NodeParams() {}

void NodeParams::SetNodeParams(const ros::NodeHandle& node)
{
    node.param<int>("n_states", n_states_, 6);
    node.param<int>("n_control_inputs", n_control_inputs_, 6);
    node.param<int>("n_measurement_states", n_measurement_states_, 3);




}