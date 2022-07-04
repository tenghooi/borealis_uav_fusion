#include "node_param.h"

NodeParams::NodeParams() {}

NodeParams::~NodeParams() {}

void NodeParams::SetNodeParams(const ros::NodeHandle& node)
{
    node.param<std::string>("borealis_fusion/frame_id", frame_id, "");

    node.param<int>("borealis_fusion/n_states", n_states_, 6);
    node.param<int>("borealis_fusion/n_control_inputs", n_control_inputs_, 6);
    node.param<int>("borealis_fusion/n_measurement_states", n_measurement_states_, 3);

    node.param<std::vector<double>>("borealis_fusion/process_transition_F", F_vector_form_, std::vector<double>());
    node.param<std::vector<double>>("borealis_fusion/control_transition_B", B_vector_form_, std::vector<double>());
    node.param<std::vector<double>>("borealis_fusion/observation_transition_H", H_vector_form_, std::vector<double>());

    node.param<std::vector<double>>("borealis_fusion/initial_states_cov_P", P_vector_form_, std::vector<double>());
    node.param<std::vector<double>>("borealis_fusion/process_noise_cov_Q", Q_vector_form_, std::vector<double>());
    node.param<std::vector<double>>("borealis_fusion/sensor_noise_cov_R", R_vector_form_, std::vector<double>());


}