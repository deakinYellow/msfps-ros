#ifndef PARAMS_LOAD_H
#define PARAMS_LOAD_H

#include "../../mros/parameter.hpp"
#include "../../common/utility-math/meigen.hpp"
#include "uidata_type.h"


void load_anchors_map( mros::Parameter* parameter, uifp::AnchorsMapT& anchors_map );
void load_kf_params( mros::Parameter* parameter,
                     Eigen::Vector3d&  initial_position,
                     uifp::KfParametersT& kf_params,
                     uifp::DynamicThresholdParametersT& distance_innovation_dynamic_threshold_params,
                     uifp::KfStatusCheckParametersT& kf_status_check_params );

#endif // PARAMS_LOAD_H
