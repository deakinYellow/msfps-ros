#ifndef UIFP_COMMON_H
#define UIFP_COMMON_H

#include <math.h>
#include "../../common/utility-cc/tool.hpp"

namespace uifp {

///将取值转换到周期[-PI - PI]
///value绝对值必须小于 2PI+0.001：
static double TransformToCyclePi( const double& value ){
  if( fabs( value ) > 2*M_PI + 0.001 ){
    MLOGE_EXIT("input value abs must be less tan pi, but: %f ", value );
  }
  double ret;
  if( value > M_PI ){
    ret = value - 2 * M_PI;
  }
  else if( value < -1 *M_PI ){
    ret = value + 2 * M_PI;
  }
  else {
    ret = value;
  }
  return ret;
}


}  //end of namespace uifp

#endif

