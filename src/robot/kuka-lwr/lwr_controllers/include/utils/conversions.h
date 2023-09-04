#ifndef LWR_CONTROLLERS_CONVERSIONS_H_
#define LWR_CONTROLLERS_CONCERSIONS_H_

#include <kdl/frames.hpp>
#include <control_core/types.h>

namespace lwr_controllers
{

  /*!
  * \brief KDL::Frame to cc::CartesianPosition
  */
  inline cc::CartesianPosition convert(const KDL::Frame& T_kdl)
  {
    cc::CartesianPosition X;
    for(int i = 0; i < 3; ++i)
    {
      X.pos()[i] = T_kdl.p.data[i];
    }
    T_kdl.M.GetQuaternion(
      X.angular().x(),
      X.angular().y(),
      X.angular().z(),
      X.angular().w());
    return X;
  }

}

#endif