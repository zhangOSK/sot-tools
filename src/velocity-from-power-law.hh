//
// Copyright (C) 2012 LAAS-CNRS
//
// Author: Naveau Maximilien,
//

#include <limits>
#include <time.h>

#include <dynamic-graph/command.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-direct-setter.h>
#include <dynamic-graph/command-direct-getter.h>

#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/factory.h>
#include <dynamic-graph/entity.h>

#include <sot/core/matrix-homogeneous.hh>

#include "power_law/PowerLaw.hh"

namespace dynamicgraph {
  namespace sot {
    using command::makeDirectSetter;
    using command::docDirectSetter;
    using command::docDirectGetter;
    using command::makeDirectGetter;
    namespace tools {
      class VelocityFromPowerLaw : public Entity
      {
        DYNAMIC_GRAPH_ENTITY_DECL();
      public:
        VelocityFromPowerLaw (const std::string name,
                              double radiusx,
                              double radiusy,
                              double gamma,
                              double beta,
                              double zerokapparadius);

        VelocityFromPowerLaw (const std::string name);

        dynamicgraph::Vector& computeVelocitySignal (dynamicgraph::Vector& vsout,
                                                     const int& time);
        void initializeEllipse (const double &radiusx,
                                const double &radiusy,
                                const double &gamma,
                                const double &beta);

        void initializeCommand();

        /*! \brief Externalize the velocity computed from the vector field. */
        SignalTimeDependent < dynamicgraph::Vector, int > vectorSoutSOUT_;

        /*! \brief Take the current left foot homogeneous position. */
        SignalPtr<MatrixHomogeneous,int> LeftFootCurrentPosSIN;

        /*! \brief Take the current right foot homogeneous position. */
        SignalPtr<MatrixHomogeneous,int> RightFootCurrentPosSIN;

        /*! \brief Take the current waist vector6d position. */
        SignalPtr<dynamicgraph::Vector,int> WaistCurrentPosSIN;

      private :
        EllipseVectorFieldNumeric::HopfParameters hp_;
        typedef PowerLaw<VelocityFromPowerLaw> PowerLawGeneration;
        PowerLawGeneration powerLawGeneration_;

      }; // class VelocityFromPowerLaw
    } // namespace tools
  } // namespace sot
} // namespace dynamicgraph
