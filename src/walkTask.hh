/*
 *  Copyright 2015 CNRS-AIST
 *
 *
 */

#ifndef WALK_TASK_HH
#define WALK_TASK_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <iostream>
#include <fstream>
#include <time.h>
#include <math.h>

namespace dynamicgraph {
  namespace sot {
    namespace tools {
      using dynamicgraph::Entity;
      using dynamicgraph::Vector;
      using dynamicgraph::Signal;
      using dynamicgraph::sot::MatrixHomogeneous;
      /**
       \brief Compute desired walking velocity for pattern Generator

       This class computes the desired waking velocity por PG given a goal position of the waist.
    */
      class walkTask : public Entity
      {

        DYNAMIC_GRAPH_ENTITY_DECL();
        walkTask(const std::string& inName);

        ~walkTask();


        /// Header documentation of the python class
        virtual std::string getDocString () const {
          return
              "Walking task for computing a desired velocity for PG\n";
        }

        void setGainConstant (const double& inGcte)
        {
          gain_ = inGcte;
        }

        void setAdaptiveGain (const double& inmaxGain)
        {
          A_ = inmaxGain;
          C_ = 0.0;
          double p = max_vel_/inmaxGain;  // Max velocity allowed = gain*error, when error = 1, vel = gain
          if (A_ == 0)    
            B_ = 0;
          else
            B_ = (-1)*log( p );
        }

        void setGoalPosition (const Vector& inPos) {
          posDes_ = inPos;
        }

      protected:


      private:
        double computeGain();
        double getYaw(const MatrixHomogeneous& Rw);

        Vector& computeDesiredVel(Vector& veldes, const int& inTime);

        SignalPtr <MatrixHomogeneous, int> waistSIN;

        SignalTimeDependent < ::dynamicgraph::Vector, int > velocitySOUT;

        ///
        double gain_, A_, B_, C_, max_vel_;
        Vector posDes_, e_, e_dot_;
        int t_1_;

        // Temporary variables for internal computations
        std::ofstream pos_, vel_;

        //For realtime
        struct tm iniTime_;

      }; // class walkTask
    } // namespace tools
  } //namespace sot
} // namespace dynamicgraph

#endif //WALK_TASK_HH
