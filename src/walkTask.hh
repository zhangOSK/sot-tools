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
          C_ = 0.15;
          A_ = inmaxGain - C_;
          double p = (1.0 - C_)/A_;  // Set gain = 1 for e = max_vel_
          if (A_ == 0)    
            B_ = 0;
          else
            B_ = (-1/max_vel_(0))*log( p );
        
          factor_(1) = 2.0;
          factor_(2) = 2.0;
        }

        void setGoalPosition (const Vector& inPos)
        {
	  posDes_ = inPos;
          double mag = posDes_.norm();
          gain_ = max_vel_(0)/mag;
        }

        void setErrorTolerance (const Vector& inTolerance)
        {
          tolerance_ = inTolerance;
        }

      protected:


      private:
        double computeGain();
        double getYaw(const MatrixHomogeneous& Rw);

        Vector& computeDesiredVel(Vector& veldes, const int& inTime);

        SignalPtr <MatrixHomogeneous, int> waistSIN;

        SignalTimeDependent < ::dynamicgraph::Vector, int > velocitySOUT;

        ///
        double gain_, A_, B_, C_, eyt_1_, rms_;
        Vector posDes_, e_, e_dot_, max_vel_, factor_, tolerance_, endVel_;
        int t_1_;
        bool init_;

        // Temporary variables for internal computations
        std::ofstream pos_, vel_;

        //For realtime
        struct tm iniTime_;

      }; // class walkTask
    } // namespace tools
  } //namespace sot
} // namespace dynamicgraph

#endif //WALK_TASK_HH
