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
#include <sot/core/vector-roll-pitch-yaw.hh>
#include <iostream>
#include <fstream>
#include <time.h>
#include <math.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>

//#define DEBUG

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
	      posDes_ = Eigen::Vector3d(inPos(0), inPos(1), inPos(2));
          double mag = posDes_.norm();
          gain_ = max_vel_(0)/mag;
        }

        void setErrorTolerance (const Vector& inTolerance)
        {
          tolerance_ = Eigen::Vector3d(inTolerance(0), inTolerance(1), inTolerance(2));
        }

        inline Eigen::Vector3d extractVector(const MatrixHomogeneous& m);
        inline Eigen::MatrixXd extractMatrix(const MatrixHomogeneous& m);

      protected:


      private:
        double computeGain();
        double getYaw(const MatrixHomogeneous& Rw);

        Vector& computeDesiredVel(Vector& veldes, const int& inTime);

        SignalPtr <MatrixHomogeneous, int> waistSIN;
        SignalPtr <MatrixHomogeneous, int> rightFootSIN;
        SignalPtr <MatrixHomogeneous, int> leftFootSIN;

        SignalTimeDependent < ::dynamicgraph::Vector, int > velocitySOUT;

        ///
        double gain_, A_, B_, C_, eyt_1_;
        Eigen::Vector3d posDes_, e_, e_dot_, max_vel_, factor_, tolerance_, endVel_;
        int t_1_;
        bool init_;

        // Temporary variables for internal computations
        Eigen::Vector3d pos_, erot_;
        Vector posL_,posR_;

#ifdef DEBUG
        // debugging damping variable
        std::ofstream pose_file_, vel_file_;
#endif

      }; // class walkTask
    } // namespace tools
  } //namespace sot
} // namespace dynamicgraph

#endif //WALK_TASK_HH
