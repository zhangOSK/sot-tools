/*
 *  Copyright 2010 CNRS
 *
 *
 */

#ifndef IMPEDANCE_CONTROLLER_HH
#define IMPEDANCE_CONTROLLER_HH

#include <dynamic-graph/entity.h>
#include <dynamic-graph/signal-ptr.h>
#include <dynamic-graph/signal-time-dependent.h>
#include <dynamic-graph/linear-algebra.h>
#include <sot/core/matrix-homogeneous.hh>
#include <sot/core/matrix-rotation.hh>
#include <iostream>
#include <fstream>
#include <time.h>
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
       \brief Impedance controller for manipulation a Hose

       This class implements an impedance control for the left wrist of the robot
    */
      class ImpedanceController : public Entity
      {

        DYNAMIC_GRAPH_ENTITY_DECL();
        ImpedanceController(const std::string& inName);

        ~ImpedanceController();


        /// Header documentation of the python class
        virtual std::string getDocString () const {
          return
              "Impedance controller for manipulating a hose\n";
        }

        void setMass (const double& inMass) {
          m_ = inMass;
        }

        double getMass () const {
          return m_;
        }

        void setDamping (const double& inDamping) {
          c_ = inDamping;
        }

        double getDamping () const {
          return c_;
        }

        void setForceDes (const Vector& inForce) {
          fd_ = Eigen::Vector3d(inForce(0), inForce(1), inForce(2));
        }


        Vector getForceDes () const {
          Vector res;
          res.resize(3);
          for(unsigned int i=0; i<3; i++)
            res(i) = fd_(i);

          return res;
        }

        void start();
        void stop();
        void save();
        void hold();

        void openGripper();
        void closeGripper();

        inline Eigen::Vector3d extractVector(const MatrixHomogeneous& m);
        inline Eigen::Matrix3d extractMatrix(const MatrixHomogeneous& m);
        inline MatrixHomogeneous buildfrom(Eigen::Vector3d& v, Eigen::Matrix3d& m);
        inline Vector fill(Eigen::Vector3d& v);

      protected:


      private:

        MatrixHomogeneous& computeControlOutput(MatrixHomogeneous& lw, const int& inTime);
        Vector& computeSelection(Vector& selec, const int& inTime);
        Vector& computePosture(Vector& q, const int& inTime);
        Vector& computeForce(Vector& force, const int& inTime);

        SignalPtr < ::dynamicgraph::Vector, int> forceSIN;
        SignalPtr < ::dynamicgraph::Vector, int> postureSIN;
        SignalPtr <MatrixHomogeneous, int> lwSIN;
        SignalPtr <MatrixHomogeneous, int> laSIN;
        SignalPtr <MatrixHomogeneous, int> raSIN;
        SignalPtr < ::dynamicgraph::Vector, int> velocitySIN;

        SignalTimeDependent <MatrixHomogeneous, int > lwSOUT;
        SignalTimeDependent <Vector, int> postureSOUT;
        SignalTimeDependent <Vector, int> forceSOUT;

        /// \brief Parameters of the Impedance controller
        double m_, c_;
        int mx_, cx_;
        double dy_, max_dx_, dist_, vel_fix_;
        Eigen::Vector3d xt_1_, xt_1_local_, xct_1_, xlat_1_, xlat_2_, xrat_1_, xrat_2_, xcft_1_, xcft_2_, xreft_1_, xreft_1_local_;
        int t_1_, tf_1_, elapsed_;
        Eigen::Vector3d fd_, f_ini_, ff_1_, ff_2_, fraw_;
        Eigen::Vector3d fRt_1_, fRt_2_, fR_, ff_, ft_;
        Eigen::Matrix3d pos_ini_, ;

        // Temporary variables for internal computations
        Vector q0_, qt_;
        Eigen::Vector3d xt_local_, fr_local_, xcf_world_, xw_local_, xw_;
        Eigen::Vector3d xd_local_;
        Eigen::Vector3d xt_, xg_, imp_, df_, xla_, xra_, fla_, fra_ ;
        Eigen::Vector3d fstatic_, vla_, vra_, xcf_, xlw_, xrot_, xini_;

        // Temporary useful variables for internal computations
        bool start_, stop_, hold_, init_, walk_, walkStop_, open_, close_;

#ifdef DEBUG
        std::ofstream wrist_, force_, res_, pos_, check_;
#endif

      }; // class ImpedanceController
    } // namespace tools
  } //namespace sot
} // namespace dynamicgraph

#endif //IMPEDANCE_CONTROLLER_HH
