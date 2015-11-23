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
	  fd_ = inForce;
        }


        Vector getForceDes () const {
  	  return fd_;
        }

	void start();
	void stop();
	void save();
	void hold();

      protected:


      private:

        MatrixHomogeneous& computeControlOutput(MatrixHomogeneous& lw, const int& inTime);
	Vector& computeSelection(Vector& selec, const int& inTime);
        Vector& computePosture(Vector& q, const int& inTime);
        Vector& computeForce(Vector& force, const int& inTime);

        SignalPtr < ::dynamicgraph::Vector, int> forceSIN;
	SignalPtr < ::dynamicgraph::Vector, int> postureSIN;
	SignalPtr < ::dynamicgraph::Vector, int> velocitySIN;   
	SignalPtr <MatrixHomogeneous, int> lwSIN;
	SignalPtr <MatrixHomogeneous, int> laSIN;
	SignalPtr <MatrixHomogeneous, int> raSIN;
        //SignalTimeDependent <MatrixHomogeneous, int> lwSIN;

        SignalTimeDependent <MatrixHomogeneous, int > lwSOUT;
	SignalTimeDependent < Vector, int> gripSOUT;    
	SignalTimeDependent <Vector, int> postureSOUT; 
	SignalTimeDependent <Vector, int> forceSOUT;

      /// \brief Parameters of the Impedance controller
        double m_, c_;
		int mx_, cx_;
        double dy_, max_dx_;
        Vector xt_1_, q0_, xct_1_, xlat_1_, xlat_2_, xrat_1_, xrat_2_, xcft_1_, xcft_2_, xreft_1_;
        int t_1_, tf_1_, elapsed_;        
	Vector fd_, ff_1_, ff_2_, fraw_;
	Vector fRt_1_, fRt_2_;
	MatrixHomogeneous lw_initial_, lwct_1_, pos_ini_;

      // Temporary variables for internal computations
        MatrixRotation Rot_;
        bool start_, stop_, hold_, init_, walk_, walkStop_;
	std::ofstream wrist_, force_, res_, pos_, check_; 

	//For realtime
	struct tm iniTime_;

      }; // class ImpedanceController
    } // namespace tools
  } //namespace sot
} // namespace dynamicgraph

#endif //IMPEDANCE_CONTROLLER_HH
