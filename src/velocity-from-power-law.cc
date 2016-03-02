//
// Copyright (C) 2012 LAAS-CNRS
//
// Author: Florent Lamiraux, 
//         Mehdi Benallegue <mehdi@benallegue.com>
//

#include "velocity-from-power-law.hh"
#include <dynamic-graph/command-bind.h>

namespace dynamicgraph {
  namespace sot {
    using command::makeDirectSetter;
    using command::docDirectSetter;
    using command::docDirectGetter;
    using command::makeDirectGetter;
    namespace tools {
      VelocityFromPowerLaw::VelocityFromPowerLaw (const std::string name,
                                                  double radiusx,
                                                  double radiusy,
                                                  double gamma,
                                                  double beta,
                                                  double zerokapparadius):
        Entity(name),
        vectorSoutSOUT_( boost::bind(&VelocityFromPowerLaw::computeVelocitySignal,this,_1,_2),
                         sotNOSIGNAL,"VelocityFromPowerLaw("+name+")::output(vector)::velocity" ),
        LeftFootCurrentPosSIN(NULL,"VelocityFromPowerLaw("+name+")::input(homogeneousmatrix)::leftfootcurrentpos"),
        RightFootCurrentPosSIN(NULL,"VelocityFromPowerLaw("+name+")::input(homogeneousmatrix)::rightfootcurrentpos"),
        WaistCurrentPosSIN(NULL,"VelocityFromPowerLaw("+name+")::input(vector)::waist"),
        hp_(radiusx,radiusy,gamma,beta,zerokapparadius),
        powerLawGeneration_(this,hp_)
      {
        vectorSoutSOUT_.addDependency(LeftFootCurrentPosSIN );
        vectorSoutSOUT_.addDependency(RightFootCurrentPosSIN);
        vectorSoutSOUT_.addDependency(WaistCurrentPosSIN    );
        signalRegistration(vectorSoutSOUT_);

        signalRegistration(LeftFootCurrentPosSIN <<
                           RightFootCurrentPosSIN <<
                           WaistCurrentPosSIN);
	
        initializeCommand();

	previousValues_.resize(5,0.0);
      }

      VelocityFromPowerLaw::VelocityFromPowerLaw (const std::string name):
        Entity(name),
        vectorSoutSOUT_( boost::bind(&VelocityFromPowerLaw::computeVelocitySignal,this,_1,_2),
                         sotNOSIGNAL,"VelocityFromPowerLaw("+name+")::output(vector)::velocity" ),
        LeftFootCurrentPosSIN(NULL,"VelocityFromPowerLaw("+name+")::input(homogeneousmatrix)::leftfootcurrentpos"),
        RightFootCurrentPosSIN(NULL,"VelocityFromPowerLaw("+name+")::input(homogeneousmatrix)::rightfootcurrentpos"),
        WaistCurrentPosSIN(NULL,"VelocityFromPowerLaw("+name+")::input(vector)::waist"),
        hp_(0.0, 0.0, 0.0, 0.0, 0.0),
        // HopfParameters(radiusx, radiusy, gamma, beta, zerokapparadius)
        powerLawGeneration_(this,hp_)
      {
        vectorSoutSOUT_.addDependency(LeftFootCurrentPosSIN );
        vectorSoutSOUT_.addDependency(RightFootCurrentPosSIN);
        vectorSoutSOUT_.addDependency(WaistCurrentPosSIN    );
        signalRegistration(vectorSoutSOUT_);

        signalRegistration(LeftFootCurrentPosSIN <<
                           RightFootCurrentPosSIN <<
                           WaistCurrentPosSIN);

        initializeCommand();

	previousValues_.resize(5,0.0);
      }

      void VelocityFromPowerLaw::initializeCommand()
      {
        std::string docstring = "    \n"
        "    Set ellipse parameters\n"
        "      Input:\n"
        "        - a floating point number: the X half axe,\n"
        "        - a floating point number: the Y half axe,\n"
        "        - a floating point number: the gamma of the power law,\n"
        "        - a floating point number: the beta of the power law,\n"
        "    \n";
        addCommand("initializePowerLaw",
           dynamicgraph::command::makeCommandVoid4(
                     *this,&VelocityFromPowerLaw::initializeEllipse,docstring)
                   );
      }

      void VelocityFromPowerLaw::initializeEllipse (const double& radiusx,
                                                    const double& radiusy,
                                                    const double& gamma,
                                                    const double& beta)
      {
        hp_.~HopfParameters();
        hp_ = HopfParameters(radiusx,
                           radiusy,
                           gamma,
                           beta,
                           2.0);
//        std::cout << "radiusx ; " << hp_.radiusx_ << "   "
//             << "radiusy ; " << hp_.radiusy_ << "   "
//             << "gamma ; " << hp_.gamma_ << "   "
//             << "beta ; " << hp_.beta_ << "   " << std::endl ;
        //hp_.save();
        powerLawGeneration_.~PowerLawGeneration();
        powerLawGeneration_ = PowerLawGeneration(this,hp_);

        return ;
      }

      dynamicgraph::Vector& VelocityFromPowerLaw::computeVelocitySignal (
          dynamicgraph::Vector& vsout, const int& time)
      {
        vsout.resize(3,true) ;
        MatrixHomogeneous rf ;
        MatrixHomogeneous lf ;
        dynamicgraph::Vector waist;
        Eigen::Vector3d velocity ;
        try{
          rf=RightFootCurrentPosSIN(time);
          lf=LeftFootCurrentPosSIN(time);
          waist=WaistCurrentPosSIN(time);
//          std::cout << "lfx = " << lf(0,3) << std::endl;
//          std::cout << "lfy = " << lf(1,3) << std::endl;
//          std::cout << "rfx = " << rf(0,3) << std::endl;
//          std::cout << "rfy = " << rf(1,3) << std::endl;
//          std::cout << "ctheta = " << waist(5) << std::endl;
	  double lfx(0.0), lfy(0.0), rfx(0.0), rfy(0.0), theta(0.0), thresh(10.0);
	  if(std::abs(lf(0,3))>thresh || std::abs(lf(1,3))>thresh || std::abs(rf(0,3))>thresh || std::abs(rf(1,3))>thresh)
	    {
	      lfx = previousValues_[0];
	      lfy = previousValues_[1];
	      rfx = previousValues_[2];
	      rfy = previousValues_[3];
	      theta = previousValues_[4];
	    }
	  else
	    {
	      lfx = lf(0,3);
	      lfy = lf(1,3);
	      rfx = rf(0,3);
	      rfy = rf(1,3);
	      theta = waist(5);
	      previousValues_.resize(5,0.0);
	      previousValues_[0] = lf(0,3);
	      previousValues_[1] = lf(1,3);
	      previousValues_[2] = rf(0,3);
	      previousValues_[3] = rf(1,3);
	      previousValues_[4] = waist(5);
	    }	  
	  
        velocity = powerLawGeneration_.generateVelocityFromPowerLawVectorField(
                time*0.005,theta,
                lfx,lfy,
                rfx,rfy);
        }catch(...)
        {
          velocity << 0.0001 , 0.0 , 0.0 ;
        }

        for (unsigned i=0 ; i<3 ; ++i )
        {
          if(std::abs(velocity(i))<0.0001)
            vsout(i)=0.0001;
          else
            vsout(i)=velocity(i);
        }

        return vsout;
      }

      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(VelocityFromPowerLaw, "VelocityFromPowerLaw");

    } // namespace tools
  } // namespace sot
} // namespace dynamicgraph
