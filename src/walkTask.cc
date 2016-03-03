/*
 *  Copyright 2015 CNRS
 *
 *
 */

#include <vector>
#include <stdexcept>
#include <boost/tokenizer.hpp>
#include <iostream>
#include <dynamic-graph/command-bind.h>

#include "walkTask.hh"
#include <dynamic-graph/factory.h>
#include <dynamic-graph/command-setter.h>
#include <dynamic-graph/command-getter.h>
#include <sot/core/vector-roll-pitch-yaw.hh>

#include <iostream>
#include <fstream>
#include <time.h>
#include <math.h>

#define DEBUG

namespace dynamicgraph
{
  namespace sot
  {
    namespace tools
    {
      using dynamicgraph::Entity;
      using dynamicgraph::command::makeCommandVoid0;
      using dynamicgraph::command::docCommandVoid0;


      //DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(walkTask, "walkTask");

      walkTask::walkTask(const std::string& inName) :
        Entity(inName),
        waistSIN(0, "walkTask("+inName+")::input(MatrixHomo)::waistIN"),
        rightFootSIN(0, "walkTask("+inName+")::input(MatrixHomo)::rightfootSIN"),
        leftFootSIN(0, "walkTask("+inName+")::input(MatrixHomo)::leftfootSIN"),
        velocitySOUT(waistSIN, "walkTask("+inName+")::output(vector)::velocitydesOUT"),
        gain_(0.20), A_(0.0), B_(0.0), C_(0.0), eyt_1_(0.0), posDes_(), e_(0.0), e_dot_(0.0), t_1_(0), init_(false), iniTime_(std::tm())
      {
        // Register signals into the entity.
        signalRegistration (waistSIN);
        signalRegistration (rightFootSIN);
        signalRegistration (leftFootSIN);
        signalRegistration (velocitySOUT);

        velocitySOUT.addDependency( rightFootSIN  );
        velocitySOUT.addDependency( leftFootSIN  );
        velocitySOUT.addDependency( waistSIN  );

        posDes_.resize(3);
        posDes_.setZero();
        posDes_(0) = 0.0;
        posDes_(1) = 0.0;
        posDes_(2) = 0.0;

        e_.resize(3);    e_dot_.resize(3);  max_vel_.resize(3); factor_.resize(3);  tolerance_.resize(3);   endVel_.resize(3);
        e_.setZero();    e_dot_.setZero();  max_vel_.setZero(); factor_.setZero();  tolerance_.setZero();   endVel_.setZero();

        max_vel_(0) = 0.1;  max_vel_(1) = 0.1;   max_vel_(2) = 0.08;
        factor_(0) = 1.0;   factor_(1) = 20.0;   factor_(2) = 5.0;
        tolerance_(0) = 0.05;   tolerance_(1) = 0.05;   tolerance_(2) = 0.09; // aprox. 5 deg
        endVel_(0) = 0.0001;    endVel_(1) = 0.0;   endVel_(2) = 0.0;

        double mag = posDes_.norm();
        gain_ = max_vel_(0)/mag;

        iniTime_.tm_hour = 9;	iniTime_.tm_min = 50;	iniTime_.tm_sec = 0;
        iniTime_.tm_year = 115;	iniTime_.tm_mon = 11;	iniTime_.tm_mday = 1;
        // year counted from 1900
#ifdef DEBUG
        pos_.open("/tmp/WaistPosition.txt", std::ios::out);
        vel_.open("/tmp/CommandedVelocity.txt", std::ios::out);
#endif
        velocitySOUT.setFunction (boost::bind(&walkTask::computeDesiredVel, this, _1, _2));

        std::string docstring;
        // setGainConstant
        docstring =
            "\n"
            "    Set the gain for the walking task\n"
            "      takes a double number as input\n"
            "\n";
        addCommand(std::string("setGainConstant"),
                   new ::dynamicgraph::command::Setter<walkTask, double>
                   (*this, &walkTask::setGainConstant, docstring));

        // setAdaptiveGain
        docstring =
            "\n"
            "    Set an adaptive gain for the walking task\n"
            "    set with a passing point, max vel allowed = 0.1\n"
            "      Input:\n"
            "        floating point value: value at 0\n"
            "    \n";
        addCommand(std::string("setAdaptiveGain"),
                   new ::dynamicgraph::command::Setter<walkTask, double>
                   (*this, &walkTask::setAdaptiveGain, docstring));

        // setGoalPosition
        docstring =
            "\n"
            "    Set the Goal Position for the waist\n"
            "      takes a tuple of 3 floating point numbers as input\n"
            "\n";
        addCommand(std::string("setGoalPosition"),
                   new ::dynamicgraph::command::Setter<walkTask, Vector>
                   (*this, &walkTask::setGoalPosition, docstring));

        // setErrorTolerance
        docstring =
            "\n"
            "    Set the Error Tolerance for the Walking Task\n"
            "      takes a tuple of 3 floating point numbers as input\n"
            "\n";
        addCommand(std::string("setErrorTolerance"),
                   new ::dynamicgraph::command::Setter<walkTask, Vector>
                   (*this, &walkTask::setErrorTolerance, docstring));


      }

      walkTask::~walkTask()
      {
#ifdef DEBUG
        pos_.close();
        vel_.close();
#endif
      }

      Vector& walkTask::computeDesiredVel(Vector& veldes, const int& inTime)
      {
        bool dataReadFeet = false ;
        bool dataReadFF = false ;
        MatrixRotation Rot, Rinv;
        Vector pos, erot;
        veldes.resize(3);   pos.resize(3);  erot.resize(3);
        veldes.setZero();   pos.setZero();  erot.setZero();

        try
        {
          const MatrixHomogeneous & leftfoot = leftFootSIN(inTime);
          const MatrixHomogeneous & rightfoot = rightFootSIN(inTime);
          Vector posL,posR;
          posL.resize(3);  posR.resize(3);
          posL.setZero();  posR.setZero();
          leftfoot.extract(posL);
          leftfoot.extract(Rot);
          rightfoot.extract(posR);
          pos(2) = getYaw(leftfoot);
          pos(0) = ( posL(0) + posR(0) )*0.5;
          pos(1) = ( posL(1) + posR(1) )*0.5;
          dataReadFeet=true;
        }catch(...)
        {
          dataReadFeet=false;
        }

        if(!dataReadFeet)
        {
          try
          {
            const MatrixHomogeneous & Rwaist = waistSIN(inTime);
            Rwaist.extract(pos);
            Rwaist.extract(Rot);
            pos(2) = getYaw(Rwaist);
            dataReadFF=true;
          }catch(...)
          {
            dataReadFF=false;
          }
        }

        if(!dataReadFeet && !dataReadFF)
        {
          std::cout << "problem in walk task : no signal in input " << std::endl ;
          veldes.setZero();
          return veldes;
        }

#ifdef DEBUG
        pos_ << inTime;
        vel_ << inTime;
#endif
        for(unsigned int i=0; i<3; i++)
        {
          erot(i) = pos(i) - posDes_(i);
#ifdef DEBUG
          pos_ << " " << pos(i);
          pos_ << " " << posDes_(i);
#endif
        }
        Rot.inverse(Rinv);
        Rinv.multiply(erot, e_);
        e_(2) = erot(2);
        double g = computeGain();
        for(unsigned int i=0; i<3; i++)
        {
          if(i==1)
            e_dot_(i) = -factor_(i)*g * e_(i) - factor_(i)*g * (0.005/2)*(e_(i) + eyt_1_);
          else
            e_dot_(i) = -factor_(i)*g * e_(i);

          if( ( (fabs(e_(i)) < tolerance_(i)) && (e_.norm() < 0.12) ) || isnan(e_dot_(i)))
          {
            veldes(i) = endVel_(i);
          }
          else if(fabs(e_dot_(i)) > max_vel_(i))
          {
            if(e_dot_(i)>0.0)
            {
              veldes(i) = max_vel_(i) ;
            }
            else
            {
              veldes(i) = -max_vel_(i) ;
            }
          }
          else if( fabs(e_dot_(i)) < 0.0001)
          {
            veldes(i) = 0.0001 ;
          }
          else
          {
            veldes(i) = e_dot_(i);
          }
#ifdef DEBUG
          vel_ << " " << veldes(i);
          pos_ << " " << e_(i);
#endif
        }

        eyt_1_ = e_(1);
#ifdef DEBUG
        pos_ << std::endl;
        vel_ << "   " << g << std::endl;
#endif
        return veldes;
      }

      double walkTask::computeGain()
      {
        double res=0.0, enorm = e_.norm();
        if(A_ == 0)
          res = gain_;
        else
        {
          // Usually use the magnitude of the error, for now just using X error
          res = A_ * exp( -B_*enorm ) + C_;
        }

        return res;
      }

      double walkTask::getYaw(const MatrixHomogeneous& Rw)
      {
        MatrixRotation Rot;
        VectorRollPitchYaw rpy;

        Rw.extract(Rot);
        rpy.fromMatrix(Rot);

        return rpy(2);
      }

      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(walkTask, "walkTask");
    } // namespace tools
  } //namespace sot
} // namespace dynamicgraph

