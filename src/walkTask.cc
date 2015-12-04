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
        velocitySOUT(waistSIN, "walkTask("+inName+")::output(vector)::velocitydesOUT"),
        gain_(0.20), A_(0.0), B_(0.0), C_(0.0), max_vel_(0.1), posDes_(), e_(0.0), e_dot_(0.0), t_1_(0), iniTime_(std::tm())
      {
        // Register signals into the entity.
        signalRegistration (waistSIN);
        signalRegistration (velocitySOUT);

        posDes_.resize(3);
        posDes_.setZero();
        posDes_(0) = 1.5;
        posDes_(1) = 0.0;
        posDes_(2) = 0.0;

        e_.resize(3);    e_dot_.resize(3);
        e_.setZero();    e_dot_.setZero();

        double mag = posDes_.norm();
        gain_ = max_vel_/mag;

        iniTime_.tm_hour = 9;	iniTime_.tm_min = 50;	iniTime_.tm_sec = 0;
        iniTime_.tm_year = 115;	iniTime_.tm_mon = 11;	iniTime_.tm_mday = 1;
        // year counted from 1900

        pos_.open("/tmp/WaistPosition.txt", std::ios::out);
        vel_.open("/tmp/CommandedVelocity.txt", std::ios::out);

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

      }

      walkTask::~walkTask()
      {
        pos_.close();
        vel_.close();
      }

      Vector& walkTask::computeDesiredVel(Vector& veldes, const int& inTime)
      {
        const MatrixHomogeneous& Rwaist = waistSIN(inTime);
        MatrixRotation Rot, Rinv;
        Vector pos, erot, g;
        veldes.resize(3);   pos.resize(3);  erot.resize(3); g.resize(3);
        veldes.setZero();   pos.setZero();  erot.setZero(); g.setZero();
        Rwaist.extract(pos);
        Rwaist.extract(Rot);
        pos(2) = getYaw(Rwaist);

        pos_ << inTime;
        vel_ << inTime;
        for(unsigned int i=0; i<3; i++)
        {
          erot(i) = pos(i) - posDes_(i);    
          pos_ << " " << pos(i);
        }
        Rot.inverse(Rinv);
        Rinv.multiply(erot, e_);
        e_(2) = erot(2);

        double k = computeGain(); 
        g(0) = k;   g(1) = 3*k;   g(2) = 2*k;
        for(unsigned int i=0; i<3; i++)
        {
          e_dot_(i) = -g(i) * e_(i);
          if(fabs(e_(i)) < 0.001 || fabs(e_dot_(i) > 0.20) )
            e_dot_(i) = 0.0;
          vel_ << " " << e_dot_(i); // << " " << -g_e *erot(i);

          veldes(i) = e_dot_(i);
        }
               
        pos_ << std::endl;
        vel_ << "   " << k << std::endl;

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

