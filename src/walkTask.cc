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

#include <iostream>
#include <fstream>
#include <time.h>
#include <math.h>

//#define DEBUG

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
        gain_(0.20), A_(0.0), B_(0.0), C_(0.0), eyt_1_(0.0), t_1_(0), init_(false)
      {
        // Register signals into the entity.
        signalRegistration (waistSIN << rightFootSIN << leftFootSIN << velocitySOUT);

        velocitySOUT.addDependency( rightFootSIN  );
        velocitySOUT.addDependency( leftFootSIN  );
        velocitySOUT.addDependency( waistSIN  );

        posL_.resize(3);  posR_.resize(3);
        posL_.setZero();  posR_.setZero();

        posDes_.setZero(); pos_.setZero();  erot_.setZero();
        e_.setZero();    e_dot_.setZero();  max_vel_.setZero(); factor_.setZero();  tolerance_.setZero();   endVel_.setZero();

        max_vel_(0) = 0.1;  max_vel_(1) = 0.1;   max_vel_(2) = 0.08;
        factor_(0) = 1.0;   factor_(1) = 20.0;   factor_(2) = 5.0;
        tolerance_(0) = 0.05;   tolerance_(1) = 0.05;   tolerance_(2) = 0.09; // aprox. 5 deg
        endVel_(0) = 0.0001;    endVel_(1) = 0.0;   endVel_(2) = 0.0;

        double mag = posDes_.norm();
        gain_ = max_vel_(0)/mag;


#ifdef DEBUG
        pose_file_.open("/tmp/WaistPosition.txt", std::ios::out);
        vel_file_.open("/tmp/CommandedVelocity.txt", std::ios::out);
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
        pose_file_.close();
        vel_file_.close();
#endif
      }

      Vector& walkTask::computeDesiredVel(Vector& veldes, const int& inTime)
      {
        bool dataReadFeet = false ;
        bool dataReadFF = false ;
        veldes.resize(3);
        veldes.setZero();
        Eigen::Matrix3d Rot, Rinv;

        try
        {
          const MatrixHomogeneous & leftfoot = leftFootSIN(inTime);
          const MatrixHomogeneous & rightfoot = rightFootSIN(inTime);
          leftfoot.extract(posL_);
          Rot = extractMatrix(leftfoot);
          rightfoot.extract(posR_);
          pos_(2) = getYaw(leftfoot);
          pos_(0) = ( posL_(0) + posR_(0) )*0.5;
          pos_(1) = ( posL_(1) + posR_(1) )*0.5;
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
            pos_ = extractVector(Rwaist);
            Rot = extractMatrix(Rwaist);
            pos_(2) = getYaw(Rwaist);
            dataReadFF=true;
          }catch(...)
          {
            dataReadFF=false;
          }
        }

        if(!dataReadFeet && !dataReadFF)
        {
          assert(! (!dataReadFeet && !dataReadFF) );
          std::cout << "problem in walk task : no signal in input " << std::endl ;
          veldes.setZero();
          return veldes;
        }

#ifdef DEBUG
        pose_file_ << inTime;
        vel_file_ << inTime;
#endif
        for(unsigned int i=0; i<3; i++)
        {
          erot_(i) = pos_(i) - posDes_(i);
#ifdef DEBUG
          pose_file_ << " " << pos_(i);
          pose_file_ << " " << posDes_(i);
#endif
        }
        Rinv = Rot.inverse();
        e_ = Rinv * erot_;
        e_(2) = erot_(2);
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
          else
          {
            veldes(i) = e_dot_(i);
          }
#ifdef DEBUG
          vel_file_ << " " << veldes(i);
          pose_file_ << " " << e_(i);
#endif
        }

        if( fabs(e_dot_(0)) + fabs(e_dot_(1)) + fabs(e_dot_(2)) < 0.0001)
        {
          veldes(0) = 0.0001 ;
          veldes(1) = 0.0    ;
          veldes(2) = 0.0    ;
        }

        eyt_1_ = e_(1);
#ifdef DEBUG
        pose_file_ << std::endl;
        vel_file_ << "   " << g << std::endl;
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
        double yaw;
        yaw = atan2( Rw(1,0), Rw(0,0));
        return yaw;
      }

      inline Eigen::Vector3d walkTask::extractVector(const MatrixHomogeneous& m)
      {
        Eigen::Vector3d v;
        for(int i=0; i<v.size(); i++)
            v(i) = m(i, 3);

        return v;
      }

      inline Eigen::Matrix3d walkTask::extractMatrix(const MatrixHomogeneous& m)
      {
        Eigen::Matrix3d res;
        res.resize(3,3);
        for(int i=0; i<res.rows(); i++)
        {
          for(int j=0; j<res.cols(); j++)
            res(i, j) = m(i, j);
        }

        return res;
      }

      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(walkTask, "walkTask");
    } // namespace tools
  } //namespace sot
} // namespace dynamicgraph

