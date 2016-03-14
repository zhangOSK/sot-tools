/*
 *  Copyright 2014 CNRS
 *
 *
 */

#include <vector>
#include <stdexcept>
#include <boost/tokenizer.hpp>
#include <iostream>
#include <dynamic-graph/command-bind.h>

#include "impController.hh"
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


      //DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ImpedanceController, "ImpedanceController");

      ImpedanceController::ImpedanceController(const std::string& inName) :
        Entity(inName),
        forceSIN(0, "ImpedanceController("+inName+")::input(vector)::force"),
        postureSIN(0, "ImpedanceController("+inName+")::input(vector)::postureIN"),
        lwSIN(0, "ImpedanceController("+inName+")::input(MatrixHomo)::leftWristIN"),
        laSIN(0, "ImpedanceController("+inName+")::input(MatrixHomo)::leftAnkleIN"),
        raSIN(0, "ImpedanceController("+inName+")::input(MatrixHomo)::rightAnkleIN"),
        velocitySIN(0, "ImpedanceController("+inName+")::input(vector)::velocityIN"),
        lwSOUT(forceSIN << lwSIN << laSIN, "ImpedanceController("+inName+")::output(MatrixHomo)::leftWristOUT"),
        postureSOUT(forceSIN << postureSIN, "ImpedanceController("+inName+")::output(vector)::postureOUT"),
        forceSOUT(forceSIN << lwSIN, "ImpedanceController("+inName+")::output(vector)::leftWristForce"),
        m_(1.00), c_(5.0), mx_(20), cx_(100), max_dx_(0.005), dist_(0.0), vel_fix_(0.0), t_1_(0), tf_1_(0), elapsed_(0),
        start_(false), stop_(false), hold_(false), init_(false), walk_(false), walkStop_(false), open_(false), close_(false)
      {
        // Register signals into the entity.
        signalRegistration (forceSIN);
        signalRegistration (postureSOUT);
        signalRegistration (lwSIN);
        signalRegistration (laSIN);
        signalRegistration (raSIN);
        signalRegistration (velocitySIN);
        signalRegistration (lwSOUT);
        signalRegistration (postureSIN);
        signalRegistration (forceSOUT);

        

        fraw_.setZero();    ff_1_.setZero();	ff_2_.setZero();    xt_1_local_.setZero();
        xcft_1_.setZero();  fRt_1_.setZero();	fRt_2_.setZero();	xcft_2_.setZero();
        xreft_1_.setZero(); fd_.setZero();      xct_1_.setZero();	xlat_1_.setZero();
        xlat_2_.setZero();	xrat_1_.setZero();  xrat_2_.setZero();  xreft_1_local_.setZero();

        // longer hose (1.25 times longHose) = 11.34 -> 8.84 (MLJ), longest Hose (1.5 times longHose) = 13.64 -> 10.64 (MLJ)
        double massHose = 7.04; //full hose: 9.04 -> 7.04 (massless joints MLJ), heavy = 13.56, light = 4.52 (50% fullHose), semi-light = 6.78 (75% of fullHose),  coiled hose: 13.197
        double part = 0.32;   // 0.32 for longHose // 0.3 for heavy-longHose //Hold part % of the total weight of the Hose
        //for longer and longest Hose CAREFUL!!-holding weight in Z does not changes with length!!!
        double mu = 0.5;
        double gx = -9.8;
        fd_(0) = mu * (0.4 * massHose) * gx;
        fd_(1) = 0.0;
        fd_(2) = (part * massHose * gx) - 11.0;   //Sensor offset = -11.0

//        WAIST
//        1	-7.03457e-10	6.28389e-06	2.0636e-06
//        7.01005e-10    	1	3.9016e-07	2.83508e-07
//        -6.28389e-06	-3.9016e-07	1	0.648703
//        0	0	0	1

        // pos_ini_ should be the position of the left wrist in half sitting for hrp2
        //LEFT-WRIST
        pos_ini_ = Eigen::MatrixXd::Zero(3, 3);
        pos_ini_(0,0) = 0.963962  ; pos_ini_(0,1) =0.0449441; pos_ini_(0,2) =-0.262218 ; //pos_ini_(0,3) =0.0418365;
        pos_ini_(1,0) = -0.0868231; pos_ini_(1,1) =0.984808 ; pos_ini_(1,2) =-0.150382 ; //pos_ini_(1,3) =0.331007 ;
        pos_ini_(2,0) = 0.251475  ; pos_ini_(2,1) =0.167729 ; pos_ini_(2,2) =0.953219  ; //pos_ini_(2,3) =0.704286 ;
        //pos_ini_(3,0) = 0	      ; pos_ini_(3,1) =0	    ; pos_ini_(3,2) =0	       ; pos_ini_(3,3) =1        ;

        xini_(0) = 0.0418365;   xini_(1) = 0.331007 ;   xini_(2) = 0.704286 ;


#ifdef DEBUG
        wrist_.open("/tmp/WristPos.txt", std::ios::out);
        force_.open("/tmp/Force.txt", std::ios::out);
        res_.open("/tmp/ControllerOutput.txt", std::ios::out);
        check_.open("/tmp/controllerCalcs.txt", std::ios::out);
        //	pos_.open("/tmp/Controller.posture", std::ios::out);
        pos_.open("/tmp/Impedance.txt", std::ios::out);

        res_ << "----> Mass Hose: " << massHose << std::endl;
        res_ << "----> Fd: " << fd_ << std::endl;
        res_ << "----> Controller: m = " << m_ << ",  c = " << c_ << ", times m: " << mx_ << ", times c: " << cx_ << std::endl;
#endif


        lwSOUT.setFunction (boost::bind(&ImpedanceController::computeControlOutput, this, _1, _2));

        postureSOUT.setFunction (boost::bind(&ImpedanceController::computePosture, this, _1, _2));

        forceSOUT.setFunction (boost::bind(&ImpedanceController::computeForce, this, _1, _2));


        std::string docstring;
        // setMass
        docstring =
            "\n"
            "    Set the mass of the impedance controller\n"
            "      takes a double number as input\n"
            "\n";
        addCommand(std::string("setMass"),
                   new ::dynamicgraph::command::Setter<ImpedanceController, double>
                   (*this, &ImpedanceController::setMass, docstring));

        // getMass
        docstring =
            "\n"
            "    Get the mass of the impedance controller\n"
            "      returns a double number\n"
            "\n";
        addCommand(std::string("getMass"),
                   new ::dynamicgraph::command::Getter<ImpedanceController, double>
                   (*this, &ImpedanceController::getMass, docstring));

        // setDamping
        docstring =
            "\n"
            "    Set the damping of the impedance controller\n"
            "      takes a double number as input\n"
            "\n";
        addCommand(std::string("setDamping"),
                   new ::dynamicgraph::command::Setter<ImpedanceController, double>
                   (*this, &ImpedanceController::setDamping, docstring));

        // getDamping
        docstring =
            "\n"
            "    Get the damping of the impedance controller\n"
            "      returns a double number\n"
            "\n";
        addCommand(std::string("getDamping"),
                   new ::dynamicgraph::command::Getter<ImpedanceController, double>
                   (*this, &ImpedanceController::getDamping, docstring));

        // setForceDes
        docstring =
            "\n"
            "    Set the desired force of the impedance controller\n"
            "      takes a tuple of 3 floating point numbers as input\n"
            "\n";
        addCommand(std::string("setForceDes"),
                   new ::dynamicgraph::command::Setter<ImpedanceController, Vector>
                   (*this, &ImpedanceController::setForceDes, docstring));

        // getForceDes
        docstring =
            "\n"
            "    Get the desired force of the impedance controller\n"
            "      returns a double number\n"
            "\n";
        addCommand(std::string("getForceDes"),
                   new ::dynamicgraph::command::Getter<ImpedanceController, Vector>
                   (*this, &ImpedanceController::getForceDes, docstring));

        addCommand (std::string("start"),
                    makeCommandVoid0 (*this, &ImpedanceController::start,
                                      docCommandVoid0 ("Start Controller")));

        addCommand (std::string("stop"),
                    makeCommandVoid0 (*this, &ImpedanceController::stop,
                                      docCommandVoid0 ("Stop Controller")));

        addCommand (std::string("save"),
                    makeCommandVoid0 (*this, &ImpedanceController::save,
                                      docCommandVoid0 ("Save data in file")));

        addCommand (std::string("openGripper"),
                    makeCommandVoid0 (*this, &ImpedanceController::openGripper,
                                      docCommandVoid0 ("Open both grippers")));

        addCommand (std::string("closeGripper"),
                    makeCommandVoid0 (*this, &ImpedanceController::closeGripper,
                                      docCommandVoid0 ("Close both grippers")));
      }

      ImpedanceController::~ImpedanceController()
      {
      }

      MatrixHomogeneous& ImpedanceController::computeControlOutput(MatrixHomogeneous& lw, const int& inTime)
      {
        const MatrixHomogeneous& R = lwSIN(inTime);
        const MatrixHomogeneous& la = laSIN(inTime);
        const MatrixHomogeneous& ra = raSIN(inTime);
        const Vector& fr = forceSOUT.access(inTime);
        const Vector& qs = postureSIN(inTime);
        const Vector& vel = velocitySIN(inTime);
        const double& dt = 0.005; //inTime - t_1_;
        vla_.setZero();
        vra_.setZero();
        xcf_.setZero();
        xt_.setZero();
        xrot_.setZero();
        xlw_.setZero();
        xini_.setZero();
        fstatic_.setZero();

        Eigen::MatrixXd Ryaw, Rlw, Rrot, Rinv;
        Eigen::Vector3d flw = Eigen::Vector3d( fr(0), fr(1), fr(2) );

        Ryaw = extractMatrix(la);
        Rinv = Ryaw.inverse();
        
        if(!start_ || stop_ || hold_)
        {
          xrot_ = Ryaw * xini_;

          if(!init_)
          {
            // Save the distance from wrist to the  waist at the starting position
            dy_ = R(1, 3)-qs(1);
            xreft_1_ = extractVector(R);

            init_ = true;
            //f_ini_ = Eigen::Vector3d(fr(0), fr(1), fr(2));
#ifdef DEBUG
            res_ << "----->>> dy = " << dy_ << std::endl;
           // res_ << "===>>> fr ini = " << f_ini_ << std::endl;
#endif
          }

          xt_ = extractVector(R);
          xt_1_local_  = Rinv * xt_;

          xt_1_ = extractVector(R);
          xct_1_ = xt_1_;
          xlat_1_ = extractVector(la);
          xlat_2_ = xlat_1_;
          xrat_1_ = extractVector(ra);
          xrat_2_ = xrat_1_;
          xcft_1_ = xct_1_;
          xcft_2_ = xct_1_;
          
          lw = buildfrom(xini_, pos_ini_);
         
          lw(0,3) = qs(0)+xrot_(0);
          lw(1,3) = qs(1)+xrot_(1);
          lw(2,3) = qs(2)-0.648703+xini_(2);
          //f_ini_ = fr;

          if(hold_)
          {
            lw(0, 3) = (3*R(0,3) + (qs(0) +xrot_(0)) + 2*xreft_1_(0))/6;
            lw(1, 3) = ( (qs(1)+xrot_(1)) + 3*R(1,3) + 2*xreft_1_(1))/6;
            lw(2, 3) = (3*R(2,3) + (qs(2)-0.648703+xini_(2)) + 2*xreft_1_(2))/6;
#ifdef DEBUG
            res_ << "**" << inTime << "	" << lw << std::endl;
            wrist_ << inTime;

            for(unsigned k=0; k < 3; ++k)
              wrist_ << "	" << R(k, 3) << "	" << lw(k, 3) << "	0.001" ;

            wrist_ << "	" << la(0, 3) << "	" << ra(0, 3) << "	" << la(1, 3) << "	" << ra(1, 3);
            wrist_ << "	" << la(2, 3) << "	" << ra(2, 3) << "	0.001	0.001	" << realTime << std::endl;
#endif

            xt_1_ = extractVector(R);
            xreft_1_ = extractVector(lw);

          }

          if(stop_)
          {
            lw = R;
            lw(0,3) = ( (qs(0)+xrot_(0)) + 3*R(0,3) + 2*xreft_1_(0))/6;
            lw(1,3) = ( (qs(1)+xrot_(1)) + 3*R(1,3) + 2*xreft_1_(1))/6;
            lw(2,3) = ( (qs(2)-0.648703+xini_(2)) + 3*R(2,3) + 2*xreft_1_(2))/6;
#ifdef DEBUG
            res_ << "~~~~ stopped = " << inTime << "    " << lw << std::endl;
#endif
          }  

          xlw_= extractVector(lw);
          xreft_1_local_ = Rinv * xlw_;
        }
        else if (start_)
        {
          lw = R;

          for(unsigned i=0; i < 3; i++)
          {
            xt_(i) = R(i, 3);
            xla_(i) = la(i, 3);
            xra_(i) = ra(i, 3);
          }

          int over = 0;
          for(unsigned i=0; i < 3; i++)
          {
            if( (vel(0) != 0.0) && (fabs(fraw_(i)) > 400.0) )
              over = over + 1;
          }

          if (over > 2)
          {
            lw = R;
#ifdef DEBUG
            res_ << "fr = " << fr << ",  fraw = " << fraw_ << std::endl;
#endif
            hold();
          }

          ///Update desired force depending on distance to the reel
         //----Experiment----------------------------------------------
 /*         dist_ = xt(0) - xini_(0);
          double massHose = dist_*1.8;      //Real hose mass = 1.8 kg/m
          fd_(0) = f_ini_(0) + (0.5 * massHose * (-9.8));
          if(dist_ < 1.5)
            fd_(2) = f_ini_(2) + (0.5 * massHose * (-9.8));
          else
            fd_(2) = f_ini_(2) + (1.8 * (-9.8));
          //--------------------------------------------------

          //---Simulation-------------------------------
          //dist_ = (xt(0) - xini_(0))+2.0;
          massHose = dist_*1.8;       
          fd_(0) = f_ini_(0) + (0.5 * (0.15 * massHose) * (-9.8));
          fd_(2) = f_ini_(2) + (0.05 * massHose * (-9.8) );
          //----------------------------------------------------
//*/

          for(unsigned int i=0; i<3; i++)
          {
            vla_(i) = ( xla_(i) - xlat_1_(i) )/dt;
            vra_(i) = ( xra_(i) - xrat_1_(i) )/dt;
          }


          //if( ((fla_(0) > 50.0) || (fra_(0) > 50.0)) && !walk_)
          bool floor = ( fabs(xla_(2) < 0.10505) && fabs(xra_(2) < 0.10505) );
          if( ((vla_(2) > 0.1) || (vra_(2) > 0.1)) && ((xla_(2) > 0.106) || (xra_(2) > 0.106)) && !walk_)
          {
            walk_ = true;
            walkStop_ = false;
#ifdef DEBUG
            res_ << "======> Started walking at : " << inTime*0.005 << ", realtime: " << realTime << std::endl;
#endif
          }
          else if( ( (((fabs(vla_(2))) < 0.0001)&&((fabs(vra_(2))) < 0.0001)) || ((fabs(xla_(0)-xra_(0)) < 0.00001)&&(fabs(vla_(0)) < 0.001)) ) && walk_ && floor && (vel(0) ==0.0) )
          {
            walkStop_ = true;
            walk_ = false;
            elapsed_ = 0;
#ifdef DEBUG
            res_ << "####### Finish walking at : " << inTime*0.005<< ", realtime: " << realTime << std::endl;
#endif
          }

          if( (xla_(2) < 0.1051) && (xra_(2) < 0.1051) && walk_ && !walkStop_)
          {
            if(elapsed_ == 0)
              vel_fix_ = vel(0);

            fstatic_(0) = vel_fix_*(27000);		//2700
            elapsed_ = elapsed_ + 1;
          }
          else
          {
            elapsed_ = 0;
            fstatic_.setZero();
          }

          fr_local_.setZero();  xt_local_.setZero();
          xcf_world_.setZero(); xw_.setZero(); xw_local_.setZero();
          fr_local_ = Rinv * flw;    //          Rinv.multiply(fr, fr_local_);
          xt_local_ = Rinv * xt_;      //  Rinv.multiply(xt_, xt_local_);
          xw_(0) = qs(0);    xw_(1) = qs(1);  xw_(2) = qs(2);
          xw_local_ = Rinv * xw_;

          fr_local_(1) = 0.0;
          
          xg_ = (((dt * dt) / m_ ) * (fr_local_ - (fd_ - fstatic_) - ((c_/dt) * (xt_local_ - xt_1_local_)) )) + (2 * xt_local_) - xt_1_local_;
          vra_ = (xg_ + (2*xcft_1_) + xcft_2_)/4;
          xct_1_ = xg_;
          xcft_2_ = xcft_1_;
          xcft_1_ = vra_;

          xd_local_.setZero();  
          for(unsigned i=0; i < 3; i++)
            xd_local_(i) = (vra_(i) + xreft_1_local_(i) + xt_local_(i))/3;

          //In Y, keep the distance at starting position with the waist, always!!
          xd_local_(1) = dy_ + xw_local_(1);

          //Check limits on the local (waist) frame
          if( xd_local_(0) > (xw_local_(0) + 0.20) )
          {
#ifdef DEBUG
            res_ << inTime << "	" << "--> dx exceeding max limit!! changing from: " << xd_local_(0) << " to xt = " << xw_local_(0) + 0.20 << std::endl;
#endif
            xd_local_(0) = xw_local_(0) + 0.20;
          }

          double sign = fabs(xd_local_(0) - xreft_1_local_(0))/(xd_local_(0) - xreft_1_local_(0));
          if( (fabs(xd_local_(0) - xreft_1_local_(0)) > max_dx_ ) && ((xla_(2) >= 0.1052) || (xra_(2) >= 0.1052)) )
          {
#ifdef DEBUG
            res_ << inTime << "	" << "--> vx exceeding max limit when wlkg!! changing from: " << xd_local_(0) << " to xt = " << xreft_1_local_(0) + sign*max_dx_ << ",    vel sign= " << sign << std::endl;
#endif
            xd_local_(0) = xreft_1_local_(0) + sign*max_dx_;
          }
          else if( walkStop_ && (fabs(xd_local_(0) - xreft_1_local_(0)) > 2*max_dx_) )
          {
#ifdef DEBUG
            res_ << inTime << "	" << "--> vx exceeding max limit!! changing from: " << xd_local_(0) << " to xt = " << xreft_1_local_(0) + sign*2*max_dx_ << ",    vel sign= " << sign << std::endl;
#endif
            xd_local_(0) = xreft_1_local_(0) + sign*2*max_dx_;
          }  

          double z;
          if(xt_local_(2) > xreft_1_local_(2))
            z = xt_local_(2);
          else
            z = xreft_1_local_(2);

          sign = fabs(xd_local_(2) - z)/(xd_local_(2) - z);
          if( inTime > 1)
          {
            if( ((xd_local_(2) < 0.695) || (xd_local_(2) > 0.80)) && ((xla_(2) >= 0.1052) || (xra_(2) >= 0.01052) ))
            {
              xd_local_(2) = (xt_(2) + 2*xreft_1_local_(2))/3;
#ifdef DEBUG
              res_ << "--> Z below 0.695 or over 0.730 while wkg, chaging z -> " << xd_local_(2) << std::endl;
#endif
            }  
            else if(((xd_local_(2) < 0.65) || (xd_local_(2) > 0.80)))
            {
              xd_local_(2) = xreft_1_local_(2);
#ifdef DEBUG
              res_ << "----> Z below 0.65 or over 0.80, changing z to previous value -> " << xd_local_(2) << std::endl;
#endif
            }
          }
                                    // max dz
          if( (fabs(xd_local_(2) - z) > 0.0015) &&  walk_)
          {
            if( ((z + sign*0.0015) > 0.695) && ((z + sign*0.0015) < 0.80) )
              xd_local_(2) = z + sign*0.0015;
            else if((z + sign*0.0015) < 0.695)
              xd_local_(2) = 0.695;
            else
              xd_local_(2) = 0.80;
#ifdef DEBUG
            res_ << "--> vz exceeding max limit when wlkg!! changing to xt_= " << xd_local_(2) << ",    vel sign= " << sign << std::endl;
#endif
          }
          else if( (fabs(xd_local_(2) - z) > 0.003) )
          {
            if( ((z + sign*0.003) > 0.65) && ((z + sign*0.003) < 0.80) )
              xd_local_(2) = z + sign*0.003;
            else if((z + sign*0.0015) < 0.65)
              xd_local_(2) = 0.65;
            else
              xd_local_(2) = 0.80;
#ifdef DEBUG
            res_ << "--> vz exceeding max limit!! changing to xt_= " << xd_local_(2) << ",    vel sign= " << sign << std::endl;
#endif
          }

          //Go back to the world frame
          xcf_world_ = Ryaw * xd_local_;
               
          for(unsigned i=0; i < 3; i++)
            lw(i, 3) =  xcf_world_(i);       // + xreft_1_(i) )/2;        // + xt_(i) )/3;


#ifdef DEBUG
          for(unsigned i=0; i < 3; i++)
            imp_(i) = ((m_ / (dt * dt) ) * (xg_(i) - (2*xt_local_(i)) + xt_1_local_(i) )) + ( (c_/dt) * (xt_local_(i) - xt_1_local_(i)) );

          df_ = fd_ - fstatic_;

          xrot_ = Ryaw * xini_;

          force_ << (inTime*0.005) << "	" << inTime;
          for(unsigned k=0; k < 3; ++k)
            force_ << "	" << fr(k);

          for(unsigned k=0; k < 3; ++k)
            force_ << "	" << fraw_(k);

          //force_  << "	" << realTime << std::endl;
          res_ << inTime << "	" << realTime << "	|" << qs(0) << ", "  << qs(1) << "|	" << lw << std::endl;
          res_ << "--   xrot_: " << xrot_ << ", vra_: " << vra_ << "  xcf_w: " << xcf_world_ << std::endl;   //"   xw_: " << xw_ << std::endl;
          check_ << inTime << "	";
   
          double ccx = (c_/dt) * (xt_local(0) - xt_1_local_(0)), ccz = (c_/dt) * (xt_local(2) - xt_1_local_(2));
          double ddx = (2 * xt_local(0)) - xt_1_local_(0), ddz = (2 * xt_local(2)) - xt_1_local_(2);
          double mdx = (((dt * dt) / m_ ) * (fr_local_(0) - (fd_(0) - fla_(0) - fra_(0)) - ccx));
          double mdz = (((dt * dt) / m_ ) * (fr_local_(2) - (fd_(2) - fla_(2) - fra_(2)) - ccz));
          check_<< fla_(0)<< "	" << fra_(0)<< "	" << fr_local_(0) - fd_(0)<< "	" << ccx << "	" << ddx << "	" << mdx << "	";
          check_<< fla_(2)<< "	" << fra_(2)<< "	" << fr_local_(2) - fd_(2)<< "	" << ccz << "	" << ddz << "	" << mdz << "	" << realTime << std::endl;

          pos_ << inTime << "	" << imp_(0) << "	" << imp_(1) << "	" << imp_(2);
          pos_ << "	" << df_(0) << "	" << df_(1) << "	" << df_(2);
          pos_ << "	" << realTime << std::endl;

          wrist_ << inTime;
          for(unsigned k=0; k < 3; ++k)
            wrist_ << "	" << xt_(k) << "	" << lw(k, 3) << "	" << xt_local(k);   //xcf_world_(k);

          wrist_ << "	" << xla_(0) << "	" << xg_(0) << "	" << xla_(1) << "	" << xd_local_(1);
          wrist_ << "	" << xla_(2) << "	" << xg_(2) << "	" << qs(0)+xrot_(0) << "	" << qs(1)+xrot_(1) << "	" << realTime << std::endl;
#endif

          xlat_2_ = xlat_1_;
          xlat_1_ = xla_;
          xrat_2_ = xrat_1_;
          xrat_1_ = xra_;

          xt_1_ = xt_;
          xt_1_local_ = xt_local_;
          xreft_1_local_ = xd_local_;
        }

        if( start_ && ((fabs(fr(0)) < 2.0) && (fabs(fr(1)) < 2.0 )) && (fr(2) > -12.0) && (fr(2) < -10.0))
        {
          xrot_.setZero();
          xrot_ = Ryaw * xini_;
          lw(0,3) = ((qs(0)+xrot_(0)) + xreft_1_(0) + R(0,3))/3;
          lw(1,3) = ((qs(1) + xrot_(1)) + xreft_1_(1) + R(1,3))/3;
          lw(2,3) = ((qs(2)-0.648703+xini_(2)) + 2*xreft_1_(2) + 3*R(2,3))/6;
#ifdef DEBUG
          res_ << "~~~~ " << inTime << ", fr = " << fr << ",    xref = " << xreft_1_ << std::endl;
          res_ << "~~~~ final = " << lw << std::endl;
#endif
        }

        t_1_ = inTime;

        //Rotate posture of lw as the waist yaw

        xlw_ = extractVector(lw);
        Rrot = Ryaw * pos_ini_;    
        lw = buildfrom(xlw_, Rrot);

        xreft_1_ = xlw_; 

#ifdef DEBUG
        res_ << inTime << "	" << "++++++ ref local = " << xreft_1_local_ << std::endl;
#endif
        return lw;
      }

      Vector& ImpedanceController::computePosture(Vector& q, const int& inTime)
      {
        const Vector& force = forceSOUT.access(inTime);
        const Vector& qs = postureSIN(inTime);
        bool change = false;

        if(!start_)
          q0_ = qs;

        for(unsigned i=0; i < 3; i++)
        {
          if( fabs( force(i) ) > 300.0)
            change = true;
        }

        if(change && !hold_)
        {
          qt_ = q0_;
          //  Keep only the most recent position & orientation of the waist
          for(unsigned i=0; i < 6; i++)
            qt_(i) = qs(i);

          qt_(28) = 0.75;
          qt_(35) = 0.75;
          q = qt_;
          hold();
        }
        else if(hold_ || open_)
        {
          qt_ = qs;
          qt_(28) = 0.75;
          qt_(35) = 0.75;
          q = qt_;
        }
        else if(close_)
        {
          qt_ = qs;
          qt_(28) = 0.15;
          qt_(35) = 0.15;
          q = qt_;
        }
        else
          q = qs;

        return q;
      }

      Vector& ImpedanceController::computeForce(Vector& force, const int& inTime)
      {
        const Vector& fin = forceSIN(inTime);
        const MatrixHomogeneous& R = lwSIN(inTime);
        force.resize(3);
        ff_.setZero(); fR_.setZero(); ft_.setZero();
        Eigen::Matrix3d MRot;

        if (inTime != tf_1_)
        {
          ft_(2) = fin(2);
          ft_(0) = -fin(1);
          ft_(1) = fin(0);
          MRot = extractMatrix(R);
          fR_ = MRot * ft_;

          if( (ff_2_(2) == 0.0) && (ff_1_(2) == 0))
          {
            ff_1_ = fR_;
            ff_2_ = fR_;
            fRt_1_ = fR_;
            fRt_2_ = fR_;
          
#ifdef DEBUG
            res_ << "f0: " << fR_ << ", " << ff_2_ << std::endl;
#endif
          }

          fraw_ = fR_;
          ff_ = (0.00024136*fR_) + (0.00048272*fRt_1_) + (0.00024136*fRt_2_) + (1.95557824*ff_1_) - (0.95654368*ff_2_);
          ff_2_ = ff_1_;
          ff_1_ = ff_;
          fRt_2_ = fRt_1_;
          fRt_1_ = fR_;
          force = fill(ff_);

          tf_1_ = inTime;
        }
        else
        {
          force = fill(ff_1_);
        }

        return force;
      }

      void ImpedanceController::start()
      {
        if(!start_)
        {
          start_ = true;
          //startTime_ = std::max (std::max (forceSIN.getTime (), lwSIN.getTime ()), lwSOUT.getTime ());
          stop_ = false;
        }
      }

      void ImpedanceController::stop()
      {
        if(start_)
        {
          stop_ = true;
          start_ = false;
#ifdef DEBUG
          res_ << "=========Controller stoppped at t = " << t_1_ << std::endl;
#endif
        }
        else
        {
          throw std::runtime_error
              ("The Controller has not been started!");
        }
      }

      void ImpedanceController::save()
      {
        if(stop_)
        {
#ifdef DEBUG
          wrist_.close();
          res_.close();
          force_.close();
          pos_.close();
          check_.close();
#endif
        }
        else
        {
          throw std::runtime_error
              ("The Controller is still running! Stop the controller before saving data");
        }
      }

      void ImpedanceController::hold()
      {
        if(start_ && !stop_)
        {
          hold_ = true;
          //start_ = false;
          //stop_  = true;
          //throw std::runtime_error
          //("The Controller was stopped!! Hose released!!");
#ifdef DEBUG
          res_ << "The Controller was stopped!! Hose released!! at t = " << (t_1_) << std::endl;
#endif
        }
      }

      void ImpedanceController::openGripper()
      {
        close_ = false;
        open_ = true;
      }

      void ImpedanceController::closeGripper()
      {
        close_ = true;
        open_ = false;
      }

      inline Eigen::Vector3d ImpedanceController::extractVector(const MatrixHomogeneous& m)
      {
        Eigen::Vector3d v;
        for(unsigned int i=0; i<v.size(); i++)
            v(i) = m(i, 3);

        return v;
      }

      inline Eigen::MatrixXd ImpedanceController::extractMatrix(const MatrixHomogeneous& m)
      {
        Eigen::MatrixXd res;
        res.resize(3,3);
        for(unsigned int i=0; i<res.rows(); i++)
        {
          for(unsigned int j=0; j<res.cols(); j++)
            res(i, j) = m(i, j);
        }

        return res;
      }

      inline MatrixHomogeneous ImpedanceController::buildfrom(Eigen::Vector3d& v, Eigen::MatrixXd& m)
      {
        MatrixHomogeneous a;
            
        for(unsigned int i=0; i<3; i++)
        {
          a(i, 3) = v(i);
          a(3, i) = 0.0;
          for(unsigned int j=0; j<3; j++)
            a(i, j) = m(i, j);
        }
        a(3,3) = 1.0;

        return a;
      }

      inline Vector ImpedanceController::fill(Eigen::Vector3d& v)
      {
        Vector a;
        a.resize(3);
        
        for(unsigned int i=0; i<3; i++)
            a(i) = v(i);
        
        return a;
      }

      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ImpedanceController, "ImpedanceController");
    } // namespace tools
  } //namespace sot
} // namespace dynamicgraph

