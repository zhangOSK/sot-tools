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
        start_(false), stop_(false), hold_(false), init_(false), walk_(false), walkStop_(false), open_(false), close_(false), iniTime_(std::tm())
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


        xt_1_.resize(3);	fraw_.resize(3);	fraw_.setZero();
        ff_1_.resize(3);	ff_2_.resize(3);    xt_1_local_.resize(3);
        ff_1_.setZero();	ff_2_.setZero();    xt_1_local_.setZero();
        xcft_1_.resize(3);		xcft_1_.setZero();
        fRt_1_.resize(3);	fRt_2_.resize(3);	xcft_2_.resize(3);
        fRt_1_.setZero();	fRt_2_.setZero();	xcft_2_.setZero();
        xreft_1_.resize(3);	xreft_1_.setZero();
        fd_.resize(3);		fd_.setZero();
        xct_1_.resize(3);
        xct_1_.setZero();	xlat_1_.resize(3);
        xlat_2_.resize(3);	xlat_1_.setZero();
        xlat_2_.setZero();	xrat_1_.resize(3);
        xrat_2_.resize(3);	xrat_1_.setZero();
        xrat_2_.setZero();
        xreft_1_local_.resize(3);   xreft_1_local_.setZero();
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

        // pos_ini_ should be the position of the left wrist in half sitting for hrp2-14
        //LEFT-WRIST
        pos_ini_(0,0) = 0.963962  ; pos_ini_(0,1) =0.0449441; pos_ini_(0,2) =-0.262218 ; pos_ini_(0,3) =0.0418365;
        pos_ini_(1,0) = -0.0868231; pos_ini_(1,1) =0.984808 ; pos_ini_(1,2) =-0.150382 ; pos_ini_(1,3) =0.331007 ;
        pos_ini_(2,0) = 0.251475  ; pos_ini_(2,1) =0.167729 ; pos_ini_(2,2) =0.953219  ; pos_ini_(2,3) =0.704286 ;
        pos_ini_(3,0) = 0	      ; pos_ini_(3,1) =0	    ; pos_ini_(3,2) =0	       ; pos_ini_(3,3) =1        ;

//        pos_ini_(0,0) = 0.922498;      pos_ini_(0,1) = 0.0287972;      pos_ini_(0,2) = -0.384925;      pos_ini_(0,3) = 0.149084;
//        pos_ini_(1,0) = -0.106533;      pos_ini_(1,1) = 0.977476;       pos_ini_(1,2) = -0.182185;      pos_ini_(1,3) = 0.40; //0.350307;
//        pos_ini_(2,0) = 0.371009;       pos_ini_(2,1) = 0.209073;       pos_ini_(2,2) = 0.904788;       pos_ini_(2,3) = 0.698064;
//        pos_ini_(3,0) = 0;      pos_ini_(3,1) = 0;      pos_ini_(3,2) = 0;      pos_ini_(3,3) = 1;

        iniTime_.tm_hour = 10;	iniTime_.tm_min = 30;	iniTime_.tm_sec = 0;
        iniTime_.tm_year = 115;	iniTime_.tm_mon = 11;	iniTime_.tm_mday = 15;
        // year counted from 1900

        wrist_.open("/tmp/WristPos.txt", std::ios::out);
        force_.open("/tmp/Force.txt", std::ios::out);
        res_.open("/tmp/ControllerOutput.txt", std::ios::out);
        check_.open("/tmp/controllerCalcs.txt", std::ios::out);
        //	pos_.open("/tmp/Controller.posture", std::ios::out);
        pos_.open("/tmp/Impedance.txt", std::ios::out);

        res_ << "----> Mass Hose: " << massHose << std::endl;
        res_ << "----> Fd: " << fd_ << std::endl;
        res_ << "----> Controller: m = " << m_ << ",  c = " << c_ << ", times m: " << mx_ << ", times c: " << cx_ << std::endl;
        // Define refresh function for output signal
        //boost::function2<double&, double&,const int&> ftest
        //= boost::bind(&ImpedanceController::computeControlOutput, this, _1, _2);

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
        Vector xt, xg, imp, df, ftemp, xla, xra, fla, fra, fstatic, vla, vra, xcf, xlw, xrot, xini;
        fla.resize(3);	fra.resize(3);	fstatic.resize(3);
        xt.resize(3);	 xg.resize(3);	fstatic.setZero();
        imp.resize(3);	 df.resize(3);  xt.setZero();
        ftemp.resize(3);	xla.resize(3);	xra.resize(3);	xcf.resize(3);	xcf.setZero();
        vla.resize(3);	vra.resize(3);	vla.setZero();	vra.setZero();
        xlw.resize(3);  xlw.setZero();  xrot.resize(3); xrot.setZero();
        xini.resize(3); xini.setZero();
        time_t myTime;
        time(&myTime);
        double realTime = difftime(myTime, mktime(&iniTime_));

        MatrixRotation Ryaw, Rlw, Rrot, Rinv;

        la.extract(Ryaw);
        Ryaw.inverse(Rinv);
        
        if(!start_ || stop_ || hold_)
        {
          pos_ini_.extract(xini);
          Ryaw.multiply(xini, xrot);

          if(!init_)
          {
            // Save the distance from wrist to the  waist at the starting position
            dy_ = R(1, 3)-qs(1);
            lw_initial_ = R;
            for(unsigned i=0; i < 3; i++)
              xreft_1_(i) = R(i, 3);

            init_ = true;
            f_ini_ = fr;
            res_ << "----->>> dy = " << dy_ << std::endl;
            res_ << "===>>> lw ini = " << lw_initial_ << std::endl;
            res_ << "===>>> fr ini = " << f_ini_ << std::endl;
          }

          R.extract(xt);
          Rinv.multiply(xt, xt_1_local_);
          for(unsigned i=0; i < 3; i++)
          {
            xt_1_(i) = R(i, 3);
            xct_1_(i) = R(i, 3);
            xlat_1_(i) = la(i, 3);
            xlat_2_(i) = la(i, 3);
            xrat_1_(i) = ra(i, 3);
            xrat_2_(i) = ra(i, 3);
            xcft_1_(i) = xct_1_(i);
            xcft_2_(i) = xct_1_(i);
          }

          t_1_ = inTime;
          lw = pos_ini_;
          lw(0,3) = qs(0)+xrot(0);
          lw(1,3) = qs(1)+xrot(1);
          lw(2,3) = qs(2)-0.648703+pos_ini_(2,3);
          f_ini_ = fr;

          if(hold_)
          {
            lw = pos_ini_;
            lw(0, 3) = (3*R(0,3) + (qs(0) +xrot(0)) + 2*xreft_1_(0))/6;
            lw(1, 3) = ( (qs(1)+xrot(1)) + 3*R(1,3) + 2*xreft_1_(1))/6;
            lw(2, 3) = (3*R(2,3) + (qs(2)-0.648703+pos_ini_(2,3)) + 2*xreft_1_(2))/6;

            res_ << "**" << inTime << "	" << lw << std::endl;
            wrist_ << inTime;
            for(unsigned k=0; k < 3; ++k)
            {
              wrist_ << "	" << R(k, 3) << "	" << lw(k, 3) << "	0.001" ;
              xt_1_(k) = R(k, 3);
              xreft_1_(k) = lw(k, 3);
            }

            wrist_ << "	" << la(0, 3) << "	" << ra(0, 3) << "	" << la(1, 3) << "	" << ra(1, 3);
            wrist_ << "	" << la(2, 3) << "	" << ra(2, 3) << "	0.001	0.001	" << realTime << std::endl;

          }
          lwct_1_ = R;

          if(stop_)
          {
            lw = R;
            lw(0,3) = ( (qs(0)+xrot(0)) + 3*R(0,3) + 2*xreft_1_(0))/6;
            lw(1,3) = ( (qs(1)+xrot(1)) + 3*R(1,3) + 2*xreft_1_(1))/6;
            lw(2,3) = ( (qs(2)-0.648703+pos_ini_(2,3)) + 3*R(2,3) + 2*xreft_1_(2))/6;
            res_ << "~~~~ stopped = " << inTime << "    " << lw << std::endl;
          }  

          lw.extract(xlw);
          Rinv.multiply(xlw, xreft_1_local_);       
        }
        else if (start_)
        {
          lw = R;
          for(unsigned i=0; i < 3; i++)
          {
            xt(i) = R(i, 3);
            xla(i) = la(i, 3);
            xra(i) = ra(i, 3);
            vla(i) = 0.0;
            vra(i) = 0.0;
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
            res_ << "fr = " << fr << ",  fraw = " << fraw_ << std::endl;
            hold();
          }

          ///Update desired force depending on distance to the reel
         //----Experiment----------------------------------------------
 /*         dist_ = xt(0) - pos_ini_(0,3);
          double massHose = dist_*1.8;      //Real hose mass = 1.8 kg/m
          fd_(0) = f_ini_(0) + (0.5 * massHose * (-9.8));
          if(dist_ < 1.5)
            fd_(2) = f_ini_(2) + (0.5 * massHose * (-9.8));
          else
            fd_(2) = f_ini_(2) + (1.8 * (-9.8));
          //--------------------------------------------------

          //---Simulation-------------------------------
          //dist_ = (xt(0) - pos_ini_(0,3))+2.0;
          massHose = dist_*1.8;       
          fd_(0) = f_ini_(0) + (0.5 * (0.15 * massHose) * (-9.8));
          fd_(2) = f_ini_(2) + (0.05 * massHose * (-9.8) );
          //----------------------------------------------------
//*/

          for(unsigned int i=0; i<3; i++)
          {
            vla(i) = ( xla(i) - xlat_1_(i) )/dt;
            vra(i) = ( xra(i) - xrat_1_(i) )/dt;
          }


          //if( ((fla(0) > 50.0) || (fra(0) > 50.0)) && !walk_)
          bool floor = ( fabs(xla(2) < 0.10505) && fabs(xra(2) < 0.10505) );
          if( ((vla(2) > 0.1) || (vra(2) > 0.1)) && ((xla(2) > 0.106) || (xra(2) > 0.106)) && !walk_)
          {
            walk_ = true;
            walkStop_ = false;
            res_ << "======> Started walking at : " << inTime*0.005 << ", realtime: " << realTime << std::endl;
          }
          else if( ( (((fabs(vla(2))) < 0.0001)&&((fabs(vra(2))) < 0.0001)) || ((fabs(xla(0)-xra(0)) < 0.00001)&&(fabs(vla(0)) < 0.001)) ) && walk_ && floor && (vel(0) ==0.0) )
          {
            walkStop_ = true;
            walk_ = false;
            elapsed_ = 0;
            res_ << "####### Finish walking at : " << inTime*0.005<< ", realtime: " << realTime << std::endl;
          }

          if( (xla(2) < 0.1051) && (xra(2) < 0.1051) && walk_ && !walkStop_)
          {
            if(elapsed_ == 0)
              vel_fix_ = vel(0);

            fstatic(0) = vel_fix_*(27000);		//2700
            elapsed_ = elapsed_ + 1;
          }
          else
          {
            elapsed_ = 0;
            for(unsigned i=0; i < 3; i++)
              fstatic(i) = 0.0;
          }


          Vector xt_local, fr_local, xcf_world, xw_local, xw;
          fr_local.resize(3);   fr_local.setZero();     xt_local.resize(3);    xt_local.setZero();
          xcf_world.resize(3);  xcf_world.setZero();    xw.resize(3);   xw_local.resize(3);     xw.setZero();   xw_local.setZero();
          Rinv.multiply(fr, fr_local);
          Rinv.multiply(xt, xt_local);
          xw(0) = qs(0);    xw(1) = qs(1);  xw(2) = qs(2);
          Rinv.multiply(xw, xw_local);      

          fr_local(1) = 0.0;    
          for(unsigned i=0; i < 3; i++)
          {
            xg(i) = (((dt * dt) / m_ ) * (fr_local(i) - (fd_(i) - fstatic(i)) - ((c_/dt) * (xt_local(i) - xt_1_local_(i))) )) + (2 * xt_local(i)) - xt_1_local_(i);
            xcf(i) = (xg(i) + (2*xcft_1_(i)) + xcft_2_(i))/4;
            xct_1_(i) = xg(i);
            xcft_2_(i) = xcft_1_(i);
            xcft_1_(i) = xcf(i);
          }

          Vector xd_local;
          xd_local.resize(3);   xd_local.setZero();
    
          for(unsigned i=0; i < 3; i++)
            xd_local(i) = (xcf(i) + xreft_1_local_(i) + xt_local(i))/3;

          //In Y, keep the distance at starting position with the waist, always!!
          xd_local(1) = dy_ + xw_local(1); 

          //Check limits on the local (waist) frame
          if( xd_local(0) > (xw_local(0) + 0.20) )
          {
            res_ << inTime << "	" << "--> dx exceeding max limit!! changing from: " << xd_local(0) << " to xt = " << xw_local(0) + 0.20 << std::endl;
            xd_local(0) = xw_local(0) + 0.20;
          }

          double sign = fabs(xd_local(0) - xreft_1_local_(0))/(xd_local(0) - xreft_1_local_(0));
          if( (fabs(xd_local(0) - xreft_1_local_(0)) > max_dx_ ) && ((xla(2) >= 0.1052) || (xra(2) >= 0.1052)) )
          {
            res_ << inTime << "	" << "--> vx exceeding max limit when wlkg!! changing from: " << xd_local(0) << " to xt = " << xreft_1_local_(0) + sign*max_dx_ << ",    vel sign= " << sign << std::endl;
            xd_local(0) = xreft_1_local_(0) + sign*max_dx_;
          }
          else if( walkStop_ && (fabs(xd_local(0) - xreft_1_local_(0)) > 2*max_dx_) )
          {
            res_ << inTime << "	" << "--> vx exceeding max limit!! changing from: " << xd_local(0) << " to xt = " << xreft_1_local_(0) + sign*2*max_dx_ << ",    vel sign= " << sign << std::endl;
            xd_local(0) = xreft_1_local_(0) + sign*2*max_dx_;
          }  

          double z;
          if(xt_local(2) > xreft_1_local_(2))
            z = xt_local(2);
          else
            z = xreft_1_local_(2);

          sign = fabs(xd_local(2) - z)/(xd_local(2) - z);
          if( inTime > 1)
          {
            if( ((xd_local(2) < 0.695) || (xd_local(2) > 0.80)) && ((xla(2) >= 0.1052) || (xra(2) >= 0.01052) ))
            {
              xd_local(2) = (xt(2) + 2*xreft_1_local_(2))/3;   
              res_ << "--> Z below 0.695 or over 0.730 while wkg, chaging z -> " << xd_local(2) << std::endl;
            }  
            else if(((xd_local(2) < 0.65) || (xd_local(2) > 0.80)))
            {
              xd_local(2) = xreft_1_local_(2);
              res_ << "----> Z below 0.65 or over 0.80, changing z to previous value -> " << xd_local(2) << std::endl;
            }
          }
                                    // max dz
          if( (fabs(xd_local(2) - z) > 0.0015) &&  walk_)
          {
            if( ((z + sign*0.0015) > 0.695) && ((z + sign*0.0015) < 0.80) )
              xd_local(2) = z + sign*0.0015;
            else if((z + sign*0.0015) < 0.695)
              xd_local(2) = 0.695;
            else
              xd_local(2) = 0.80;
            res_ << "--> vz exceeding max limit when wlkg!! changing to xt= " << xd_local(2) << ",    vel sign= " << sign << std::endl;
          }
          else if( (fabs(xd_local(2) - z) > 0.003) )
          {
            if( ((z + sign*0.003) > 0.65) && ((z + sign*0.003) < 0.80) )
              xd_local(2) = z + sign*0.003;
            else if((z + sign*0.0015) < 0.65)
              xd_local(2) = 0.65;
            else
              xd_local(2) = 0.80;
            res_ << "--> vz exceeding max limit!! changing to xt= " << xd_local(2) << ",    vel sign= " << sign << std::endl;
          }

          //Go back to the world frame
          Ryaw.multiply(xd_local, xcf_world);

          for(unsigned i=0; i < 3; i++)
            imp(i) = ((m_ / (dt * dt) ) * (xg(i) - (2*xt_local(i)) + xt_1_local_(i) )) + ( (c_/dt) * (xt_local(i) - xt_1_local_(i)) );

          df = fd_ - fstatic;     
               
          for(unsigned i=0; i < 3; i++)
            lw(i, 3) =  xcf_world(i);       // + xreft_1_(i) )/2;        // + xt(i) )/3;

          pos_ini_.extract(xini);
          Ryaw.multiply(xini, xrot);         

          force_ << (inTime*0.005) << "	" << inTime;
          for(unsigned k=0; k < 3; ++k)
            force_ << "	" << fr(k);

          for(unsigned k=0; k < 3; ++k)
            force_ << "	" << fraw_(k);


          force_  << "	" << realTime << std::endl;

          res_ << inTime << "	" << realTime << "	|" << qs(0) << ", "  << qs(1) << "|	" << lw << std::endl;
          res_ << "--   xrot: " << xrot << ", xcf: " << xcf << "  xcf_w: " << xcf_world << std::endl;   //"   xw: " << xw << std::endl;
          check_ << inTime << "	";
          double ccx = (c_/dt) * (xt_local(0) - xt_1_local_(0)), ccz = (c_/dt) * (xt_local(2) - xt_1_local_(2));
          double ddx = (2 * xt_local(0)) - xt_1_local_(0), ddz = (2 * xt_local(2)) - xt_1_local_(2);
          double mdx = (((dt * dt) / m_ ) * (fr_local(0) - (fd_(0) - fla(0) - fra(0)) - ccx));
          double mdz = (((dt * dt) / m_ ) * (fr_local(2) - (fd_(2) - fla(2) - fra(2)) - ccz));
          check_<< fla(0)<< "	" << fra(0)<< "	" << fr_local(0) - fd_(0)<< "	" << ccx << "	" << ddx << "	" << mdx << "	";
          check_<< fla(2)<< "	" << fra(2)<< "	" << fr_local(2) - fd_(2)<< "	" << ccz << "	" << ddz << "	" << mdz << "	" << realTime << std::endl;

          pos_ << inTime << "	" << imp(0) << "	" << imp(1) << "	" << imp(2);
          pos_ << "	" << df(0) << "	" << df(1) << "	" << df(2);
          pos_ << "	" << realTime << std::endl;

          pos_ini_.extract(xini);
          xrot.setZero();
          Ryaw.multiply(xini, xrot);

          wrist_ << inTime;
          for(unsigned k=0; k < 3; ++k)
            wrist_ << "	" << xt(k) << "	" << lw(k, 3) << "	" << xt_local(k);   //xcf_world(k);

          wrist_ << "	" << xla(0) << "	" << xg(0) << "	" << xla(1) << "	" << xd_local(1);
          wrist_ << "	" << xla(2) << "	" << xg(2) << "	" << qs(0)+xrot(0) << "	" << qs(1)+xrot(1) << "	" << realTime << std::endl;

          lwct_1_ = lw;
          xlat_2_ = xlat_1_;
          xlat_1_ = xla;
          xrat_2_ = xrat_1_;
          xrat_1_ = xra;

          xt_1_ = xt;
          xt_1_local_ = xt_local;
          xreft_1_local_ = xd_local;
        }

        if( start_ && ((fabs(fr(0)) < 2.0) && (fabs(fr(1)) < 2.0 )) && (fr(2) > -12.0) && (fr(2) < -10.0))
        {
          pos_ini_.extract(xini);
          xrot.setZero();
          Ryaw.multiply(xini, xrot);
          lw(0,3) = ((qs(0)+xrot(0)) + xreft_1_(0) + R(0,3))/3;
          lw(1,3) = ((qs(1) + xrot(1)) + xreft_1_(1) + R(1,3))/3;
          lw(2,3) = ((qs(2)-0.648703+pos_ini_(2,3)) + 2*xreft_1_(2) + 3*R(2,3))/6;
          res_ << "~~~~ " << inTime << ", fr = " << fr << ",    xref = " << xreft_1_ << std::endl;
          res_ << "~~~~ final = " << lw << std::endl;
        }

        t_1_ = inTime;

        //Rotate posture of lw as the waist yaw
        la.extract(Ryaw);
        lw.extract(xlw);
        pos_ini_.extract(Rlw);
        Ryaw.multiply(Rlw, Rrot);
        lw.buildFrom(Rrot, xlw);

        lw.extract(xreft_1_);
        //Rinv.multiply(xlw, xreft_1_local_);
        res_ << inTime << "	" << "++++++ ref local = " << xreft_1_local_ << std::endl;

        return lw;
      }

      Vector& ImpedanceController::computePosture(Vector& q, const int& inTime)
      {
        const Vector& force = forceSOUT.access(inTime);
        const Vector& qs = postureSIN(inTime);
        bool change = false;
        int tam = qs.size();
        Vector qt;
        qt.resize(tam);

        if(!start_)
          q0_ = qs;

        for(unsigned i=0; i < 3; i++)
        {
          if( fabs( force(i) ) > 300.0)
            change = true;
        }

        if(change && !hold_)
        {
          qt = q0_;
          //  Keep only the most recent position & orientation of the waist
          for(unsigned i=0; i < 6; i++)
            qt(i) = qs(i);

          qt(28) = 0.75;
          qt(35) = 0.75;
          q = qt;
          hold();
        }
        else if(hold_ || open_)
        {
          qt = qs;
          qt(28) = 0.75;
          qt(35) = 0.75;
          q = qt;
        }
        else if(close_)
        {
          qt = qs;
          qt(28) = 0.15;
          qt(35) = 0.15;
          q = qt;
        }
        else
          q = qs;

        return q;
      }

      Vector& ImpedanceController::computeForce(Vector& force, const int& inTime)
      {
        const Vector& fin = forceSIN(inTime);
        const MatrixHomogeneous& R = lwSIN(inTime);
        MatrixRotation MRot;
        Vector fR, ff, ft;
        fR.resize(3);	ft.resize(3);
        fR.setZero();
        ff.resize(3);	force.resize(3);

        if (inTime != tf_1_)
        {
          ft(2) = fin(2);
          ft(0) = -fin(1);
          ft(1) = fin(0);
          R.extract (MRot);
          MRot.multiply (ft, fR);

          if( (ff_2_(2) == 0.0) && (ff_1_(2) == 0))
          {
            for(unsigned i=0; i < 3; i++)
            {
              ff_1_(i) = fR(i);
              ff_2_(i) = fR(i);
              fRt_1_(i) = fR(i);
              fRt_2_(i) = fR(i);
            }
            res_ << "f0: " << fR << ", " << ff_2_ << std::endl;
          }

          for(unsigned i=0; i < 3; i++)
          {
            fraw_(i) = fR(i);
            ff(i) = (0.00024136*fR(i)) + (0.00048272*fRt_1_(i)) + (0.00024136*fRt_2_(i)) + (1.95557824*ff_1_(i)) - (0.95654368*ff_2_(i));
            ff_2_(i) = ff_1_(i);
            ff_1_(i) = ff(i);
            fRt_2_(i) = fRt_1_(i);
            fRt_1_(i) = fR(i);
            force(i) = ff(i);
          }

          tf_1_ = inTime;
        }
        else
        {
          force = ff_1_;
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
          res_ << "=========Controller stoppped at t = " << t_1_ << std::endl;
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
          wrist_.close();
          res_.close();
          force_.close();
          pos_.close();
          check_.close();
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
          res_ << "The Controller was stopped!! Hose released!! at t = " << (t_1_) << std::endl;
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

      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ImpedanceController, "ImpedanceController");
    } // namespace tools
  } //namespace sot
} // namespace dynamicgraph

