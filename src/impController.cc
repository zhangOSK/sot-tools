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
        lwSOUT(forceSIN << lwSIN << laSIN, "ImpedanceController("+inName+")::output(MatrixHomo)::leftWristOUT"),
        gripSOUT(forceSIN, "ImpedanceController("+inName+")::output(vector)::grip"),
        postureSOUT(forceSIN << postureSIN, "ImpedanceController("+inName+")::output(vector)::postureOUT"),
	forceSOUT(forceSIN << lwSIN, "ImpedanceController("+inName+")::output(vector)::leftWristForce"),
        m_(1.00), c_(5.0), mx_(20), cx_(100), max_dx_(0.0025), xt_1_(), q0_(), xct_1_(), xlat_1_(), xlat_2_(), xrat_1_(), xrat_2_(), t_1_(0), tf_1_(0), elapsed_(0), walk_vel_(0.1),
		ff_1_(), ff_2_(), lw_initial_(), lwct_1_(), pos_ini_(), start_(false), stop_(false), hold_(false), init_(false), 
		walk_(false), walkStop_(false)
      {
       // Register signals into the entity.
        signalRegistration (lwSOUT); 
        signalRegistration (forceSIN);
        signalRegistration (lwSIN);
        signalRegistration (laSIN);
        signalRegistration (raSIN);
	signalRegistration (postureSIN);
	signalRegistration (gripSOUT);
	signalRegistration (postureSOUT);
	signalRegistration (forceSOUT);

    xt_1_.resize(3);	fraw_.resize(3);	fraw_.setZero();
	ff_1_.resize(3);	ff_2_.resize(3);
	ff_1_.setZero();	ff_2_.setZero();
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
								 // longer hose (1.25 times longHose) = 11.34 -> 8.84 (MLJ), longest Hose (1.5 times longHose) = 13.64 -> 10.64 (MLJ)
        double massHose = 13.197; //full hose: 9.04 -> 7.04 (massless joints MLJ), heavy = 13.56, light = 4.52 (50% fullHose), semi-light = 6.78 (75% of fullHose);
		double part = 0.22;   // 0.32 for longHose // 0.3 for heavy-longHose //Hold part % of the total weight of the Hose 
							//for longer and longest Hose CAREFUL!!-holding weight in Z does not changes with length!!!
        double mu = 0.5;
        double gx = -9.8;
        fd_(0) = mu * (0.15 * massHose) * gx;
        fd_(1) = 0.0;
        fd_(2) = (part * massHose * gx) - 11.0;   //Sensor offset = -11.0
        pos_ini_(0,0) = 0.922498;	pos_ini_(0,1) = 0.0287972;	pos_ini_(0,2) = -0.384925;	pos_ini_(0,3) = 0.149084;
		pos_ini_(1,0) = -0.106533;	pos_ini_(1,1) = 0.977476;	pos_ini_(1,2) = -0.182185;	pos_ini_(1,3) = 0.40; //0.350307;
		pos_ini_(2,0) = 0.371009;	pos_ini_(2,1) = 0.209073;	pos_ini_(2,2) = 0.904788;	pos_ini_(2,3) = 0.698064;
		pos_ini_(3,0) = 0;	pos_ini_(3,1) = 0;	pos_ini_(3,2) = 0;	pos_ini_(3,3) = 1;
		iniTime_ = {0};
		iniTime_.tm_hour = 9;	iniTime_.tm_min = 10;	iniTime_.tm_sec = 0;
		iniTime_.tm_year = 115;	iniTime_.tm_mon = 10;	iniTime_.tm_mday = 23;
		// year counted from 1900

        wrist_.open("/home/ixchel/devel/ros-sot/sim-logs/WristPos.txt", std::ios::out);
	force_.open("/home/ixchel/devel/ros-sot/sim-logs/Force.txt", std::ios::out);
	res_.open("/home/ixchel/devel/ros-sot/sim-logs/ControllerOutput.txt", std::ios::out);
	check_.open("/home/ixchel/devel/ros-sot/sim-logs/controllerCalcs.txt", std::ios::out);
//	pos_.open("/home/ixchel/devel/ros-sot/sim-logs/Controller.posture", std::ios::out);
	pos_.open("/home/ixchel/devel/ros-sot/sim-logs/Impedance.txt", std::ios::out);

	res_ << "----> Mass Hose: " << massHose << std::endl;
	res_ << "----> Fd: " << fd_ << std::endl;
	res_ << "----> Controller: m = " << m_ << ",  c = " << c_ << ", times m: " << mx_ << ", times c: " << cx_ << std::endl;
        // Define refresh function for output signal
        //boost::function2<double&, double&,const int&> ftest
          //= boost::bind(&ImpedanceController::computeControlOutput, this, _1, _2);

        lwSOUT.setFunction (boost::bind(&ImpedanceController::computeControlOutput, this, _1, _2));

        gripSOUT.setFunction (boost::bind(&ImpedanceController::computeSelection, this, _1, _2));

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

        // setVelocity
        docstring =
          "\n"
          "    Set the walking velocity when using the impedance controller\n"
          "      takes a double number as input\n"
          "\n";
        addCommand(std::string("setVelocity"),
	           new ::dynamicgraph::command::Setter<ImpedanceController, double>
	          (*this, &ImpedanceController::setVelocity, docstring));


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
      }

      ImpedanceController::~ImpedanceController()
      { 
      }

      MatrixHomogeneous& ImpedanceController::computeControlOutput(MatrixHomogeneous& lw, const int& inTime)
      {
		//const Vector& force = forceSIN(inTime);
		const MatrixHomogeneous& R = lwSIN(inTime);
		const MatrixHomogeneous& la = laSIN(inTime);
		const MatrixHomogeneous& ra = raSIN(inTime);
		const Vector& fr = forceSOUT.access(inTime);
		const Vector& qs = postureSIN(inTime);
		const double& dt = 0.005; //inTime - t_1_;
		Vector xt, xg, imp, df, ftemp, xla, xra, fla, fra, fstatic, vla, vra, xcf;
		fla.resize(3);	fra.resize(3);	fstatic.resize(3);
		xt.resize(3);	 xg.resize(3);	fstatic.setZero();
		imp.resize(3);	 df.resize(3);
		ftemp.resize(3);	xla.resize(3);	xra.resize(3);	xcf.resize(3);	xcf.setZero();
		vla.resize(3);	vra.resize(3);	vla.setZero();	vra.setZero();
		time_t myTime;
		time(&myTime);
		double realTime = difftime(myTime, mktime(&iniTime_));

		if(!start_ || stop_ || hold_)
		{
		  if(!init_)
		  {	     
			// Save the distance from wrist to the  waist at the starting position
			dy_ = R(1, 3)-qs(1);
			init_ = true;
			res_ << "----->>> dy = " << dy_ << std::endl;
		  }
            
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

          //t_1_ = inTime;
		  lw = R;
		  if(!start_ && !hold_)
			lw_initial_ = R;

		  if(hold_)
		  {
			//lw(0,3) = qs(0);
			lw = lw_initial_;		
			lw(0, 3) = (2*R(0,3) + qs(0) + xt_1_(0) + xreft_1_(0))/5;
			lw(1, 3) = qs(1) + dy_;
			lw(2, 3) = (2*R(2,3) + lw_initial_(2,3) + xt_1_(2) + xreft_1_(2))/5;

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

		  //wrist_ << "---" << inTime << "	" << lw << std::endl;
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
			  for(unsigned j=0; j < 3; j++)
			  {  
				lw(i, j) = 0.0;
				if (i == j)
				  lw(i, j) = 1.0;
			  }
			}

			lw = pos_ini_;

			bool check = false;
			for(unsigned i=0; i < 3; i++)
			{
			  //if ( fabs(fr(i)) > 300.0 )
			    //check = true;

			  if( (inTime) > 5000 && (fabs(fraw_(i)) > 300.0) )
			  {
				check = true;
				break;
			  }
			}

			if (check)
			{
			  lw = R;
			  res_ << fr(0) << "	" << fr(1) << "	" << fr(2) << std::endl;
			  hold();
			}     

			for(unsigned int i=0; i<3; i++)
			{
				vla(i) = ( xla(i) - xlat_1_(i) )/dt;
				vra(i) = ( xra(i) - xrat_1_(i) )/dt;
				fla(i) = ((mx_*m_ / (dt * dt) ) * (xla(i) - (2*xlat_1_(i)) + xlat_2_(i) )) + ( (cx_*c_/dt) * (xla(i) - xlat_1_(i)) );
				fra(i) = ((mx_*m_ / (dt * dt) ) * (xra(i) - (2*xrat_1_(i)) + xrat_2_(i) )) + ( (cx_*c_/dt) * (xra(i) - xrat_1_(i)) );
			}

			double axl = ( xla(0) - (2*xlat_1_(0)) + xlat_2_(0) )/ (dt * dt);		   
			if( (vla(0) < 0.0) || (vla(0) > 0.5) )	//vel max in X direction
				fla(0) = 0.0;
			else if (fabs(axl > 2.3))					//max accleration in X
				fla(0) = (mx_*m_ * 2.2) + ( (cx_*c_/dt) * (xla(0) - xlat_1_(0)) );

			double axr = ( xra(0) - (2*xrat_1_(0)) + xrat_2_(0) )/ (dt * dt);
			if( (vra(0) < 0.0) || (vra(0) > 0.5) )
				fra(0) = 0.0;
			else if ( fabs(axr > 2.3))
				fra(0) = (mx_*m_ * 2.2) + ( (cx_*c_/dt) * (xra(0) - xrat_1_(0)) );		
			

			double azl = ( xla(2) - (2*xlat_1_(2)) + xlat_2_(2) )/ (dt * dt);
			if( azl > 2.0 )						//max accleration in Z
				fla(2) = (mx_*m_ * 2.0) + ( (cx_*c_/dt) * (xla(2) - xlat_1_(2)) );
			else if( (azl < -2.5) && (xla(0)< xra(0)) )  
				fla(2) = ( (cx_*c_/dt) * (xla(2) - xlat_1_(2)) );

			//if( ((fla(0) > 50.0) || (fra(0) > 50.0)) && !walk_)
			if( ((vla(2) > 0.1) || (vra(2) > 0.1)) && ((xla(2) > 0.107) || (xra(2) > 0.107)) && !walk_)
			{
				walk_ = true;
				walkStop_ = false;
				res_ << "======> Started walking at : " << inTime*0.005 << ", realtime: " << realTime << std::endl;
			}
			else if( ( (((fabs(vla(2))) < 0.0001) && ((fabs(vra(2))) < 0.0001) && (elapsed_ > 45) ) || ((fabs(xla(0)-xra(0)) < 0.00001) && (fabs(vla(0)) < 0.001)) ) && walk_)
			{
				walkStop_ = true;
				walk_ = false;				
				elapsed_ = 0;
				res_ << "####### Finish walking at : " << inTime*0.005<< ", realtime: " << realTime << std::endl;
			}
			else if(!walk_)
			{
				for(unsigned i=0; i < 3; i++)
				{
					fla(i) = 0.0;
					fra(i) = 0.0;
				}
			}

			if( (xla(2) < 0.106) && (xra(2) < 0.106) && walk_ && !walkStop_)
			{
				fstatic(0) = walk_vel_*(2700/0.1);		//2800 for longHose
				//fstatic(2) = 100.0;
				if( (fla(0) > 250) || (fla(0) < -15) )		//Abrupt changes in xla or xra are little unwanted jumps in the feet
					fla(0) = 0.0;
				else if( (fra(0) > 250) || (fra(0) < -15) )
					fra(0) = 0.0;  //*/

				elapsed_ = elapsed_ + 1;
				if( (xla(2) > xra(2)) && (xla(2) > 0.10501) )
					fra.setZero();
				else if( (xra(2) > xla(2)) && (xra(2) > 0.10501) )
					fla.setZero();
				else
				{
					fla.setZero();
					fra.setZero();
				}
			}	
			else
			{
				elapsed_ = 0;
				for(unsigned i=0; i < 3; i++)
				  fstatic(i) = 0.0;

				if( xla(2) > 0.1055 )
				  fra.setZero();
				else if(xra(2) > 0.1055)
				  fla.setZero();
			}

			//Don't need to move the wrist in Z direction, when the right foot is up
			fra(2) = 0.0;

			//Abrupt changes in zla are little unwanted jumps in the feet
			if( (fla(2) > 150) || (fla(2) < -180))
			  fla(2) = 0.0;


			for(unsigned i=0; i < 3; i++)
			{
			  xg(i) = (((dt * dt) / m_ ) * (fr(i) - (fd_(i) - fla(i) - fra(i) - fstatic(i)) - ((c_/dt) * (xt(i) - xt_1_(i))) )) + (2 * xt(i)) - xt_1_(i);
			  //xg(i) = (((dt * dt) / m_ ) * (fr(i) - (fd_(i) - fla(i) - fstatic(i)) - ((c_/dt) * (xt(i) - xt_1_(i))) )) + (2 * xt(i)) - xt_1_(i);
			  //xcf(i) = (0.0220*xct_1_(i)) + (1.7787*xcft_1_(i)) - (0.8008*xcft_2_(i));
			  xcf(i) = (xg(i) + (2*xcft_1_(i)) + xcft_2_(i))/4;
			  xct_1_(i) = xg(i);
			  xcft_2_(i) = xcft_1_(i);
			  xcft_1_(i) = xcf(i);
			}

			//----In Z the filter is not neccesary (only pulling in X)
			xcf(2) = xg(2);

		   for(unsigned i=0; i < 3; i++)
		     imp(i) = ((m_ / (dt * dt) ) * (xg(i) - (2*xt(i)) + xt_1_(i) )) + ( (c_/dt) * (xt(i) - xt_1_(i)) );

		   df = fd_ - fla - fra - fstatic;

			if( (fabs(xt(0) - xt_1_(0)) > max_dx_) && (fabs(xreft_1_(0) - xcf(0)) < (fabs(xt(0) - xt_1_(0)))) )
			{
           		for(unsigned i=0; i < 3; i++)
					lw(i, 3) = xcf(i); //  ( xcf(i) + xreft_1_(i) + xt_1_(i) )/3;
			}
			else
			{
           		for(unsigned i=0; i < 3; i++)
					lw(i, 3) = ( xcf(i) + xreft_1_(i) + xt(i) )/3;
			}
				
		   // lw(i, 3) = 0.5 * (xg(i) + xct_1_(i));


		   //In Y, keep the distance at starting position with the waist, always!!
		   lw(1, 3) = qs(1) + dy_;

		   force_ << (inTime*0.005) << "	" << inTime; 
		   for(unsigned k=0; k < 3; ++k)
		     force_ << "	" << fr(k);  

		   for(unsigned k=0; k < 3; ++k)
		     force_ << "	" << fraw_(k);

		   //for(unsigned k=0; k < 3; ++k)
		     //force_ << "	" << ff2_(k);
		   force_  << "	" << realTime << std::endl;

		   res_ << inTime << "	" << realTime << "	" << lw << std::endl;
		   check_ << inTime << "	";
			double ccx = (c_/dt) * (xt(0) - xt_1_(0)), ccz = (c_/dt) * (xt(2) - xt_1_(2));
			double ddx = (2 * xt(0)) - xt_1_(0), ddz = (2 * xt(2)) - xt_1_(2);
			double mdx = (((dt * dt) / m_ ) * (fr(0) - (fd_(0) - fla(0) - fra(0)) - ccx));
			double mdz = (((dt * dt) / m_ ) * (fr(2) - (fd_(2) - fla(2) - fra(2)) - ccz));
		  	check_<< fla(0)<< "	" << fra(0)<< "	" << fr(0) - fd_(0)<< "	" << ccx << "	" << ddx << "	" << mdx << "	";
			check_<< fla(2)<< "	" << fra(2)<< "	" << fr(2) - fd_(2)<< "	" << ccz << "	" << ddz << "	" << mdz << "	" << realTime << std::endl;
	
		   pos_ << inTime << "	" << imp(0) << "	" << imp(1) << "	" << imp(2);
		   pos_ << "	" << df(0) << "	" << df(1) << "	" << df(2);
		   pos_ << "	" << realTime << std::endl;


			if( (fabs(lw(0,3) - xt(0)) > max_dx_ ) && ((xla(2) >= 0.1080) || (xra(2) >= 0.1080)) )
			{
			  lw(0,3) = xt(0) + max_dx_;
			  res_ << "--> vx exceeding max limit when wlkg!! changing to xt= " << lw(0,3) << std::endl;
			}
			else if( walkStop_ && (fabs(lw(0,3) - xt(0)) > 2*max_dx_) )
			{ 
			  if( (xt(0) + 2*max_dx_) < (qs(0)+0.15) )
				lw(0,3) = xt(0) + 2*max_dx_;
			  else
				lw(0,3) = qs(0) + 0.15;
			  res_ << "--> vx exceeding max limit!! changing to xt= " << lw(0,3) << std::endl;
			}

			if( lw(0,3) > (qs(0)+0.15) )
			{
			  if( (xla(2) >= 0.1060) && (vla(0) < 0.5) )
				lw(0,3) = xt(0) + fabs(xla(0) - xlat_1_(0));
			  else if( (xra(2) >= 0.1060) && (vra(0) < 0.5) )
				lw(0,3) = xt(0) + fabs(xra(0) - xrat_1_(0));
			  else
			  {
				if( (xla(2) < 0.1060) && (xra(2) < 0.1060) )
				{
				  if( walkStop_ || (xt(0) < xla(0)) )
					lw(0,3) = qs(0) + 0.15;
				  else
					lw(0,3) = (xreft_1_(0) + xt(0))/2;
				}
				else if( (xt(0) + max_dx_) < (qs(0)+0.15) )
				  lw(0,3) = xt(0) + max_dx_;
				else
				  lw(0,3) = qs(0) + 0.15;
			  }
			  res_ << "--> dx exceeding max limit!! changing to xt= " << lw(0,3) << std::endl;
			}
			

			if( (lw(0,3) < xla(0)) && (xla(2) < 0.106) && (xra(2) < 0.106) && walk_)
			{
				lw(0,3) = xla(0);
				res_ << "==> dx smaller than xla while in DST!! changing to xt= " << lw(0,3) << std::endl;
			}
	
		   if( inTime > 4500)
		   {
		     if( ((lw(2,3) < 0.695) || (lw(2,3) > 0.80)) && ((xla(2) >= 0.1060) || (xra(2) >= 0.01060) ))
		     {
		       lw(2,3) = (xt_1_(2) + xt(2) + xreft_1_(2))/3;   //before was using x(t)
		       res_ << "--> Z below 0.695 or over 0.730 while wkg, chaging z -> " << lw(2,3) << std::endl;
		     }  //*/
			 else if(((lw(2,3) < 0.65) || (lw(2,3) > 0.80)))
			 {
			   lw(2,3) = (2*xt_1_(2) + xt(2) + lw(2,3))/4;
				res_ << "----> Z below 0.65 or over 0.80, changing z to previous value -> " << lw(2,3) << std::endl;
			 }
		   }

		   wrist_ << inTime;
		   for(unsigned k=0; k < 3; ++k)
		     wrist_ << "	" << xt(k) << "	" << lw(k, 3) << "	" << xcf(k);
	
	   	   wrist_ << "	" << xla(0) << "	" << xra(0) << "	" << xla(1) << "	" << xra(1);
		   wrist_ << "	" << xla(2) << "	" << xra(2) << "	" << xg(0) << "	" << xg(2) << "	" << realTime << std::endl;

           for(unsigned i=0; i < 3; i++)
		     xreft_1_(i) = lw(i,3);


           lwct_1_ = lw;
		   xlat_2_ = xlat_1_;
		   xlat_1_ = xla;
		   xrat_2_ = xrat_1_;
		   xrat_1_ = xra;

           xt_1_ = xt;
         }
         return lw;
      }     

      Vector& ImpedanceController::computeSelection(Vector& selec, const int& inTime)
      {
        const Vector& force = forceSOUT.access(inTime);

		if(!start_)
        {
		  selec.resize(1);
          selec(0) = false;
        }
		else
        {
          selec(0) = false;

          for(unsigned i=0; i < 3; i++)
		  {
		    if( fabs( force(i) ) > 300.0)
		      selec(0) = true;
          }
        }
        return selec;
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
		else if(hold_)
		{
		   qt = qs;
		   qt(28) = 0.75;
		   qt(35) = 0.75;
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
	   res_ << "The Controller was stopped!! Hose released!! at t = " << (t_1_+1) << std::endl;
         }
      }

      DYNAMICGRAPH_FACTORY_ENTITY_PLUGIN(ImpedanceController, "ImpedanceController");
    } // namespace tools
  } //namespace sot
} // namespace dynamicgraph

