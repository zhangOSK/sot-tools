#include <cmath>
#include <iostream>
#include <fstream>
#include <sstream>

#if 0
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x) 
#endif

#include "ellipse-vector-field-numeric.hh"

using namespace EllipseVectorFieldNumeric;

namespace EllipseVectorFieldNumeric
{
  double datan2(double dy,double dx)
  {
    if (dx==0.0)
      return 0.0;
    
    double res=0;
    double r= (dy/dx);
    res = 1.0/(1.0 + r*r);
    if ((dy<0.0) && (dx<0.0))
      res -= M_PI;
    if ((dy<0.0) && (dx>0.0))
      res = -res;
    if ((dy>0.0) && (dx<0.0))
      res = M_PI - res;  
    return res;
  }
}

void PLHopfVectorUnit(double x, double y, double gamma, double beta, double omega,
		 double &vx, double &vy)
{
  ODEBUG( "PLHopfVectorUnit: (x,y)=" << x << " " << y);

  if ((x==0.0) && (y==0.0))
    {
      vx=0.0; vy=0.0;
      return;
    }
  double m = x*x+y*y;
  ODEBUG("PLHopfVectorUnit: m=" << m);
  double c = gamma * pow(m,((beta-1.0)/2.0)) *
    pow(fabs(pow((m+1.0),2.0)+omega*omega),3*beta/2.0) /
    pow(fabs(omega*m*m-omega*omega*omega+omega),beta)/
    pow(fabs(1+omega-2*m+m*m),0.5);
  ODEBUG( "PLHopfVectorUnit: c=" << c);
  vx = c * ((1-m)*x + omega*y);
  vy = c * ((1-m)*y - omega*x);
  ODEBUG("PLHopfVectorUnit: (vx,vy) = (" << vx << "," << vy << ")");
}

void PLNormVector(double x, double y, double radiusx, double radiusy, double omega,
		  double &vx, double &vy)
{
  double beta=0.0;
  double gamma=1.0;
  ODEBUG("PLNormVectorUnit: radiusx = " << radiusx << ", radiusy = " << radiusy);
  PLHopfVectorUnit(x/radiusx, y/radiusy,gamma,beta, omega,vx,vy);
  ODEBUG("PLNormVectorUnit: 1/ (vxc,vyc) = (" << vx << "," << vy << ")");
  vx = radiusx * vx;
  vy = radiusy * vy;
  ODEBUG("PLNormVectorUnit: 2/ (vx,vy) = (" << vx << "," << vy << ")");
  double vsize = sqrt(vx*vx + vy*vy);
  ODEBUG("PLNormVectorUnit: 2.5/ vsize = (" << vsize << ")");
  if (vsize==0.0)
    return;
  vx = (1.0 / vsize)*vx;
  vy = (1.0 / vsize)*vy;
  ODEBUG("PLNormVectorUnit: 3/ (vx,vy) = (" << vx << "," << vy << ")");
}

void PLVectorFieldCurvature(double x, double y,
			    double radiusx, double radiusy, double omega, double epsilon,
			    double & kappa)
{
  double vx=0, vy=0;
  PLNormVector(x,y,radiusx, radiusy, omega,vx,vy);
  ODEBUG("PLVectorFieldCurvature: 1/ (vx,vy) = (" << vx << "," << vy << ")");
  double x2 = x + vx * epsilon;
  double y2 = y + vy * epsilon;
  ODEBUG("PLVectorFieldCurvature: 2/ (x2,y2) = (" << x2 << "," << y2 << ")");
  double vx2=0, vy2=0;
  PLNormVector(x2,y2,radiusx,radiusy,omega,vx2,vy2);
  ODEBUG("PLVectorFieldCurvature: 3/ (vx2,vy2) = (" << vx2 << "," << vy2 << ")");
  double ax = (vx2-vx)/epsilon;
  double ay = (vy2-vy)/epsilon;
  ODEBUG("PLVectorFieldCurvature: 4/ (ax,ay) = (" << ax << "," << ay << ")");
  double r = vx*vx+vy*vy;
  if (r==0.0)
    kappa = INFINITY;
  else 
    {
      kappa = (vx * ay - vy *ax)/pow(r,(3.0/2.0));
      if (isnan(kappa))
	kappa = 0.0;
    }
  ODEBUG("PLVectorFieldCurvature: 5/ kappa = (" << kappa << ")");
}

void PLNumericHopfVector(double x, double y, double radiusx, double radiusy, double gamma, double beta,double omega, double epsilon,
			 double &vx, double &vy)
{
  double kappa;
  ODEBUG("PLNumericHopfVector: 1/ (x,y) = (" << x << "," << y << ")");
  PLNormVector(x,y,radiusx,radiusy, omega,vx,vy);
  ODEBUG("PLNumericHopfVector: 2/ (vx,vy) = (" << vx << "," << vy << ")");
  PLVectorFieldCurvature(x,y,radiusx,radiusy,omega,epsilon,kappa);
  double c = gamma * pow(fabs(kappa),-beta);
  ODEBUG("PLNumericHopfVector: 3/ c = (" << c << ")");
  vx = c * vx;
  vy = c * vy;
  ODEBUG("PLNumericHopfVector: 4/ (vx,vy) = (" << vx << "," << vy << ")");
}

void InEllipseDonut(double x, double y, double radiusx, double radiusy, double ratioin, double ratioout,
		    bool &isin)
{
  double normradius = pow(x/radiusx,2.0) + pow(y/radiusy,2.0);
  isin = ((normradius<=ratioout*ratioout) & (normradius>=ratioin*ratioin));
}

HopfParameters::HopfParameters(double radiusx, double radiusy, double gamma, double beta, double zkpd):
zerokapparadius_(zkpd),radiusx_(radiusx),radiusy_(radiusy), gamma_(gamma), beta_(beta)
{
  zerokapparadius(zkpd); 
  epsilon_ = 0.001;
  ODEBUG("HopfParameters::HopfParameters:" << omega_ << " " << epsilon_ );
}

/* Implementation of Hopf Parameters */
void HopfParameters::HopfVectorUnit(double x, double y, double &vx, double &vy)
{
  PLHopfVectorUnit(x,y,gamma_,beta_,omega_,vx,vy);
}

void HopfParameters::NormVector(double x,double y, double &vx,double &vy)
{
  PLNormVector(x,y,radiusx_,radiusy_, omega_,vx,vy);
}

void HopfParameters::EllipseNumeric(double x, double y, double &vx, double &vy)
{
  ODEBUG("*************************************************************");
  PLNumericHopfVector(x,y,radiusx_,radiusy_,gamma_, beta_, omega_,epsilon_, vx, vy);
}

void HopfParameters::zerokapparadius(double zkpd)
{
  zerokapparadius_ = zkpd;
  omega_ = -sqrt(pow(zkpd,4.0)-1);
}

double HopfParameters::zerokapparadius() const
{
  return zerokapparadius_;
}

void HopfParameters::save() const
{
  std::string filename = "/tmp/debug";
  save(filename);
}

void HopfParameters::save(std::string &filename) const
{
  std::ofstream ofs;
  std::stringstream os;
  os << filename
     << "_RX_" << radiusx_
     << "_RY_" << radiusy_
     << "_G_" << gamma_
     << "_B_" << beta_
     << "_Z_" << zerokapparadius_;
  std::string dumpfilename = os.str();
  ofs.open(dumpfilename.c_str());
  if (ofs.is_open())
    {
      ofs << radiusx_ 
	  << " " << radiusy_
	  << " " << gamma_
	  << " " << beta_
	  << " " << zerokapparadius_
	  << " " << omega_
	  << " " << epsilon_
	  << std::endl;
      ofs.close();
    }

}
EllipseVectorField::EllipseVectorField():
  //hopfParameters_(2.0,1.0,1.0,0.33333,2.0)
  // test hopfParameters_(2.0,1.0,1.0,0.0,2.0)
  hopfParameters_(1.75,0.9,1.0,0.0,1.05)
{
}

EllipseVectorField::EllipseVectorField(const HopfParameters &hopfParameters):
  hopfParameters_(hopfParameters)
{
}

EllipseVectorField::EllipseVectorField(HopfParameters &hopfParameters):
  hopfParameters_(hopfParameters)
{
}



