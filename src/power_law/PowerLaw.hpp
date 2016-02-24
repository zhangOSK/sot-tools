#ifndef POWER_LAW_HPP
#define POWER_LAW_HPP

#include<cmath>
typedef double FloatType;

using namespace::EllipseVectorFieldNumeric;

#ifdef DEBUG
#define ODEBUG(x) std::cout << x << std::endl
#else
#define ODEBUG(x) 
#endif

/// \brief Constructor of the class Power Law.
template <typename EntityClass>
PowerLaw<EntityClass>::
PowerLaw(EntityClass *lEntityClass,
             HopfParameters &hp):
  Entity_(lEntityClass),
  indexQueueVelocity_(0),
  power_law_stop_(false),
  power_law_stop_it_(0),
  heading_angle_(M_PI/2.0),
  internal_time_(0.0),
  ellipseVectorField_(hp),
  integral_(0.0)
{
  ellipseVectorField_.hopfParameters_.save();
  for(unsigned i=0; i<4 ;++i)
    error_[i]=0.0;
}

/// \brief Constructor of the class Power Law.
template <typename EntityClass>
PowerLaw<EntityClass>::
PowerLaw(EntityClass *lEntityClass):
  Entity_(lEntityClass),
  indexQueueVelocity_(0),
  power_law_stop_(false),
  power_law_stop_it_(0),
  heading_angle_(M_PI/2.0),
  internal_time_(0.0),
  ellipseVectorField_(),
  integral_(0.0)
{
  for(unsigned i=0; i<4 ;++i)
    error_[i]=0.0;
}

template <typename VelocityFromPowerLawClass>
double PowerLaw<VelocityFromPowerLawClass>::
normalize_angle(double langle)
{
  double res=fmod(langle,2.0*M_PI);
  if (res>M_PI)
    res = -M_PI+fmod(langle,M_PI);
  if (res<-M_PI)
    res = M_PI+fmod(langle,M_PI);
  return res;
}

template <typename VelocityFromPowerLawClass>
double PowerLaw<VelocityFromPowerLawClass>::
PID(double* e, double& integral )
{
  double Kp_(1.5),Ki_(1.0),Kd_(1.5),T(0.005);
  double Saturation_integral = 0.5;
  double error = e[0];
  double derivate = ( e[0] + 3*e[1] - 3*e[2] - e[3] ) / (6*T);
  integral += e[0]*T;

  if(Ki_!=0.0)
  {
    if(integral*Ki_>Saturation_integral)
      integral = Saturation_integral/Ki_;
    if(integral*Ki_<-Saturation_integral)
      integral = -Saturation_integral/Ki_;
  }

  double v = Kp_ * error +
             Kd_ * derivate +
             Ki_ * integral;

//  if (std::abs(v)>0.4)
//    if(v>0)
//      v=0.4;
//    if(v<0)
//      v=-0.4;

  e[1] = e[0];
  e[2] = e[1];
  e[3] = e[2];

  return v;
}

template <typename VelocityFromPowerLawClass>
Eigen::Vector3d PowerLaw<VelocityFromPowerLawClass>::
generateVelocityFromPowerLawVectorField(double time, double ctheta,
                                        double lfx, double lfy,
                                        double rfx, double rfy)
{
  internal_time_=time;
  Eigen::Vector3d returnedVelocity (0.0,0.0,0.0);

  double px((lfx+rfx)*0.5);
  double py((lfy+rfy)*0.5);

  vector_double_t VelProf(3);

  // Initialize VelProf
  for(unsigned int li=0;li<2;li++)
    VelProf[li]=0.0;

  //double local_x = -cy+ellipseVectorField_.hopfParameters_.radiusx_;
  //double local_y =  cx;
  double local2_x = -py+ellipseVectorField_.hopfParameters_.radiusx_;
  double local2_y =  px;

  double nctheta = normalize_angle(ctheta);
  heading_angle_ = normalize_angle(nctheta+M_PI/2.0);// try update before ***

  ellipseVectorField_.hopfParameters_.EllipseNumeric(local2_x,local2_y,VelProf[0],VelProf[1]);

  // Integrate over the heading angle.
  double theta = atan2(VelProf[1],VelProf[0]);
  VelProf[2] = (theta-heading_angle_);
  if(std::abs(VelProf[2])>M_PI)
  {
    if(VelProf[2]<-M_PI/2)
      VelProf[2]+=2*M_PI;
    if(VelProf[2]>M_PI/2)
      VelProf[2]-=2*M_PI;
  }
  VelProf[2] = atan2(sin(VelProf[2]), cos(VelProf[2]));
  error_[0]=VelProf[2];
  VelProf[2] = PID(error_,integral_);

  double c = std::cos(heading_angle_);
  double s = std::sin(heading_angle_);
  Eigen::Matrix3d R;
  R << c  ,  -s, 0.0,
       s  ,   c, 0.0,
       0.0, 0.0, 1.0;

  Eigen::Vector3d w = Eigen::Vector3d(0.0, 0.0, VelProf[2]);
  Eigen::Vector3d v = Eigen::Vector3d(VelProf[0], VelProf[1], 0.0);

  Eigen::Matrix<double,3,1> w2;
  Eigen::Matrix<double,3,1> v2;
  w2 = R.transpose()*w;
  v2 = R.transpose()*v;

  returnedVelocity(0) = v2[0];
  returnedVelocity(1) = v2[1];
  returnedVelocity(2) = w2[2];

  {
    // Thresholding the vector field.
    double MaxVel[3] = { 0.4, 0.2 , 100000.0};
    double ScaleFactorXY=1.0;
//    double ScaleFactorYaw=1.0;
    ScaleFactorXY = std::max(fabs(returnedVelocity(0)/MaxVel[0]),
        fabs(returnedVelocity(1)/MaxVel[1]));
    if (ScaleFactorXY>1.0) // need to threshold
    {
      returnedVelocity(0) = returnedVelocity(0)/ScaleFactorXY;
      returnedVelocity(1) = returnedVelocity(1)/ScaleFactorXY;
    }
//    ScaleFactorYaw = fabs(returnedVelocity(2))/MaxVel[2];
//    if(ScaleFactorYaw > 1)
//    {
//      returnedVelocity(2)=returnedVelocity(2)/ScaleFactorYaw;
//    }
  }

  ODEBUG("end of generateEventPowerLawVelocity.");
  return returnedVelocity;
}

#endif //POWER_LAW_HPP
