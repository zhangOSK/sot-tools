#ifndef _ELLIPSE_VECTOR_FIELD_NUMERIC_HH_
#define _ELLIPSE_VECTOR_FIELD_NUMERIC_HH_


namespace EllipseVectorFieldNumeric
{
  
  double datan2(double dx,double dy);
  
  class HopfParameters
  {
  protected:
    /// \brief zerokapparadius - the radius for which the unit circle system has 0 curvature
    double zerokapparadius_;

  public:
    /// \brief radiusx,radiusy - radii of limit cycle ellipse
    double radiusx_, radiusy_;
    /// \brief gamma - gain factor scalar
    double gamma_;
    /// \brief beta - power law exponent scalar
    double beta_;
    /// \brief omega - angular speed of the hopf system
    double omega_;
    /// \brief epsilon - infinitisimal step size for numeric kappa calculation
    double epsilon_;

  public:
    HopfParameters(double radiusx, double radiusy, double gamma, double beta, double zerokapparadius);

    void HopfVectorUnit(double x, double y, double &vx, double &vy);
    void NormVector(double x, double y, double &vx, double &vy);
    void EllipseNumeric(double x, double y, double &vx, double &vy);
    double zerokapparadius() const;
    void zerokapparadius(double );
    void save(std::string &filename) const;
    void save() const;
  };

  /**! \brief Generates a Vector field for an elliptic pattern with Hopf structure.
     output:
     evf is a struct containing parameters, vector fields and scalar
     functions on the space.
     evf.parameters. :
     inputs 
     omega - angular speed of the hopf system
     epsilon - infinitisimal step size for numeric kappa calculation
  */
  class EllipseVectorField
  {
    
  public:
    EllipseVectorField();
    EllipseVectorField(const HopfParameters &hopfParameters);
    EllipseVectorField(HopfParameters &hopfParameters);
    
    HopfParameters hopfParameters_;
      
  };
}
#endif
