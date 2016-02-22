/*
 * Copyright 2015, LAAS CNRS
 *
 * Author: Olivier Stasse
 * 
 *
 * This file is part of walkGenJrl.
 * walkGenJrl is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * walkGenJrl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Lesser Public License for more details.
 * You should have received a copy of the GNU Lesser General Public License
 * along with walkGenJrl.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
#ifndef _POWER_LAW_HH_
#define _POWER_LAW_HH_

#include <vector>
#include <string>
#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include <dynamic-graph/entity.h>
#include "ellipse-vector-field-numeric.hh"

template <typename EntityClass>
/// \brief Class loading a velocity profile generated to simulate the power law.
class PowerLaw
{

protected: 
  /// Link to the entity object.
  EntityClass *Entity_;

  /// Array of velocities.
  typedef std::vector<double> vector_double_t;
  /// Array of arrays
  std::vector<vector_double_t> queueOfVelocity_;
  /// Index in the queue of velocities.
  unsigned long int indexQueueVelocity_;
  /// Stop has been reached
  bool power_law_stop_;
  /// Stop iteration.
  unsigned long int power_law_stop_it_;
  /// Store Heading Angle.
  double heading_angle_;
  /// Store internal time.
  double internal_time_;
  /// Hopf oscillator based ellipse vector field generator.
  EllipseVectorFieldNumeric::EllipseVectorField ellipseVectorField_;
  /// PID control
  double error_[4],integral_;
  
public:
  /// \brief Constructor of the class Power Law.
  PowerLaw(EntityClass * lVelocityFromPowerLaw,
	       EllipseVectorFieldNumeric::HopfParameters & hp);

  /// \brief Constructor of the class Power Law.
  PowerLaw(EntityClass * lVelocityFromPowerLaw);

  /// \brief Generate velocities according to the power law.
  Eigen::Vector3d generateVelocityFromPowerLawVectorField(
      double time, double ctheta,
      double lfx, double lfy,
      double rfx, double rfy);

  EllipseVectorFieldNumeric::HopfParameters * getEllipse()
  {
    return &(ellipseVectorField_);
  }

protected:
  /// \brief Normalize angle into 0 2*PI.
  double normalize_angle(double langle);
  /// \brief Implemente a PID controller.
  double PID(double* e, double& integral );

};

#include "PowerLaw.hpp"
#endif

