#include "laser_line_extraction/line.h"

namespace line_extraction
{

///////////////////////////////////////////////////////////////////////////////
// Constructor / destructor
///////////////////////////////////////////////////////////////////////////////
Line::Line(const CachedData &c_data, const RangeData &r_data, const Params &params, 
           std::vector<unsigned int> indices):
  c_data_(c_data),
  r_data_(r_data),
  params_(params),
  indices_(indices)
{
}

Line::Line(double angle, double radius, const boost::array<double, 4> &covariance,
       const boost::array<double, 2> &start, const boost::array<double, 2> &end,
       const std::vector<unsigned int> &indices):
  angle_(angle),
  radius_(radius),
  covariance_(covariance),
  start_(start),
  end_(end),
  indices_(indices)
{
}

Line::~Line()
{
}

///////////////////////////////////////////////////////////////////////////////
// Get methods for line parameters
///////////////////////////////////////////////////////////////////////////////
double Line::getAngle() const
{
  return angle_;
}

const boost::array<double, 4>& Line::getCovariance() const
{
  return covariance_;
}

const boost::array<double, 2>& Line::getEnd() const
{
  return end_;
}

const std::vector<unsigned int>& Line::getIndices() const
{
  return indices_;
}

double Line::getRadius() const
{
  return radius_;
}

const boost::array<double, 2>& Line::getStart() const
{
  return start_;
}

///////////////////////////////////////////////////////////////////////////////
// Utility methods
///////////////////////////////////////////////////////////////////////////////
double Line::distToPoint(unsigned int index)
{
  double p_rad = sqrt(pow(r_data_.xs[index], 2) + pow(r_data_.ys[index], 2));
  double p_ang = atan2(r_data_.ys[index], r_data_.xs[index]);
  return fabs(p_rad * cos(p_ang - angle_) - radius_);
}

double Line::length() const
{
  return sqrt(pow(start_[0] - end_[0], 2) + pow(start_[1] - end_[1], 2));
}

unsigned int Line::numPoints() const
{
  return indices_.size();  
}

void Line::projectEndpoints()
{
  double s = -1.0 / tan(angle_);
  double b = radius_ / sin(angle_);
  double x = start_[0];
  double y = start_[1];
  start_[0] = (s * y + x - s * b) / (pow(s, 2) + 1);
  start_[1] = (pow(s, 2) * y + s * x + b) / (pow(s, 2) + 1);
  x = end_[0];
  y = end_[1];
  end_[0] = (s * y + x - s * b) / (pow(s, 2) + 1);
  end_[1] = (pow(s, 2) * y + s * x + b) / (pow(s, 2) + 1);
}

///////////////////////////////////////////////////////////////////////////////
// Methods for endpoint line fitting
///////////////////////////////////////////////////////////////////////////////
void Line::endpointFit()
{
  start_[0] = r_data_.xs[indices_[0]]; 
  start_[1] = r_data_.ys[indices_[0]]; 
  end_[0] = r_data_.xs[indices_.back()]; 
  end_[1] = r_data_.ys[indices_.back()]; 
  angleFromEndpoints();
  radiusFromEndpoints();
}

void Line::angleFromEndpoints()
{
  double slope;
  if (fabs(end_[0] - start_[0]) > 1e-9)
  {
    slope = (end_[1] - start_[1]) / (end_[0] - start_[0]);
    angle_ = pi_to_pi(atan(slope) + M_PI/2);
  }
  else
  {
    angle_ = 0.0;
  }
}

void Line::radiusFromEndpoints()
{
  radius_ = start_[0] * cos(angle_) + start_[1] * sin(angle_);
  if (radius_ < 0)
  {
    radius_ = -radius_;
    angle_ = pi_to_pi(angle_ + M_PI);
  }
}

///////////////////////////////////////////////////////////////////////////////
// Methods for least squares line fitting
///////////////////////////////////////////////////////////////////////////////
void Line::leastSqFit()
{
  calcPointCovariances();
  double prev_radius = 0.0;
  double prev_angle = 0.0;
  while (fabs(radius_ - prev_radius) > params_.least_sq_radius_thresh ||
         fabs(angle_ - prev_angle) > params_.least_sq_angle_thresh) 
  {
    prev_radius = radius_;
    prev_angle = angle_;
    calcPointScalarCovariances();
    radiusFromLeastSq();
    angleFromLeastSq();
  }
  calcCovariance();
  projectEndpoints();
}

void Line::angleFromLeastSq()
{
  calcPointParameters();
  angle_ += angleIncrement();
}

double Line::angleIncrement()
{
  const std::vector<double> &a = p_params_.a;
  const std::vector<double> &ap = p_params_.ap;
  const std::vector<double> &app = p_params_.app;
  const std::vector<double> &b = p_params_.b;
  const std::vector<double> &bp = p_params_.bp;
  const std::vector<double> &bpp = p_params_.bpp;
  const std::vector<double> &c = p_params_.c;
  const std::vector<double> &s = p_params_.s;

  double numerator = 0; 
  double denominator = 0;
  for (std::size_t i = 0; i < a.size(); ++i)
  {
    numerator += (b[i] * ap[i] - a[i] * bp[i]) / pow(b[i], 2);
    denominator += ((app[i] * b[i] - a[i] * bpp[i]) * b[i] - 
                    2 * (ap[i] * b[i] - a[i] * bp[i]) * bp[i]) / pow(b[i], 3);
  }
  return -(numerator/denominator);
}

void Line::calcCovariance()
{
  covariance_[0] = p_rr_;

  const std::vector<double> &a = p_params_.a;
  const std::vector<double> &ap = p_params_.ap;
  const std::vector<double> &app = p_params_.app;
  const std::vector<double> &b = p_params_.b;
  const std::vector<double> &bp = p_params_.bp;
  const std::vector<double> &bpp = p_params_.bpp;
  const std::vector<double> &c = p_params_.c;
  const std::vector<double> &s = p_params_.s;

  double G = 0;
  double A = 0;
  double B = 0;
  double r, phi;
  for (std::size_t i = 0; i < a.size(); ++i)
  {
    r = r_data_.ranges[indices_[i]]; // range
    phi = c_data_.bearings[indices_[i]]; // bearing
    G += ((app[i] * b[i] - a[i] * bpp[i]) * b[i] - 2 * (ap[i] * b[i] - a[i] * bp[i]) * bp[i]) / pow(b[i], 3);
    A += 2 * r * sin(angle_ - phi) / b[i];
    B += 4 * pow(r, 2) * pow(sin(angle_ - phi), 2) / b[i];
  }
  covariance_[1] = p_rr_ * A / G;
  covariance_[2] = covariance_[1];
  covariance_[3] = pow(1.0 / G, 2) * B;
}

void Line::calcPointCovariances()
{
  point_covs_.clear();
  double r, phi, var_r, var_phi;
  for (std::vector<unsigned int>::const_iterator cit = indices_.begin(); cit != indices_.end(); ++cit)
  {
    r = r_data_.ranges[*cit]; // range
    phi = c_data_.bearings[*cit]; // bearing
    var_r = params_.range_var; // range variance
    var_phi = params_.bearing_var; // bearing variance
    boost::array<double, 4> Q; 
    Q[0] = pow(r, 2) * var_phi * pow(sin(phi), 2) + var_r * pow(cos(phi), 2);
    Q[1] = -pow(r, 2) * var_phi * sin(2 * phi) / 2.0 + var_r * sin(2 * phi) / 2.0;
    Q[2] = Q[1]; 
    Q[3] = pow(r, 2) * var_phi * pow(cos(phi), 2) + var_r * pow(sin(phi), 2);
    point_covs_.push_back(Q);
  }
}

void Line::calcPointParameters()
{
  p_params_.a.clear();
  p_params_.ap.clear();
  p_params_.app.clear();
  p_params_.b.clear();
  p_params_.bp.clear();
  p_params_.bpp.clear();
  p_params_.c.clear();
  p_params_.s.clear();

  double r, phi, var_r, var_phi;
  double a, ap, app, b, bp, bpp, c, s;
  for (std::vector<unsigned int>::const_iterator cit = indices_.begin(); cit != indices_.end(); ++cit)
  {
    r = r_data_.ranges[*cit]; // range
    phi = c_data_.bearings[*cit]; // bearing
    var_r = params_.range_var; // range variance
    var_phi = params_.bearing_var; // bearing variance
    c = cos(angle_ - phi);
    s = sin(angle_ - phi);
    a = pow(r * c - radius_, 2);
    ap = -2 * r * s * (r * c - radius_);
    app = 2 * pow(r, 2) * pow(s, 2) - 2 * r * c * (r * c - radius_);
    b = var_r * pow(c, 2) + var_phi * pow(r, 2) * pow(s, 2);
    bp = 2 * (pow(r, 2) * var_phi - var_r) * c * s;
    bpp = 2 * (pow(r, 2) * var_phi - var_r) * (pow(c, 2) - pow(s, 2));
    p_params_.a.push_back(a);
    p_params_.ap.push_back(ap);
    p_params_.app.push_back(app);
    p_params_.b.push_back(b);
    p_params_.bp.push_back(bp);
    p_params_.bpp.push_back(bpp);
    p_params_.c.push_back(c);
    p_params_.s.push_back(s);
  }
}

void Line::calcPointScalarCovariances()
{
  point_scalar_vars_.clear();
  double P;
  double inverse_P_sum = 0;
  for (std::vector<boost::array<double, 4> >::const_iterator cit = point_covs_.begin();
       cit != point_covs_.end(); ++cit)
  {
    P = (*cit)[0] * pow(cos(angle_), 2) + 2 * (*cit)[1] * sin(angle_) * cos(angle_) +
        (*cit)[3] * pow(sin(angle_), 2);
    inverse_P_sum += 1.0 / P;
    point_scalar_vars_.push_back(P);
  }
  p_rr_ = 1.0 / inverse_P_sum;
}

void Line::radiusFromLeastSq()
{
  radius_ = 0;
  double r, phi;
  for (std::vector<unsigned int>::const_iterator cit = indices_.begin(); cit != indices_.end(); ++cit)
  {
    r = r_data_.ranges[*cit]; // range
    phi = c_data_.bearings[*cit]; // bearing
    radius_ += r * cos(angle_ - phi) / point_scalar_vars_[cit - indices_.begin()]; // cit to index
  }
  
  radius_ *= p_rr_;
}

} // namespace line_extraction
