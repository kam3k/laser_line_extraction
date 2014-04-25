#include <algorithm> // std::max
#include "laser_line_extraction/line_extraction.h"
#include <iostream>

namespace line_extraction
{

/*************************************************************
 * Line
*************************************************************/

// Constructor / destructor

Line::Line(const CachedData &c_data, const RangeData &r_data, const Params &params, 
           std::vector<unsigned int> indices):
  c_data_(c_data),
  r_data_(r_data),
  params_(params),
  indices_(indices)
{
}

Line::~Line()
{
}

// Get methods for line parameters

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

double Line::getRadius() const
{
  return radius_;
}

const boost::array<double, 2>& Line::getStart() const
{
  return start_;
}

// Methods for line fitting

double Line::distToPoint(unsigned int index)
{
  double p_rad = sqrt(pow(r_data_.xs[index], 2) + pow(r_data_.ys[index], 2));
  double p_ang = atan2(r_data_.ys[index], r_data_.xs[index]);
  return fabs(p_rad * cos(p_ang - angle_) - radius_);
}

void Line::endpointFit()
{
  start_[0] = r_data_.xs[indices_[0]]; 
  start_[1] = r_data_.ys[indices_[0]]; 
  end_[0] = r_data_.xs[indices_.back()]; 
  end_[1] = r_data_.ys[indices_.back()]; 
  angleFromEndpoints();
  radiusFromEndpoints();
}

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

double Line::length() const
{
  return sqrt(pow(start_[0] - end_[0], 2) + pow(start_[1] - end_[1], 2));
}

unsigned int Line::numPoints() const
{
  return indices_.size();  
}

// Private methods

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

void Line::radiusFromEndpoints()
{
  radius_ = start_[0] * cos(angle_) + start_[1] * sin(angle_);
  if (radius_ < 0)
  {
    radius_ = -radius_;
    angle_ = pi_to_pi(angle_ + M_PI);
  }
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

/*************************************************************
 * LineExtraction
*************************************************************/

// Constructor / destructor

LineExtraction::LineExtraction()
{
}

LineExtraction::~LineExtraction()
{
}

// Run

void LineExtraction::extractLines(std::vector<Line>& lines) 
{
  // Resets
  filtered_indices_ = c_data_.indices;
  lines_.clear();

  // Filter indices
  filterClosePoints();
  filterOutlierPoints();

  // Return no lines if not enough points left
  if (filtered_indices_.size() <= std::max(params_.min_line_points, static_cast<unsigned int>(3)))
  {
    return;
  }

  // Split indices into lines and filter out short and sparse lines
  split(filtered_indices_);
  filterLines();

  // Fit each line using least squares and merge colinear lines
  for (std::vector<Line>::iterator it = lines_.begin(); it != lines_.end(); ++it)
  {
    it->leastSqFit();
  }
  mergeLines();

  lines = lines_;
}

// Data setting

void LineExtraction::setCachedData(const std::vector<double>& bearings,
                                   const std::vector<double>& cos_bearings,
                                   const std::vector<double>& sin_bearings,
                                   const std::vector<unsigned int>& indices)
{
  c_data_.bearings = bearings;
  c_data_.cos_bearings = cos_bearings;
  c_data_.sin_bearings = sin_bearings;
  c_data_.indices = indices;
}

void LineExtraction::setRangeData(const std::vector<double>& ranges)
{
  r_data_.ranges = ranges;
  r_data_.xs.clear();
  r_data_.ys.clear();
  for (std::vector<unsigned int>::const_iterator cit = c_data_.indices.begin(); 
       cit != c_data_.indices.end(); ++cit)
  {
    r_data_.xs.push_back(c_data_.cos_bearings[*cit] * ranges[*cit]);
    r_data_.ys.push_back(c_data_.sin_bearings[*cit] * ranges[*cit]);
  }
}

// Parameter setting

void LineExtraction::setBearingVariance(double value)
{
  params_.bearing_var = value;
}

void LineExtraction::setRangeVariance(double value)
{
  params_.range_var = value;
}

void LineExtraction::setLeastSqAngleThresh(double value)
{
  params_.least_sq_angle_thresh = value;
}

void LineExtraction::setLeastSqRadiusThresh(double value)
{
  params_.least_sq_radius_thresh = value;
}

void LineExtraction::setMaxLineGap(double value)
{
  params_.max_line_gap = value;
}

void LineExtraction::setMinLineLength(double value)
{
  params_.min_line_length = value;
}

void LineExtraction::setMinLinePoints(unsigned int value)
{
  params_.min_line_points = value;
}

void LineExtraction::setMinRange(double value)
{
  params_.min_range = value;
}

void LineExtraction::setMinSplitDist(double value)
{
  params_.min_split_dist = value;
}

void LineExtraction::setOutlierDist(double value)
{
  params_.outlier_dist = value;
}

// Private methods

double LineExtraction::distBetweenPoints(unsigned int index_1, unsigned int index_2)
{
  return sqrt(pow(r_data_.xs[index_1] - r_data_.xs[index_2], 2) + 
              pow(r_data_.ys[index_1] - r_data_.ys[index_2], 2));
}

void LineExtraction::filterClosePoints()
{
  std::vector<unsigned int> output;
  for (std::vector<unsigned int>::const_iterator cit = filtered_indices_.begin(); 
       cit != filtered_indices_.end(); ++cit)
  {
    if (r_data_.ranges[*cit] >= params_.min_range)
    {
      output.push_back(*cit);
    }
  }
  filtered_indices_ = output;
}

void LineExtraction::filterOutlierPoints()
{
  if (filtered_indices_.size() < 3)
  {
    return;
  }

  std::vector<unsigned int> output;
  unsigned int p_i, p_j, p_k;
  for (std::size_t i = 0; i < filtered_indices_.size(); ++i)
  {

    // Get two closest neighbours

    p_i = filtered_indices_[i];
    if (i == 0) // first point
    {
      p_j = filtered_indices_[i + 1];
      p_k = filtered_indices_[i + 2];
    }
    else if (i == filtered_indices_.size() - 1) // last point
    {
      p_j = filtered_indices_[i - 1];
      p_k = filtered_indices_[i - 2];
    }
    else // middle points
    {
      p_j = filtered_indices_[i - 1];
      p_k = filtered_indices_[i + 1];
    }

    // Check if point is an outlier

    if (fabs(r_data_.ranges[p_i] - r_data_.ranges[p_j]) > params_.outlier_dist &&
        fabs(r_data_.ranges[p_i] - r_data_.ranges[p_k]) > params_.outlier_dist) 
    {
      // Check if it is close to line connecting its neighbours
      std::vector<unsigned int> line_indices;
      line_indices.push_back(p_j);
      line_indices.push_back(p_k);
      Line line(c_data_, r_data_, params_, line_indices);
      line.endpointFit();
      if (line.distToPoint(p_i) > params_.min_split_dist)
      {
        continue; // point is an outlier
      }
    }

    output.push_back(p_i);
  }

  filtered_indices_ = output;
}

void LineExtraction::filterLines()
{
  std::vector<Line> output;
  for (std::vector<Line>::const_iterator cit = lines_.begin(); cit != lines_.end(); ++cit)
  {
    if (cit->length() >= params_.min_line_length && cit->numPoints() >= params_.min_line_points)
    {
      output.push_back(*cit);
    }
  }
  lines_ = output;
}

void LineExtraction::mergeLines()
{
  
}

void LineExtraction::split(const std::vector<unsigned int>& indices)
{
  // Don't split if only a single point (only occurs when orphaned by gap)
  if (indices.size() <= 1)
  {
    return;
  }

  Line line(c_data_, r_data_, params_, indices);
  line.endpointFit();
  double dist_max = 0;
  double gap_max = 0;
  double dist, gap;
  int i_max, i_gap;

  // Find the farthest point and largest gap
  for (std::size_t i = 1; i < indices.size() - 1; ++i)
  {
    dist = line.distToPoint(indices[i]);
    if (dist > dist_max)
    {
      dist_max = dist;
      i_max = i;
    }
    gap = distBetweenPoints(indices[i], indices[i+1]);
    if (gap > gap_max)
    {
      gap_max = gap;
      i_gap = i;
    }
  }

  // Check for gaps at endpoints
  double gap_start = distBetweenPoints(indices[0], indices[1]);
  if (gap_start > gap_max)
  {
    gap_max = gap_start;
    i_gap = 1;
  }
  double gap_end = distBetweenPoints(indices.rbegin()[1], indices.rbegin()[0]);
  if (gap_end > gap_max)
  {
    gap_max = gap_end;
    i_gap = indices.size() - 1;
  }

  // Check if line meets requirements or should be split
  if (dist_max < params_.min_split_dist && gap_max < params_.max_line_gap)
  {
    lines_.push_back(line);
  }
  else
  {
    int i_split;
    if (dist_max >= params_.min_split_dist)
    {
      i_split = i_max;
    }
    else
    {
      i_split = i_gap;
    }
    std::vector<unsigned int> first_split(&indices[0], &indices[i_split - 1]);
    std::vector<unsigned int> second_split(&indices[i_split], &indices.back());
    split(first_split);
    split(second_split);
  }

}

/*************************************************************
 * Utility functions
*************************************************************/

double pi_to_pi(double angle)
{
  angle = fmod(angle, 2 * M_PI);
  if (angle >= M_PI)
    angle -= 2 * M_PI;
  return angle;
}

} // namespace line_extraction
