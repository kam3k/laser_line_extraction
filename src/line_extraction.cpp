#include <algorithm> // std::max
#include "laser_line_extraction/line_extraction.h"
#include <iostream>

namespace line_extraction
{

/*************************************************************
 * Line
*************************************************************/

// Constructor / destructor

Line::Line(std::vector<unsigned int> indices):
  line_indices_(indices)
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

void Line::endpointFit()
{
  
}

void Line::leastSqFit()
{
  
}

double Line::length() const
{
  return sqrt(pow(start_[0] - end_[0], 2) + pow(start_[1] - end_[1], 2));
}

unsigned int Line::numPoints() const
{
  return line_indices_.size();  
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

}

double Line::angleIncrement()
{
  
}

void Line::calcCovariance()
{

}

void Line::calcPointCovariances()
{
  
}

void Line::calcPointParameters()
{

}

void Line::calcPointScalarCovariances()
{
  
}

void Line::projectEndpoints()
{

}

void Line::radiusFromEndpoints()
{
  
}

void Line::radiusFromLeastSq()
{

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
  // Filter indices
  filterClosePoints();
  std::cout << "before filter: " << filtered_indices_.size() << " points" << std::endl;
  filterOutlierPoints();
  std::cout << "after filter: " << filtered_indices_.size() << " points" << std::endl;

  // Return no lines if not enough points left
  if (indices_.size() <= std::max(min_line_points_, static_cast<unsigned int>(3)))
    return

  // Split indices into lines and filter out short and sparse lines
  split(indices_);
  filterShortLines();
  filterSparseLines();

  // Fit each line using least squares and merge colinear lines
  for (std::vector<Line>::iterator it = lines_.begin(); it != lines_.end(); ++it)
    it->leastSqFit();
  mergeLines();

  lines = lines_;
}

// Data setting

void LineExtraction::setCachedData(const std::vector<double>& bearings,
                                   const std::vector<double>& cos_bearings,
                                   const std::vector<double>& sin_bearings,
                                   const std::vector<unsigned int>& indices)
{
  bearings_ = bearings;
  cos_bearings_ = cos_bearings;
  sin_bearings_ = sin_bearings;
  indices_ = indices;
  if (!range_std_dev_as_fraction_) // range variances can be cached
  {
    std::vector<double> range_vars(indices_.size(), range_std_dev_ * range_std_dev_);
    range_vars = range_vars;
  }
}

void LineExtraction::setRangeData(const std::vector<double>& ranges)
{
  ranges_ = ranges;
  filtered_indices_ = indices_;
  xs_.clear();
  ys_.clear();
  for (std::vector<unsigned int>::const_iterator cit = indices_.begin(); cit != indices_.end(); ++cit)
  {
    xs_.push_back(cos_bearings_[*cit] * ranges_[*cit]);
    ys_.push_back(sin_bearings_[*cit] * ranges_[*cit]);
  }

  if (range_std_dev_as_fraction_)
  {
    range_vars_.clear();
    for (std::vector<double>::const_iterator cit = ranges_.begin(); cit != ranges_.end(); ++cit)
      range_vars_.push_back((*cit * range_std_dev_) * (*cit * range_std_dev_));
  }
}

// Parameter setting

void LineExtraction::setBearingVariance(double value)
{
  bearing_var_ = value;
}

void LineExtraction::setLeastSqAngleThresh(double value)
{
  least_sq_angle_thresh_ = value;
}

void LineExtraction::setLeastSqRadiusThresh(double value)
{
  least_sq_radius_thresh_ = value;
}

void LineExtraction::setMaxLineGap(double value)
{
  max_line_gap_ = value;
}

void LineExtraction::setMinLineLength(double value)
{
  min_line_length_ = value;
}

void LineExtraction::setMinLinePoints(unsigned int value)
{
  min_line_points_ = value;
}

void LineExtraction::setMinRange(double value)
{
  min_range_ = value;
}

void LineExtraction::setMinSplitDist(double value)
{
  min_split_dist_ = value;
}

void LineExtraction::setOutlierDist(double value)
{
  outlier_dist_ = value;
}

void LineExtraction::setRangeStdDev(double value)
{
  range_std_dev_ = value;
}

void LineExtraction::setRangeStdDevAsFraction(bool value)
{
  range_std_dev_as_fraction_ = value;
}

// Private methods

double LineExtraction::distBetweenPoints(unsigned int index_1, unsigned int index_2)
{
  return sqrt(pow(xs_[index_1] - xs_[index_2], 2) + pow(ys_[index_1] - ys_[index_2], 2));
}

void LineExtraction::filterClosePoints()
{
  std::vector<unsigned int> output;
  for (std::vector<unsigned int>::const_iterator cit = filtered_indices_.begin(); 
       cit != filtered_indices_.end(); ++cit)
  {
    if (ranges_[*cit] >= min_range_)
    {
      output.push_back(*cit);
    }
  }
  filtered_indices_ = output;
}

void LineExtraction::filterOutlierPoints()
{
  if (filtered_indices_.size() < 3)
    return;

  std::vector<unsigned int> output;
  unsigned int p_i, p_j, p_k;
  for (std::size_t i = 0; i < filtered_indices_.size(); ++i)
  {

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

    if (fabs(ranges_[p_i] - ranges_[p_j]) < outlier_dist_ &&
        fabs(ranges_[p_i] - ranges_[p_k]) < outlier_dist_) 
    {
      output.push_back(p_i);
    }
  }
  filtered_indices_ = output;
}

void LineExtraction::filterShortLines()
{
  
}

void LineExtraction::filterSparseLines()
{

}

void LineExtraction::mergeLines()
{
  
}

void LineExtraction::split(const std::vector<unsigned int>& indices)
{

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
