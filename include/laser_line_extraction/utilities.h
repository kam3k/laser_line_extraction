#ifndef LINE_EXTRACTION_UTILITIES_H
#define LINE_EXTRACTION_UTILITIES_H

#include <vector>
#include <cmath>

namespace line_extraction
{

struct CachedData
{
  std::vector<unsigned int> indices;
  std::vector<double> bearings;
  std::vector<double> cos_bearings;
  std::vector<double> sin_bearings;
};

struct RangeData
{
  std::vector<double> ranges;
  std::vector<double> xs;
  std::vector<double> ys;
};

struct Params
{
  double bearing_var;
  double range_var;
  double least_sq_angle_thresh;
  double least_sq_radius_thresh;
  double max_line_gap;
  double min_line_length;
  double min_range;
  double min_split_dist;
  double outlier_dist;
  unsigned int min_line_points;
};

struct PointParams
{
  std::vector<double> a;
  std::vector<double> ap;
  std::vector<double> app;
  std::vector<double> b;
  std::vector<double> bp;
  std::vector<double> bpp;
  std::vector<double> c;
  std::vector<double> s;
};

double pi_to_pi(double angle)
{
  angle = fmod(angle, 2 * M_PI);
  if (angle >= M_PI)
    angle -= 2 * M_PI;
  return angle;
}

} // namespace line_extraction

#endif