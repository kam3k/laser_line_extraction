#ifndef LINE_EXTRACTION_H
#define LINE_EXTRACTION_H

#include <cmath>
#include <vector>
#include <boost/array.hpp>

namespace line_extraction
{

/*************************************************************
 * Data structures
*************************************************************/
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

/*************************************************************
 * Line
*************************************************************/
class Line
{

public:
  // Constructor / destructor
  Line(const CachedData&, const RangeData&, const Params&, std::vector<unsigned int>);
  ~Line();
  // Get methods for the line parameters
  double                          getAngle() const;
  const boost::array<double, 4>&  getCovariance() const;
  const boost::array<double, 2>&  getEnd() const;
  double                          getRadius() const;
  const boost::array<double, 2>&  getStart() const;
  // Methods for line fitting
  double distToPoint(unsigned int);
  void endpointFit();
  void leastSqFit();
  double length() const;
  unsigned int numPoints() const;

private:
  std::vector<unsigned int> indices_;
  // Data structures
  CachedData c_data_;
  RangeData r_data_;
  Params params_;
  PointParams p_params_;
  // Point variances used for least squares
  std::vector<double> point_scalar_vars_;
  std::vector<boost::array<double, 4> > point_covs_;
  double p_rr_;
  // Line parameters
  double angle_;
  double radius_;
  boost::array<double, 2> start_;
  boost::array<double, 2> end_;
  boost::array<double, 4> covariance_;
  // Methods
  void    angleFromEndpoints();
  void    angleFromLeastSq();
  double  angleIncrement();
  void    calcCovariance();
  void    calcPointCovariances();
  void    calcPointParameters();
  void    calcPointScalarCovariances();
  void    projectEndpoints();
  void    radiusFromEndpoints();
  void    radiusFromLeastSq();
}; // class Line


/*************************************************************
 * LineExtraction
*************************************************************/
class LineExtraction
{

public:
  // Constructor / destructor
  LineExtraction();
  ~LineExtraction();
  // Run
  void extractLines(std::vector<Line>&);
  // Data setting
  void setCachedData(const std::vector<double>&, const std::vector<double>&,
                     const std::vector<double>&, const std::vector<unsigned int>&);
  void setRangeData(const std::vector<double>&);
  // Parameter setting
  void setBearingVariance(double);
  void setRangeVariance(double);
  void setLeastSqAngleThresh(double);
  void setLeastSqRadiusThresh(double);
  void setMaxLineGap(double);
  void setMinLineLength(double);
  void setMinLinePoints(unsigned int);
  void setMinRange(double);
  void setMinSplitDist(double);
  void setOutlierDist(double);

private:
  // Data structures
  CachedData c_data_;
  RangeData r_data_;
  Params params_;
  // Indices after filtering
  std::vector<unsigned int> filtered_indices_;
  // Line data
  std::vector<Line> lines_;
  // Methods
  double distBetweenPoints(unsigned int index_1, unsigned int index_2);
  void filterClosePoints();
  void filterOutlierPoints();
  void filterLines();
  void mergeLines();
  void split(const std::vector<unsigned int>&);
}; // class LineExtraction

/*************************************************************
 * Utility functions
*************************************************************/
double pi_to_pi(double angle);

} // namespace line_extraction

#endif
