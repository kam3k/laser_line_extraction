#ifndef LINE_EXTRACTION_H
#define LINE_EXTRACTION_H

#include <cmath>
#include <vector>
#include <boost/array.hpp>

namespace line_extraction
{

/*************************************************************
 * Line
*************************************************************/
class Line
{

public:
  // Constructor / destructor
  Line(std::vector<unsigned int>);
  ~Line();
  // Get methods for the line parameters
  double                          getAngle() const;
  const boost::array<double, 4>&  getCovariance() const;
  const boost::array<double, 2>&  getEnd() const;
  double                          getRadius() const;
  const boost::array<double, 2>&  getStart() const;
  // Methods for line fitting
  void endpointFit();
  void leastSqFit();
  double length() const;
  unsigned int numPoints() const;

private:
  // Point parameters used for least squares
  std::vector<double> a_;
  std::vector<double> aa_;
  std::vector<double> aaa_;
  std::vector<double> b_;
  std::vector<double> bb_;
  std::vector<double> bbb_;
  std::vector<double> c_;
  std::vector<double> s_;
  std::vector<double> point_scalar_vars_;
  std::vector<boost::array<double, 4> > point_covs_;
  // Points
  std::vector<unsigned int> line_indices_;
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
  friend class Line;
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
  void setLeastSqAngleThresh(double);
  void setLeastSqRadiusThresh(double);
  void setMaxLineGap(double);
  void setMinLineLength(double);
  void setMinLinePoints(unsigned int);
  void setMinRange(double);
  void setMinSplitDist(double);
  void setOutlierDist(double);
  void setRangeStdDev(double);
  void setRangeStdDevAsFraction(bool);

private:
  // Parameters
  double bearing_var_;
  double least_sq_angle_thresh_;
  double least_sq_radius_thresh_;
  double max_line_gap_;
  double min_line_length_;
  double min_range_;
  double min_split_dist_;
  double outlier_dist_;
  double range_std_dev_;
  bool range_std_dev_as_fraction_;
  unsigned int min_line_points_;
  // Scan data
  std::vector<unsigned int> indices_;
  std::vector<unsigned int> filtered_indices_;
  std::vector<double> bearings_;
  std::vector<double> cos_bearings_;
  std::vector<double> sin_bearings_;
  std::vector<double> ranges_;
  std::vector<double> range_vars_;
  std::vector<double> xs_;
  std::vector<double> ys_;
  // Line data
  std::vector<Line> lines_;
  // Methods
  double distBetweenPoints(unsigned int index_1, unsigned int index_2);
  void filterClosePoints();
  void filterOutlierPoints();
  void filterShortLines();
  void filterSparseLines();
  void mergeLines();
  void split(const std::vector<unsigned int>&);
}; // class LineExtraction

/*************************************************************
 * Utility functions
*************************************************************/
double pi_to_pi(double angle);

} // namespace line_extraction

#endif
