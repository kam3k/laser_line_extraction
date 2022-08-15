#ifndef LINE_EXTRACTION_LINE_H
#define LINE_EXTRACTION_LINE_H

#include <vector>
#include <boost/array.hpp>
#include "laser_line_extraction/utilities.h"

namespace line_extraction
{

  class Line
  {

  public:
    // Constructor / destructor
    Line(const CachedData &, const RangeData &, const Params &, std::vector<unsigned int>);
    Line(double angle, double radius, const std::array<double, 4> &covariance,
         const std::array<double, 2> &start, const std::array<double, 2> &end,
         const std::vector<unsigned int> &indices);
    ~Line();
    // Get methods for the line parameters
    double getAngle() const;
    const std::array<double, 4> &getCovariance() const;
    const std::array<double, 2> &getEnd() const;
    const std::vector<unsigned int> &getIndices() const;
    double getRadius() const;
    const std::array<double, 2> &getStart() const;
    // Methods for line fitting
    double distToPoint(unsigned int);
    void endpointFit();
    void leastSqFit();
    double length() const;
    unsigned int numPoints() const;
    void projectEndpoints();

  private:
    std::vector<unsigned int> indices_;
    // Data structures
    CachedData c_data_;
    RangeData r_data_;
    Params params_;
    PointParams p_params_;
    // Point variances used for least squares
    std::vector<double> point_scalar_vars_;
    std::vector<std::array<double, 4>> point_covs_;
    double p_rr_;
    // Line parameters
    double angle_;
    double radius_;
    std::array<double, 2> start_;
    std::array<double, 2> end_;
    std::array<double, 4> covariance_;
    // Methods
    void angleFromEndpoints();
    void angleFromLeastSq();
    double angleIncrement();
    void calcCovariance();
    void calcPointCovariances();
    void calcPointParameters();
    void calcPointScalarCovariances();
    void radiusFromEndpoints();
    void radiusFromLeastSq();
  }; // class Line

} // namespace line_extraction

#endif
