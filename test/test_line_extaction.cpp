// file contains tests for the LineExtraction class
#include <gtest/gtest.h>
#include <laser_line_extraction/line_extraction.h>
#include <sensor_msgs/LaserScan.h>
#include <Eigen/Dense>

#include <vector>
#include <cmath>
#include <numeric>
#include <array>

/// @brief a line defined by the two end-points
using line_t = std::array<Eigen::Vector2d, 2>;

/// @brief Returns the intersecion of _l1 and _l2. The intersection point must lie within the lines.
///
/// @param[in] _l1 the first line
/// @param[in] _l2 the second line
/// @param[out] _point the common intersection point
/// @return true if an intersection point can be defined.
static bool intersection(const line_t &_l1, const line_t &_l2, Eigen::Vector2d &_point)
{
  const Eigen::Vector2d start(_l2[0] - _l1[0]);
  Eigen::Matrix2d m;
  m.col(0) = _l1[1] - _l1[0];
  m.col(1) = _l2[0] - _l2[1];

  // check for collinear lines.
  if (m.determinant() == 0)
    return false;

  const Eigen::Vector2d parametric = m.inverse() * start;
  // check for line-bounds.
  if ((parametric.array() > 1).any() || (parametric.array() < 0).any())
    return false;

  _point = _l1[0] + m.col(0) * parametric.x();
  return true;
}

/// @brief Constructs a laser-scan from passed _lines.
///
/// The function assumes that the sensor origin is at (0, 0).
/// The sensor_msgs::LaserScan members will be set to:
///   -angle_min = -M_PI_2
///   -angle_max = M_PI_2
///   -angle_increment = 0.01
///   -range_min = 0
///   -range_max = 30
/// If more flexibility is needed, pass these values as arguments to the function.
static sensor_msgs::LaserScan
makeScan(const std::vector<line_t> &_lines)
{
  sensor_msgs::LaserScan msg;
  // setup the const-member for the message.
  msg.range_min = 0;
  msg.range_max = 30;
  msg.angle_min = -M_PI_2;
  msg.angle_max = M_PI_2;
  msg.angle_increment = 0.01;

  // get the size
  const auto size = static_cast<size_t>((msg.angle_max - msg.angle_min) / msg.angle_increment);
  msg.ranges.resize(size);

  const Eigen::Vector2d origin(0, 0);
  for (size_t ii = 0; ii != size; ++ii)
  {
    // compute the bearing
    const double bearing = msg.angle_min + ii * msg.angle_increment;

    // compute the end-point
    Eigen::Vector2d end(msg.range_max * std::cos(bearing), msg.range_max * std::sin(bearing));
    Eigen::Vector2d curr_end;
    const line_t curr_ray{origin, end};
    // find the closest point among all lines
    for (const auto &l : _lines)
    {
      if (!intersection(curr_ray, l, curr_end))
        continue;

      if (curr_end.norm() < end.norm())
        end = curr_end;
    }

    // write the range
    msg.ranges[ii] = end.norm();
  }
  return msg;
}

/// @brief fixture with the LineExtraction class
struct LineExtractionFixture : public testing::Test
{
  line_extraction::LineExtraction le_;      ///< the tested class
  std::vector<line_extraction::Line> lines; ///< the extracted lines

  /// @brief initializes the cache of the LineExtraction
  void initCache(const sensor_msgs::LaserScan &_msg)
  {
    // the size is for all vectors identical
    const auto size = _msg.ranges.size();
    std::vector<double> bearings(size), cos_bearings(size), sin_bearings(size);

    // compute the bearings
    for (size_t ii = 0; ii != size; ++ii)
    {
      bearings[ii] = _msg.angle_min + ii * _msg.angle_increment;
      cos_bearings[ii] = std::cos(bearings[ii]);
      sin_bearings[ii] = std::sin(bearings[ii]);
    }

    // indices are just a vector with [0 ... size)
    std::vector<unsigned int> indices(size);
    std::iota(indices.begin(), indices.end(), 0);

    // pass on the cached data
    le_.setCachedData(bearings, cos_bearings, sin_bearings, indices);
  }

  /// @brief initializes some default parameters
  void initParams()
  {
    // sensor-noise
    le_.setBearingVariance(1e-4);
    le_.setRangeVariance(1e-6);

    // sq-thresholds
    le_.setLeastSqAngleThresh(1e-4);
    le_.setLeastSqRadiusThresh(1e-4);

    // pre-filtering
    le_.setMinRange(0.1);
    le_.setMaxRange(20.);
    le_.setOutlierDist(0.05);

    // splitting
    le_.setMaxLineGap(0.5);
    le_.setMinSplitDist(0.05);

    // post-filtering
    le_.setMinLineLength(0.1);
    le_.setMinLinePoints(5);
  }

  /// @brief genetic init-method
  ///
  /// Method will generate a scan from _lines and setup the LineExtractor with the generated scan.
  void init(const std::vector<line_t> &_lines)
  {
    // create a scan
    const auto scan = makeScan(_lines);

    // init the LineExtractor
    initCache(scan);
    initParams();

    // pass on the range data
    const std::vector<double> ranges(scan.ranges.begin(), scan.ranges.end());
    le_.setRangeData(ranges);
  }
};

/// @brief fixture which creates a laser-scan from a simple line
struct SimpleLineFixture : public LineExtractionFixture
{
  void SetUp()
  {
    // create a laser-scan of a "wall" 0.2 m in front of the sensor
    const line_t l{Eigen::Vector2d{0.2, 10}, Eigen::Vector2d{0.2, -10}};
    init({l});
  }
};

TEST_F(SimpleLineFixture, basic)
{
  // test verifies the simplest case -> we should extract one line
  le_.extractLines(lines);

  ASSERT_EQ(lines.size(), 1);
}

TEST_F(SimpleLineFixture, filterMinRange)
{
  // test verifies that the min-range filtering works correct
  // set the min range above of the generated line.
  le_.setMinRange(20);
  le_.extractLines(lines);

  ASSERT_TRUE(lines.empty());
}

TEST_F(SimpleLineFixture, filterMaxRange)
{
  // test verifies that the max-range filtering works correct.
  // set the max range below of the generated line.
  le_.setMaxRange(0.1);
  le_.extractLines(lines);

  ASSERT_TRUE(lines.empty());
}

/// @brief verifies that min-split-distance works correctly
/// Fixture will create two lines which are "close" to each other.
/// We will then use the minDistance parameter to tweak if they should be splitted in two or not.
struct SplitDistanceFixture : public LineExtractionFixture
{
  double distance = 0.04; ///< distance between the lines
  void SetUp()
  {
    // create two lines
    const auto d1 = 1.;
    const auto d2 = d1 - distance;
    const line_t l1{Eigen::Vector2d{d1, 10}, Eigen::Vector2d{d1, -10}};
    const line_t l2{Eigen::Vector2d{d2, 0.5}, Eigen::Vector2d{d2, -0.5}};
    init({l1, l2});
  }
};

TEST_F(SplitDistanceFixture, noSpit)
{
  // with a bigger distance we don't split
  le_.setMinSplitDist(distance * 2);
  le_.extractLines(lines);
  ASSERT_EQ(lines.size(), 1);
}

TEST_F(SplitDistanceFixture, doSpit)
{
  // we want to split the lines
  le_.setMinSplitDist(distance / 2.);
  le_.extractLines(lines);
  ASSERT_EQ(lines.size(), 3);
}

/// @brief Fixture for testing the max-line-gap splitting.
struct LineGapFixture : public LineExtractionFixture
{
  double gap = 0.2;
  void SetUp()
  {
    // create one line with a gap.
    const auto d1 = .5;
    const line_t l1{Eigen::Vector2d{d1, 0.5}, Eigen::Vector2d{d1, gap / 2}};
    const line_t l2{Eigen::Vector2d{d1, -gap / 2}, Eigen::Vector2d{d1, -0.5}};

    init({l1, l2});
  }
};

TEST_F(LineGapFixture, noSpit)
{
  le_.setMaxLineGap(gap * 2);
  le_.extractLines(lines);
  ASSERT_EQ(lines.size(), 1);
}
// the counter test is not really trival...

/// @brief Fixture for testing the post-filtering steps (line-length and number of points).
struct PostFilterFixture : public LineExtractionFixture
{
  double length = 0.2;
  void SetUp()
  {
    // create a short line
    const auto d1 = .5;
    const line_t l1{Eigen::Vector2d{d1, 0}, Eigen::Vector2d{d1, length}};
    init({l1});
  }
};

TEST_F(PostFilterFixture, noFilterLineLength)
{
  le_.setMinLineLength(length - 0.1);
  le_.extractLines(lines);
  ASSERT_EQ(lines.size(), 1);
}

TEST_F(PostFilterFixture, doFilterLineLength)
{
  le_.setMinLineLength(length);
  le_.extractLines(lines);
  ASSERT_TRUE(lines.empty());
}

TEST_F(PostFilterFixture, doFilterLinePoints)
{
  // out line has just a little bit less then 40 points
  le_.setMinLinePoints(40);
  le_.extractLines(lines);
  ASSERT_TRUE(lines.empty());
}
