#include <ros/ros.h>
#include <Eigen/Core>
#include "dynamic_perception/panther_types.hpp"

namespace tp
{

struct cluster  // one observation
{
  Eigen::Vector3d centroidXYZ;
  Eigen::Vector3d boundingbox;  // Total side x, total side y, total side z;
  Eigen::Vector4d orientation;
  double time;           // in seconds
};

class track
{
public:
  bool is_new = true;
  track(const cluster& c, const int& min_ssw_tmp, const int& max_ssw_tmp)
  {
    max_ssw = max_ssw_tmp;
    min_ssw = min_ssw_tmp;

    is_new = true;

    history = std::deque<cluster>(min_ssw, c);  // Constant initialization

    // We have only one observation --> we assume the obstacle has always been there
    // std::cout << termcolor::magenta << "c.time= " << c.time << termcolor::reset << std::endl;
    for (unsigned int i = 0; i < min_ssw; i++)
    {
      history[i].time = c.time - (min_ssw - i - 1);
      //   c.time - (size - i - 1) * c.time / size;  // I need to have different times, if not A will become singular
      // std::cout << termcolor::magenta << "i= " << i << "history[i].time= " << history[i].time << termcolor::reset
      //           << std::endl;
    }

    num_diff_samples = 1;

    color = Eigen::Vector3d(((double)rand() / (RAND_MAX)),   ////// r
                            ((double)rand() / (RAND_MAX)),   ////// g
                            ((double)rand() / (RAND_MAX)));  ////// b

    // use its hex value as the id
    // https://www.codespeedy.com/convert-rgb-to-hex-color-code-in-cpp/
    int r = color.x() * 255;
    int g = color.y() * 255;
    int b = color.z() * 255;

    std::stringstream ss;
    ss << "#";
    ss << std::hex << (r << 16 | g << 8 | b);
    id_string = ss.str();

    id_int = stoi(std::to_string(r) + std::to_string(g) + std::to_string(b));  // concatenate r, g, b

    // TODO: The previous approach will **almost** always generate different ids, but not always
  }

  void addToHistory(const cluster& c)
  {
    history.push_back(c);

    if (num_diff_samples < min_ssw)
    {
      history.pop_front();  // Delete the oldest element
    }

    if (history.size() > max_ssw)
    {
      history.pop_front();  // Delete the oldest element
    }
    num_diff_samples = num_diff_samples + 1;
  }

  unsigned int getSizeSW()
  {
    return history.size();
  }

  Eigen::Vector3d getCentroidHistory(int i)
  {
    return history[i].centroidXYZ;
  }

  int getNumDiffSamples() const
  {
    return num_diff_samples;
  }

  bool shouldPublish()
  {
    return (num_diff_samples >= min_ssw);
  }

  double getTimeHistory(int i)
  {
    return history[i].time;
  }

  double getTotalTimeSW()  // Total time of the sliding window
  {
    return (history.back().time - history.front().time);
  }

  double getOldestTimeSW()
  {
    return (history.front().time);
  }

  double getRelativeTimeHistory(int i)
  {
    return (history[i].time - history.front().time);
  }

  double getLatestTimeSW()
  {
    return (history.back().time);
  }

  double getRelativeOldestTimeSW()
  {
    return 0.0;
  }

  double getRelativeLatestTimeSW()
  {
    return (history.back().time - history.front().time);
  }

  Eigen::Vector3d getLatestCentroid()
  {
    return history.back().centroidXYZ;
  }

  Eigen::Vector3d getLatestBbox()
  {
    return history.back().boundingbox;
  }

  Eigen::Vector3d getMaxBbox()
  {
    double max_x = -std::numeric_limits<double>::max();
    double max_y = -std::numeric_limits<double>::max();
    double max_z = -std::numeric_limits<double>::max();

    for (auto& c : history)
    {
      max_x = std::max(c.boundingbox.x(), max_x);
      max_y = std::max(c.boundingbox.y(), max_y);
      max_z = std::max(c.boundingbox.z(), max_z);
    }

    return Eigen::Vector3d(max_x, max_y, max_z);
  }

  void printPrediction(double seconds, int samples)
  {
    double last_time = getLatestTimeSW();
    double delta = seconds / samples;

    std::cout << "Predictions: " << std::endl;
    for (double t = last_time; t < (last_time + seconds); t = t + delta)
    {
      std::cout << "    t_to_the_future=" << t - last_time << " = " << pwp_mean.eval(t).transpose() << std::endl;
    }
  }

  void printHistory()
  {
    std::cout << "Track History= " << std::endl;

    // for (auto& c : history)
    // {
    //   c.print();
    // }
  }

  mt::PieceWisePol pwp_mean;
  mt::PieceWisePol pwp_var;

  unsigned int num_frames_skipped = 0;
  Eigen::Vector3d color;
  std::string id_string;
  int id_int;

private:
  unsigned int max_ssw;  // max size of the sliding window
  unsigned int min_ssw;  // min size of the sliding window
  unsigned int id;
  unsigned int num_diff_samples;

  // This deque will ALWAYS have ssw elements
  std::deque<cluster> history;  //[t-N], [t-N+1],...,[t] (i.e. index of the oldest element is 0)
};

}