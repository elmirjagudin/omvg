#include <opencv2/core.hpp>
#include <openMVG/sfm/sfm.hpp>

void
init_synth_points(std::vector<cv::Mat> & points2d);

void
get_two_synth_views(openMVG::Mat & left, openMVG::Mat & right);