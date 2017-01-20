#include <grasp_candidates_generator/finger_hand.h>


FingerHand::FingerHand(double finger_width, double hand_outer_diameter,	double hand_depth)
  :	finger_width_(finger_width), hand_depth_(hand_depth),	lateral_axis_(-1),
   	forward_axis_(-1)
{
	int n = 10; // number of finger placements to consider over a single hand diameter

	Eigen::VectorXd fs_half;
	fs_half.setLinSpaced(n, 0.0, hand_outer_diameter - finger_width);
	finger_spacing_.resize(2 * fs_half.size());
	finger_spacing_	<< (fs_half.array() - hand_outer_diameter + finger_width_).matrix(), fs_half;
	fingers_ = Eigen::Array<bool, 1, Eigen::Dynamic>::Constant(1, 2 * n, false);

	hand_.resize(1, fingers_.size() / 2);
}


void FingerHand::evaluateFingers(const Eigen::Matrix3Xd& points, double bite, int idx)
{
  // calculate fingertip distance (top) and hand base distance (bottom)
  top_ = bite;
  bottom_ = bite - hand_depth_;

  fingers_.setConstant(false);

  // crop points at bite
  std::vector<int> cropped_indices;
  for (int i = 0; i < points.cols(); i++)
  {
    if (points(forward_axis_, i) < bite)
    {
      cropped_indices.push_back(i);

      // Check that the hand would be able to extend by <bite> onto the object without causing the back of the hand to
      // collide with <points>.
      if (points(forward_axis_, i) < bottom_)
      {
        return;
      }
    }
  }

  // check that there is at least one point in between the fingers
  if (cropped_indices.size() == 0)
    return;

  Eigen::Matrix3Xd cropped_points(3, cropped_indices.size());
  for (int i = 0; i < cropped_indices.size(); i++)
  {
    cropped_points.col(i) = points.col(cropped_indices[i]);
  }

  // identify free gaps
  int m = finger_spacing_.size();
  if (idx == -1)
  {
    for (int i = 0; i < m; i++)
    {
      int num_in_gap = (cropped_points.row(lateral_axis_).array() > finger_spacing_(i)
                        && cropped_points.row(lateral_axis_).array() < finger_spacing_(i) + finger_width_).count();
      if (num_in_gap == 0)
      {
        fingers_(i) = true;
      }
    }
  }
  else
  {
    for (int i = 0; i < m; i++)
    {
      if (i == idx || i == m/2 + idx)
      {
        int num_in_gap = (cropped_points.row(lateral_axis_).array() > finger_spacing_(i)
                          && cropped_points.row(lateral_axis_).array() < finger_spacing_(i) + finger_width_).count();
        if (num_in_gap == 0)
        {
          fingers_(i) = true;
        }
      }
    }
  }
}


void FingerHand::deepenHand(const Eigen::Matrix3Xd& points, double min_depth, double max_depth)
{
  std::vector<int> hand_idx;

  for (int i = 0; i < hand_.cols(); i++)
  {
    if (hand_(i) == true)
      hand_idx.push_back(i);
  }

  if (hand_idx.size() == 0)
    return;

  // choose middle hand
  int hand_eroded_idx = hand_idx[ceil(hand_idx.size() / 2.0) - 1]; // middle index

  // attempt to deepen hand
  double deepen_step_size = 0.005;
  FingerHand new_hand = *this;
  FingerHand last_new_hand = new_hand;

  for (double depth = min_depth + deepen_step_size; depth <= max_depth; depth += deepen_step_size)
  {
    new_hand.evaluateFingers(points, depth, hand_eroded_idx);
    if (new_hand.getFingers().cast<int>().sum() < 2)
      break;

    new_hand.evaluateHand();
    last_new_hand = new_hand;
  }

  *this = last_new_hand; // recover the deepest hand
  hand_.setConstant(false);
  hand_(hand_eroded_idx) = true;
}


std::vector<int> FingerHand::computePointsInClosingRegion(const Eigen::Matrix3Xd& points)
{
  // find feasible hand location
  int idx = -1;
  for (int i = 0; i < hand_.cols(); i++)
  {
    if (hand_(i) == true)
    {
      idx = i;
      break;
    }
  }
  if (idx == -1)
  {
    std::cout << "ERROR: Something went wrong!\n";
  }

  // calculate the lateral parameters of the closing region of the robot hand for this finger placement
  left_ = finger_spacing_(idx) + finger_width_;
  right_ = finger_spacing_(hand_.cols() + idx);
  center_ = 0.5 * (left_ + right_);
  surface_ = points.row(lateral_axis_).minCoeff();

  // find points inside closing region defined by <bottom_>, <top_>, <left_> and <right_>
  std::vector<int> indices;
  for (int i = 0; i < points.cols(); i++)
  {
    if (points(forward_axis_, i) > bottom_ && points(forward_axis_, i) < top_
        && points(lateral_axis_, i) > left_ && points(lateral_axis_, i) < right_)
    {
      indices.push_back(i);
    }
  }

  return indices;
}


void FingerHand::evaluateHand()
{
	int n = fingers_.size() / 2;

	for (int i = 0; i < n; i++)
	{
	  hand_(i) = (fingers_(i) == true && fingers_(n + i) == true);
	}
}


void FingerHand::evaluateHand(int idx)
{
  int n = fingers_.size() / 2;
  hand_.setConstant(false);
  hand_(idx) = (fingers_(idx) == true && fingers_(n + idx) == true);
}
