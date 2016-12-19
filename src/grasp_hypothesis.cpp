#include <grasp_candidates_generator/grasp_hypothesis.h>


GraspHypothesis::GraspHypothesis(const Eigen::Vector3d& sample, const Eigen::Matrix3d& frame,
  const FingerHand& finger_hand, const PointList& point_list_learning)
: sample_(sample), hand_frame_(frame), half_antipodal_(false), full_antipodal_(false),
  score_(0.0)
{
  construct(frame, finger_hand);

  // calculate grasp width (hand opening width)
  grasp_width_ = point_list_learning.getPoints().row(0).maxCoeff() - point_list_learning.getPoints().row(0).minCoeff();

  points_for_learning_ = point_list_learning.getPoints();
  normals_for_learning_ = point_list_learning.getNormals();
  camera_source_for_learning_ = point_list_learning.getCamSource();
}


GraspHypothesis::GraspHypothesis(const Eigen::Vector3d& sample, const Eigen::Matrix3d& frame,
  const FingerHand& finger_hand)
: sample_(sample), hand_frame_(frame), grasp_width_(0.0), half_antipodal_(false), full_antipodal_(false),
  score_(0.0)
{
  construct(frame, finger_hand);
}


void GraspHypothesis::construct(const Eigen::Matrix3d& frame, const FingerHand& finger_hand)
{
  approach_ = frame.col(0);
  binormal_ = frame.col(1);
  axis_ = frame.col(2);

  // finger positions and base/bottom and top/fingertip of grasp with respect to hand frame
  left_ = finger_hand.getLeft();
  right_ = finger_hand.getRight();
  top_ = finger_hand.getTop();
  bottom_ = finger_hand.getBottom();
  center_ = finger_hand.getCenter();

  // calculate grasp positions at the bottom/base and top of the hand and on the object surface
  calculateGraspPositions(finger_hand);

  // determine finger placement index that resulted in this grasp
  const Eigen::Array<bool, 1, Eigen::Dynamic>& indices = finger_hand.getHand();
  for (int i = 0; i < indices.size(); i++)
  {
    if (indices[i] == true)
    {
      finger_placement_index_ = i;
      break;
    }
  }
}


void GraspHypothesis::calculateGraspPositions(const FingerHand& finger_hand)
{
  // calculate grasp positions of hand middle on object surface, bottom/base and top/fingertip w.r.t. base frame
  Eigen::Vector3d pos_top, pos_bottom, pos_surface;
  pos_surface << finger_hand.getSurface(), finger_hand.getCenter(), 0.0;
  pos_bottom << bottom_, finger_hand.getCenter(), 0.0;
  pos_top << top_, finger_hand.getCenter(), 0.0;
  grasp_surface_ = hand_frame_ * pos_surface + sample_;
  grasp_bottom_ = hand_frame_ * pos_bottom + sample_;
  grasp_top_ = hand_frame_ * pos_top + sample_;
}


void GraspHypothesis::writeHandsToFile(const std::string& filename, const std::vector<GraspHypothesis>& hands)
{
  std::ofstream myfile;
  myfile.open (filename.c_str());

  for (int i = 0; i < hands.size(); i++)
  {
    std::cout << "Hand " << i << std::endl;
    print();

    myfile << vectorToString(hands[i].getGraspBottom()) << vectorToString(hands[i].getGraspSurface())
          << vectorToString(hands[i].getAxis()) << vectorToString(hands[i].getApproach())
          << vectorToString(hands[i].getBinormal()) << boost::lexical_cast<double>(hands[i].getGraspWidth()) << "\n";
  }

  myfile.close();
}


void GraspHypothesis::print()
{
  std::cout << "axis: " << axis_.transpose() << std::endl;
  std::cout << "approach: " << approach_.transpose() << std::endl;
  std::cout << "binormal: " << binormal_.transpose() << std::endl;
  std::cout << "grasp width: " << grasp_width_ << std::endl;
  std::cout << "grasp surface: " << grasp_surface_.transpose() << std::endl;
  std::cout << "grasp bottom: " << grasp_bottom_.transpose() << std::endl;
  std::cout << "grasp top: " << grasp_top_.transpose() << std::endl;
  std::cout << "score: " << score_ << std::endl;
  std::cout << "half-antipodal: " << half_antipodal_ << std::endl;
  std::cout << "full-antipodal: " << full_antipodal_ << std::endl;
  std::cout << "finger_hand:\n";
  std::cout << "  bottom: " << bottom_ << std::endl;
  std::cout << "  center: " << center_ << std::endl;
  std::cout << "  top: " << top_ << std::endl;
}


std::string GraspHypothesis::vectorToString(const Eigen::VectorXd& v)
{
  std::string s = "";
  for (int i = 0; i < v.rows(); i++)
  {
    s += boost::lexical_cast<std::string>(v(i)) + ",";
  }
  return s;
}
