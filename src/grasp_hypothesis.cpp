#include <grasp_candidates_generator/grasp_hypothesis.h>


GraspHypothesis::GraspHypothesis(const Eigen::Vector3d& sample, const Eigen::Matrix3d& frame,
  const FingerHand& finger_hand, const PointList& point_list_learning)
: sample_(sample)
{
  pose_.frame_ = frame;

  construct(finger_hand);

  // calculate grasp width (hand opening width)
  grasp_width_ = point_list_learning.getPoints().row(0).maxCoeff() - point_list_learning.getPoints().row(0).minCoeff();

  learning_data_.points_ = point_list_learning.getPoints();
  learning_data_.normals_ = point_list_learning.getNormals();
  learning_data_.camera_source_ = point_list_learning.getCamSource();
}


GraspHypothesis::GraspHypothesis(const Eigen::Vector3d& sample, const Eigen::Matrix3d& frame,
  const FingerHand& finger_hand)
: sample_(sample), grasp_width_(0.0)
{
  pose_.frame_ = frame;

  construct(finger_hand);
}


void GraspHypothesis::construct(const FingerHand& finger_hand)
{
  // finger positions and base/bottom and top/fingertip of grasp with respect to hand frame
  config_1d_.left_ = finger_hand.getLeft();
  config_1d_.right_ = finger_hand.getRight();
  config_1d_.top_ = finger_hand.getTop();
  config_1d_.bottom_ = finger_hand.getBottom();
  config_1d_.center_ = finger_hand.getCenter();

  // calculate grasp positions at the bottom/base and top of the hand and on the object surface
  calculateGraspPositions(finger_hand);

  // determine finger placement index that resulted in this grasp
  const Eigen::Array<bool, 1, Eigen::Dynamic>& indices = finger_hand.getHand();
  for (int i = 0; i < indices.size(); i++)
  {
    if (indices[i] == true)
    {
      learning_data_.finger_placement_index_ = i;
      break;
    }
  }
}


void GraspHypothesis::calculateGraspPositions(const FingerHand& finger_hand)
{
  // calculate grasp positions of hand middle on object surface, bottom/base and top/fingertip w.r.t. base frame
  Eigen::Vector3d pos_top, pos_bottom, pos_surface;
  pos_surface << finger_hand.getSurface(), finger_hand.getCenter(), 0.0;
  pos_bottom << getBottom(), finger_hand.getCenter(), 0.0;
  pos_top << getTop(), finger_hand.getCenter(), 0.0;
  pose_.surface_ = getFrame() * pos_surface + sample_;
  pose_.bottom_ = getFrame() * pos_bottom + sample_;
  pose_.top_ = getFrame() * pos_top + sample_;
}


void GraspHypothesis::writeHandsToFile(const std::string& filename, const std::vector<GraspHypothesis>& hands) const
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


void GraspHypothesis::print() const
{
  std::cout << "approach: " << getApproach().transpose() << std::endl;
  std::cout << "binormal: " << getBinormal().transpose() << std::endl;
  std::cout << "axis: " << getAxis().transpose() << std::endl;
  std::cout << "grasp width: " << getGraspWidth() << std::endl;
  std::cout << "grasp surface: " << getGraspSurface().transpose() << std::endl;
  std::cout << "grasp bottom: " << getGraspBottom().transpose() << std::endl;
  std::cout << "grasp top: " << getGraspTop().transpose() << std::endl;
  std::cout << "score: " << getScore() << std::endl;
  std::cout << "half-antipodal: " << isHalfAntipodal() << std::endl;
  std::cout << "full-antipodal: " << isFullAntipodal() << std::endl;
  std::cout << "finger_hand:\n";
  std::cout << "  bottom: " << getBottom() << std::endl;
  std::cout << "  top: " << getTop() << std::endl;
}


std::string GraspHypothesis::vectorToString(const Eigen::VectorXd& v) const
{
  std::string s = "";
  for (int i = 0; i < v.rows(); i++)
  {
    s += boost::lexical_cast<std::string>(v(i)) + ",";
  }
  return s;
}
