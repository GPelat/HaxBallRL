#include "HaxBall.h"
#include "Eigen/Dense"
#include <iostream>

#define NO_FEATURES 6//7//9
#define SIZE_F_SPACE 125000//8000000//37422//374220 // 2 * 8 * 8 * 3 * 3 * 3 * 3 * 11 * 21
#define SIZE_F_DB 2880
#define SIZE_F_SPACE_APPROACH 2079
#define SIZE_F_SPACE_SHOOT 37422

int obs_to_features(const HaxBall &haxball, const Eigen::Ref<const Eigen::VectorXd> obs);
int obs_to_feat_idx(const HaxBall &haxball, const Eigen::Ref<const Eigen::VectorXd> obs);
int feat2idx(const Eigen::Ref<const Eigen::VectorXi> &feat);
int obs_to_dumbbot(const HaxBall &haxball, const Eigen::Ref<const Eigen::VectorXd> obs);
int obs_to_approach_feat_idx(const HaxBall &haxball, const Eigen::Ref<const Eigen::VectorXd> obs);
int obs_to_shoot_feat_idx(const HaxBall &haxball, const Eigen::Ref<const Eigen::VectorXd> obs);
