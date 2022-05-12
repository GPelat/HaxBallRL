#include "HaxBall.h"
#include "Eigen/Dense"

#pragma once

///
/// This file implements a reward class which can be used by the agents. During object instantiation, the mode has to be chosen. Afterwards, the rewards are calculated depending on the chosen mode.
///

// Modes
#define SCALING_MODE 0
#define SEQUENTIAL_MODE 1

// Training stages for sequential mode
#define TRAINING_STAGE_MOVE 0
#define TRAINING_STAGE_BALL 1
#define TRAINING_STAGE_SHOOT 2
#define TRAINING_STAGE_GOAL 3

// Reward values
#define REWARD_MOVE 0
#define REWARD_BALL 0.1
#define REWARD_SHOOT 50
#define REWARD_GOAL 100

class Reward
{
public:
    Reward(int mode, HaxBall &haxball);
    ~Reward();

    double calc_reward(const Eigen::Ref<const Eigen::VectorXd>& o, int a, const Eigen::Ref<const Eigen::VectorXd>& o_prime, bool was_in_left_goal = false, bool was_in_right_goal = false) const;
    double calc_reward_new(const Eigen::Ref<const Eigen::VectorXd>& o, int a, const Eigen::Ref<const Eigen::VectorXd>& o_prime, bool was_in_left_goal = false, bool was_in_right_goal = false) const;
    double calc_reward_approach(const Eigen::Ref<const Eigen::VectorXd>& o, int a, const Eigen::Ref<const Eigen::VectorXd>& o_prime) const;
    double calc_reward_shoot(const Eigen::Ref<const Eigen::VectorXd>& o, int a, const Eigen::Ref<const Eigen::VectorXd>& o_prime) const;
    double calc_reward(int training_stage, const Eigen::Ref<const Eigen::VectorXd>& o, int a, const Eigen::Ref<const Eigen::VectorXd>& o_prime) const;
    double calc_reward_db(const Eigen::Ref<const Eigen::VectorXd> &o, int a, const Eigen::Ref<const Eigen::VectorXd> &o_prime) const;

private:
    int mode;
    HaxBall &haxball;
};
