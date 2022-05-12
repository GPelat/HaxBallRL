#include <random>
#include <fstream>
#include <vector>

#include "BaseAgent.h"
#include "HaxBall.h"
#include "Features.h"
#include "Rewards.h"

#include "Eigen/Dense"
#include "Eigen/Sparse"

///
/// \brief The RandomAgent class
///
/// A dummy agent make the code compile, it executes a hardcoded action.
/// (Technically it is not a random agent)
///
/// This agent also contains some training functions, e.g. a proposal for structuring your code later on.
///
class AgentDP : public BaseAgent
{
public:
  AgentDP();
  ~AgentDP();

  int policy(int state) const;
  int policy(const Eigen::Ref<const Eigen::VectorXd>& obs) const;

  void save(std::string path);
  void load(std::string path);
  double score();
  ///
  /// \brief training Launcher for the training process
  /// \param length The length of trajectories in the HaxBall world
  /// \param trajectories How many trajectories you generate
  /// \param threads The number of threads to process all trajectories
  ///
  double training(int length, int trajectories, int threads = 1);

    bool isTraining;

    int count_nonzero_values();
    int count_nonzero_Q_values();

    void set_training_stage(int stage) {training_stage = stage;};

private:

  ///
  /// \brief training_worker The actual place where training happens
  /// \param length The length of a trajectory in the HaxBall world
  ///
  /// Put the training code in here, such that each thread has its private copy of storage
  /// Except for the Q-function you are perfectly thread safe
  ///
  double training_worker(int length);

  double V_star_calc(int i, HaxBall& env, int nextStatei = -1);
  double ValueIteration(HaxBall& env, double delta, double discount_factor, int length);

  ///
  /// \brief reward Your one step reward function
  /// \param o Current (continuous) state
  /// \param a Executed action
  /// \param o_prime Successor state (also continuous)
  /// \return the one step reward r(s,a,s')
  ///
  /// The reward function, use only these three inputs to avoid defining the reward with quantities the agent cannot observe.
  /// Of course your agent can store an extra instance of HaxBall to access static information like the goal area.
  ///
  double reward(const Eigen::Ref<const Eigen::VectorXd>& o,
                int a,
                const Eigen::Ref<const Eigen::VectorXd>& o_prime, bool was_in_left_goal=false, bool was_in_right_goal=false) const;


private:

  /// A passive and private game instance for getting details about the game (e.g. the goal position)
  /// Do not use this single instance for multithreaded training, as this would mess up the internal state
  HaxBall m_world;
  int training_stage = TRAINING_STAGE_BALL;
  Reward r = Reward(SCALING_MODE, m_world);

  Eigen::VectorXd V_star = Eigen::VectorXd(SIZE_F_SPACE);
  Eigen::VectorXi pi_star = Eigen::VectorXi(SIZE_F_SPACE);
  Eigen::VectorXd V = Eigen::VectorXd(SIZE_F_SPACE);
  Eigen::VectorXi pi = Eigen::VectorXi(SIZE_F_SPACE);

  int A_set[18] = { 0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17 };

  Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(SIZE_F_SPACE, m_world.getNA());

};
