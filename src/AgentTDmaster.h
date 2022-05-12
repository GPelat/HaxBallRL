#include <random>
#include <fstream>
#include <vector>

#include "BaseAgent.h"
#include "HaxBall.h"
#include "Features.h"
#include "Rewards.h"

#include "Eigen/Dense"
#include "Eigen/Sparse"

#define REPEAT_RANDOM_ACTION 10

///
/// \brief The RandomAgent class
///
/// A dummy agent make the code compile, it executes a hardcoded action.
/// (Technically it is not a random agent)
///
/// This agent also contains some training functions, e.g. a proposal for structuring your code later on.
///
class AgentTDmaster : public BaseAgent
{
public:
	AgentTDmaster();
	~AgentTDmaster();

	int policy(int state) const;
	int policy(const Eigen::Ref<const Eigen::VectorXd> &obs) const;

	int no_learning_policy(const Eigen::Ref<const Eigen::VectorXd> &obs) const;

	void setHyperParam(double learningRate, double eligibilityHalflife, double discountFactor, double greedy);

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

	bool isTraining = true;
	int trainingStage = 0;

private:
	///
	/// \brief training_worker The actual place where training happens
	/// \param length The length of a trajectory in the HaxBall world
	///
	/// Put the training code in here, such that each thread has its private copy of storage
	/// Except for the Q-function you are perfectly thread safe
	///
	double training_worker(int length);

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
	double reward(const Eigen::Ref<const Eigen::VectorXd> &o,
				  int a,
				  const Eigen::Ref<const Eigen::VectorXd> &o_prime) const;


private:
	/// A passive and private game instance for getting details about the game (e.g. the goal position)
	/// Do not use this single instance for multithreaded training, as this would mess up the internal state
	HaxBall m_world;

	Reward* r = nullptr;

	Eigen::MatrixXd Q;
	double lr = 0.001;
	double eligibility = 0.7;
	double discount = 0.8;
	double eGreedy = 0.3;
    int last_random_action = 0;
	bool last_action_random = false;
    int counter_random_action = REPEAT_RANDOM_ACTION;
};
