#include "AgentTDmaster.h"

#include <cmath>
#include <iostream>

#include <QDebug>

#include <omp.h>

using namespace Eigen;

AgentTDmaster::AgentTDmaster()
{
	r = new Reward(SCALING_MODE, m_world);
	Q = MatrixXd::Zero(SIZE_F_DB, m_world.getNA());
	Q /= 10.0;
}
AgentTDmaster::~AgentTDmaster()
{
}

int AgentTDmaster::policy(int state) const
{
	Eigen::VectorXd q = Q.row(state);
	double maxValue = q(0);
	int maxi = 0;
	for (int i = 1; i < m_world.getNA(); i++)
		if (q(i) > maxValue)
		{
			maxi = i;
			maxValue = q(i);
		}
	return maxi;
}

int AgentTDmaster::policy(const Eigen::Ref<const Eigen::VectorXd> &obs) const
{
	// return no_learning_policy(obs);
	HaxBall env;
	int stateIdx = obs_to_dumbbot(env, obs);
	int a = policy(stateIdx);
	std::cout << "\r" << a << " \t" << no_learning_policy(obs) << " \t " << stateIdx << std::flush;
	return a;
}

int AgentTDmaster::no_learning_policy(const Eigen::Ref<const Eigen::VectorXd> &obs) const
{
	Eigen::Vector2d player_pos = obs.segment(0, 2);
	Eigen::Vector2d ball_pos = obs.segment(2, 2);
	Eigen::Vector2d ball_vel = obs.segment(4, 2);
	Eigen::Vector2d future_ball_pos = ball_pos + 2 * m_world.getTimeDelta() * ball_vel;
	Eigen::Vector2d right_goal_pos(m_world.getGoalRight().center().rx(), m_world.getGoalRight().center().ry());
	double d_target = 0.1 + m_world.getRadiusBall() + m_world.getRadiusPlayer();
	Eigen::Vector2d target = d_target * (future_ball_pos - right_goal_pos).normalized() + future_ball_pos;

	// ensure in field
	if (target(0) < -4)
		target(0) = -4 - (target(0) + 4);
	if (target(0) > 4)
		target(0) = 4 - (target(0) - 4);
	if (target(1) < -2)
		target(1) = -2 - (target(1) + 2);
	if (target(1) > 2)
		target(1) = 2 - (target(1) - 2);

	Eigen::Vector2d p2t = target - player_pos;

	if (p2t(0) < 0 && target(0) < right_goal_pos(0))
	{
		if (abs(player_pos[1] - future_ball_pos[1]) < d_target)
			return (target(1) < 0) ? 8 : 4; // South-West, North-West
		return 6;							// West
	}

	int action = 0;

	if (p2t(0) < -0.02)
		action = 6;
	else if (p2t(0) > 0.02)
		action = 14;

	if (p2t(1) < -0.02)
		switch (action)
		{
		case 0:
			action = 10;
			break;
		case 6:
			action = 8;
			break;
		case 14:
			action = 12;
		}
	else if (p2t(1) > 0.02)
		switch (action)
		{
		case 0:
			action = 2;
			break;
		case 6:
			action = 4;
			break;
		case 14:
			action = 16;
		}

	if (p2t.squaredNorm() < 0.005)
		action++; // Shoot

	return action;
}

void AgentTDmaster::setHyperParam(double learningRate, double eligibilityHalflife, double discountFactor, double greedy)
{
	lr = learningRate;
	eligibility = std::pow(2, -eligibilityHalflife);
	discount = discountFactor;
	eGreedy = greedy;
}

double AgentTDmaster::training(int length, int trajectories, int threads)
{
	omp_set_num_threads(threads);
	double cum_reward = 0.0;

#pragma omp parallel for
	for (int i = 0; i < trajectories; ++i)
	{
		// std::cout << "\rTrajectory " << i+1 << " / " << trajectories << std::flush;
		cum_reward += training_worker(length);
	}

	std::cout << "Zero coefficients: " << (Q.array() == 0.0).count() << std::endl;
	return cum_reward;
}
double AgentTDmaster::training_worker(int length)
{
	bool contact = false;
	double cum_reward = 0.0;
	Eigen::SparseMatrix<double> trace(SIZE_F_DB, 18);
	// Eigen::SparseMatrix<double> trace_last(SIZE_F_DB, 18);
	HaxBall env(false);
	Eigen::VectorXd obs = Eigen::VectorXd(env.getNO());
	int stateIdx, nextStateIdx;
	Eigen::VectorXd nextObs = Eigen::VectorXd(env.getNO());
	env.reset();
	env.getState(obs);
	stateIdx = obs_to_dumbbot(env, obs);
	// std::cout<<Q.row(0) <<std::endl;
	for (int step = 0; step < length; step++)
	{
		Eigen::Vector2d player_pos = obs.segment(0, 2);
		Eigen::Vector2d ball_pos = obs.segment(2, 2);
		if ((player_pos - ball_pos).norm() < 0.5)
			contact = true;
		// std::cout << "\r - step " << step+1 << " / " << length << std::flush;
		// Choose action and observe the reward
		int action = no_learning_policy(obs);
		if (trainingStage > 35 && rand() % 100 < trainingStage)
			action = policy(stateIdx);

		if (counter_random_action)
		{
			counter_random_action++;
			action = last_random_action;
			if (counter_random_action >= REPEAT_RANDOM_ACTION)
				counter_random_action = 0;
		}
		else if (trainingStage > 35 && rand() % 100 < 100 * eGreedy)
		{
			counter_random_action = 1;
			last_random_action = rand() % 18;
			action = last_random_action;
		}

		env.step(action);
		env.getState(nextObs);
		nextStateIdx = obs_to_dumbbot(env, obs);
		// if (trainingStage < 30)
		// {
		// 	int a = no_learning_policy(obs);
		// 	points = (action == a) ? 1 : -1;
		// }

		// Update coefs
		// Q(stateIdx, action) = Q(stateIdx, action) + lr * (points + discount * (Q.row(nextStateIdx).maxCoeff() - Q(stateIdx, action)));
		// if (trainingStage < 30)
		// 	trace = 0.2 * trace;
		// else
		trace = eligibility * trace;
		trace.coeffRef(stateIdx, action) = 1;
		if (contact)
		{
			double points = r->calc_reward_db(obs, action, nextObs);
			cum_reward += points;
			points += discount * Q.row(nextStateIdx).maxCoeff() - Q(stateIdx, action);
			Q += lr * points * trace;
		}
		// if ((0.9995 * current_ball_speed - next_ball_speed).squaredNorm() > 0.1)
		// 	trace_last = trace;

		// Q(stateIdx, action) += lr * points;

		//
		// if (abs(points) > 0.99 * REWARD_GOAL)
		// 	trace = Eigen::SparseMatrix<double>(SIZE_F_DB,18);

		// Next Round
		obs = nextObs;
		stateIdx = nextStateIdx;

		if (step % 100 == 0)
			trace.prune(0.001);
	}
	return cum_reward;
}

double AgentTDmaster::reward(const Eigen::Ref<const Eigen::VectorXd> &o, int a, const Eigen::Ref<const Eigen::VectorXd> &o_prime) const
{
	double score = 0;
	// if (trainingStage > 1)
	// 	score = r->calc_reward(o, a, o_prime);
	// else
	// 	score = (a == no_learning_policy(o)) ? 1 : -1;

	score = r->calc_reward_db(o, a, o_prime);

	return score;
}

void AgentTDmaster::save(std::string path)
{
	std::ofstream file;
	file.open(path.c_str(), std::ios::trunc);

	std::cout << "Saving hyperparameters" << std::endl;
	file << SIZE_F_DB << " " << lr << " " << eligibility << " " << discount << " " << eGreedy << " " << trainingStage << "\n";

	Eigen::IOFormat save(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n");
	file << Q.format(save);
}
void AgentTDmaster::load(std::string path)
{
	std::ifstream file;
	file.open(path.c_str());
	if (!file.is_open())
	{
		std::cerr << "Could not open file. Initializing Q at random." << std::endl;
		return;
	}

	int states;
	double learningRate, eligibilityFactor, discountFactor, greedy;
	file >> states;
	if (states != SIZE_F_DB)
	{
		std::cerr << "Mismatch between current and saved number of states. Initializing Q at random." << std::endl;
		return;
	}

	file >> learningRate >> eligibilityFactor >> discountFactor >> greedy >> trainingStage;
	std::cout << "Loading hyperparameters: \n\tlr = " << learningRate << "\n\teligibilityHalfLife = " << int(-std::log2(eligibilityFactor) / std::log2(2)) << "\n\tdiscountFactor = " << discountFactor << std::endl;
	setHyperParam(learningRate, eligibilityFactor, discountFactor, greedy);

	double d;
	for (int i = 0; i < states; i++)
	{
		if (!((i + 1) % 1000))
			std::cout << "\rLoading Q: row " << i + 1 << "/" << states << std::flush;
		std::vector<double> data;
		for (int j = 0; j < 18; j++)
		{
			file >> d;
			data.push_back(d);
		}
		Q.row(i) = Eigen::VectorXd::Map(&data[0], data.size());
	}
	std::cout << "\rLoading Q: row " << states << "/" << states << std::flush;

	file.close();
	std::cout << "\n"
			  << std::endl;
}

double AgentTDmaster::score()
{
	long double sum = 0;
	for (int i = 0; i < SIZE_F_DB; i++)
		sum += Q(i, policy(i)) / SIZE_F_DB;
	return sum;
}