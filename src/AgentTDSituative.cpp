#include "AgentTDSituative.h"

#include <cmath>
#include <iostream>

#include <QDebug>

#include <omp.h>

using namespace Eigen;

AgentTDSituative::AgentTDSituative()
{
	r = new Reward(SCALING_MODE, m_world);
    Q_approach = MatrixXd::Random(SIZE_F_SPACE_APPROACH, m_world.getNA());
    Q_approach /= 10; // scale initial values in [+- 0.1]
    Q_shoot = MatrixXd::Random(SIZE_F_SPACE_SHOOT, m_world.getNA());
    Q_shoot /= 10; // scale initial values in [+- 0.1]
}
AgentTDSituative::~AgentTDSituative()
{
}

int AgentTDSituative::policy(int state) const
{
    return 0;
}

int AgentTDSituative::policy(int state, int situation) const
{
    Eigen::VectorXd q;
    if (situation == SITUATION_APPROACH)
        q = Q_approach.row(state);
    else
        q = Q_shoot.row(state);

	double maxValue = q(0);
	int maxi = 0;
	for (int i = 1; i < m_world.getNA(); i++)
		if (q(i) > maxValue)
		{
			maxi = i;
			maxValue = q(i);
		}
	if(maxValue == 0)
		return rand()%18;
	if (isTraining)
		return (rand()%100 > 100*eGreedy) ? maxi : rand()%m_world.getNA();
	return maxi;
}

int AgentTDSituative::policy(const Eigen::Ref<const Eigen::VectorXd> &obs) const
{
	HaxBall env;
    if (get_situation(obs) == SITUATION_APPROACH)
    {
        int stateIdx = obs_to_approach_feat_idx(env, obs);
        int a = policy(stateIdx, SITUATION_APPROACH);
        std::cout << "\r" << a << std::flush; //" \t" << stateF.transpose() << std::flush;
        return a;
    }
    else
    {
        int stateIdx = obs_to_shoot_feat_idx(env, obs);
        int a = policy(stateIdx, SITUATION_SHOOT);
        std::cout << "\r" << a << std::flush; // " \t" << stateF.transpose() << std::flush;
        return a;
    }

}

void AgentTDSituative::setHyperParam(double learningRate, double eligibilityHalflife, double discountFactor, double greedy)
{
	lr = learningRate;
	eligibility = std::pow(2, -eligibilityHalflife);
	discount = discountFactor;
	eGreedy = greedy;
}

double AgentTDSituative::training(int length, int trajectories, int threads)
{
    double cum_reward = 0.0;
	omp_set_num_threads(threads);

#pragma omp parallel for
	for (int i = 0; i < trajectories; ++i)
	{
		// std::cout << "\rTrajectory " << i+1 << " / " << trajectories << std::flush;
        cum_reward += training_worker(length);
	}
    return cum_reward;
}
double AgentTDSituative::training_worker(int length)
{
    Eigen::SparseMatrix<double> trace_approach(SIZE_F_SPACE_APPROACH, 18);
    Eigen::SparseMatrix<double> trace_shoot(SIZE_F_SPACE_SHOOT, 18);
	HaxBall env(false);
	Eigen::VectorXd obs = Eigen::VectorXd(env.getNO());
	int stateIdx, nextStateIdx;
	Eigen::VectorXd nextObs = Eigen::VectorXd(env.getNO());
	env.reset();
	env.getState(obs);

    double cum_reward = 0.0;
	// std::cout<<Q.row(0) <<std::endl;
	for (int step = 0; step < length; step++)
	{
		// std::cout << "\r - step " << step+1 << " / " << length << std::flush;
		// Choose action and observe the reward

        int action = -1;
        if (just_shooted)
        {
            nr_shoot_steps--;
            if (nr_shoot_steps == 0)
            {
                nr_shoot_steps = NUMBER_STEPS_IN_SHOOT;
                just_shooted = false;
            }
            //std::cout << "shoot" << std::endl;
            stateIdx = obs_to_shoot_feat_idx(env, obs);
            action = policy(stateIdx, SITUATION_SHOOT);
            if (rand()%100 > 100*eGreedy)
            {
                if (counter_random_action_shoot == REPEAT_RANDOM_ACTION)
                {
                    counter_random_action_shoot = 0;
                    last_random_action_shoot = rand()%m_world.getNA();
                }
                counter_random_action_shoot++;
                action = last_random_action_shoot;
            }
            env.step(action);
            env.getState(nextObs);
            nextStateIdx = obs_to_shoot_feat_idx(env, nextObs);
            double points = reward(obs, action, nextObs);

            cum_reward += points;

            // Update coefs
            trace_shoot = eligibility * trace_shoot;
            trace_shoot.coeffRef(stateIdx, action) = 1;
            points += discount * Q_shoot.row(nextStateIdx).maxCoeff() - Q_shoot(stateIdx, action);
            Q_shoot += lr * points * trace_shoot;
            //Q_shoot(stateIdx, action) += lr * points;

            // Next Round
            obs = nextObs;
            stateIdx = nextStateIdx;
            if(step%100 == 0)
                trace_shoot.prune(0.01);
        }
        else
        {
            if (get_situation(obs) == SITUATION_APPROACH)
            {
                //std::cout << "approach" << std::endl;
                stateIdx = obs_to_approach_feat_idx(env, obs);
                action = policy(stateIdx, SITUATION_APPROACH);
                if (rand()%100 > 100*eGreedy)
                {
                    if (counter_random_action_approach == REPEAT_RANDOM_ACTION)
                    {
                        counter_random_action_approach = 0;
                        last_random_action_approach = rand()%m_world.getNA();
                    }
                    counter_random_action_approach++;
                    action = last_random_action_approach;
                }
                env.step(action);
                env.getState(nextObs);
                nextStateIdx = obs_to_approach_feat_idx(env, nextObs);
                double points = reward(obs, action, nextObs);

                cum_reward += points;

                // Update coefs
                trace_approach = eligibility * trace_approach;
                trace_approach.coeffRef(stateIdx, action) = 1;
                points += discount * Q_approach.row(nextStateIdx).maxCoeff() - Q_approach(stateIdx, action);
                Q_approach += lr * points * trace_approach;
                //Q_approach(stateIdx, action) += lr * points;

                // Next Round
                obs = nextObs;
                stateIdx = nextStateIdx;
                if(step%100 == 0)
                    trace_approach.prune(0.01);
            }
            else
            {
                //std::cout << "shoot" << std::endl;
                stateIdx = obs_to_shoot_feat_idx(env, obs);
                action = policy(stateIdx, SITUATION_SHOOT);
                if (rand()%100 > 100*eGreedy)
                {
                    if (counter_random_action_shoot == REPEAT_RANDOM_ACTION)
                    {
                        counter_random_action_shoot = 0;
                        last_random_action_shoot = rand()%m_world.getNA();
                    }
                    counter_random_action_shoot++;
                    action = last_random_action_shoot;
                }
                env.step(action);
                env.getState(nextObs);
                nextStateIdx = obs_to_shoot_feat_idx(env, nextObs);
                double points = reward(obs, action, nextObs);

                cum_reward += points;

                // Update coefs
                trace_shoot = eligibility * trace_shoot;
                trace_shoot.coeffRef(stateIdx, action) = 1;
                points += discount * Q_shoot.row(nextStateIdx).maxCoeff() - Q_shoot(stateIdx, action);
                Q_shoot += lr * points * trace_shoot;
                //Q_shoot(stateIdx, action) += lr * points;

                // Next Round
                obs = nextObs;
                stateIdx = nextStateIdx;
                if(step%100 == 0)
                    trace_shoot.prune(0.01);

                just_shooted = true;
                nr_shoot_steps = NUMBER_STEPS_IN_SHOOT;
            }
        }

		//
		// if (abs(points) > 0.99 * REWARD_GOAL)
		// 	trace = Eigen::SparseMatrix<double>(SIZE_F_SPACE,18);
	}

    return cum_reward;

}

double AgentTDSituative::reward(const Eigen::Ref<const Eigen::VectorXd> &o, int a, const Eigen::Ref<const Eigen::VectorXd> &o_prime) const
{
    if (get_situation(o) == SITUATION_APPROACH)
        return r->calc_reward_approach(o, a, o_prime);
    else
        return r->calc_reward_shoot(o, a, o_prime);
}

void AgentTDSituative::save(std::string path)
{
    std::ofstream file;
    file.open(path.c_str(), std::ios::trunc);

	std::cout << "Saving hyperparameters" << std::endl;
    file << SIZE_F_SPACE_APPROACH << " " << SIZE_F_SPACE_SHOOT << " " << lr << " " << eligibility << " " << discount << " " << eGreedy <<  " " << trainingStage << "\n";

	Eigen::IOFormat save(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n");
    file << Q_approach.format(save) << "\n" << Q_shoot.format(save);
}
void AgentTDSituative::load(std::string path)
{
	std::ifstream file;
	file.open(path.c_str());
	if (!file.is_open())
	{
		std::cerr << "Could not open file. Initializing Q at random." << std::endl;
		return;
	}

    int states_approach, states_shoot;
	double learningRate, eligibilityFactor, discountFactor, greedy;
    file >> states_approach >> states_shoot;
    if ((states_approach != SIZE_F_SPACE_APPROACH) || (states_shoot != SIZE_F_SPACE_SHOOT))
	{
        std::cout << states_approach << " " << states_shoot << std::endl;
		std::cerr << "Mismatch between current and saved number of states. Initializing Q at random." << std::endl;
		return;
	}

	file >> learningRate >> eligibilityFactor >> discountFactor >> greedy >> trainingStage;
	std::cout << "Loading hyperparameters: \n\tlr = " << learningRate << "\n\teligibilityHalfLife = " << int(-std::log2(eligibilityFactor) / std::log2(2)) << "\n\tdiscountFactor = " << discountFactor << std::endl;
	setHyperParam(learningRate, eligibilityFactor, discountFactor, greedy);

	double d;
    for (int i = 0; i < states_approach; i++)
	{
		if (!((i + 1) % 1000))
            std::cout << "\rLoading Q: row " << i + 1 << "/" << states_approach << std::flush;
		std::vector<double> data;
		for (int j = 0; j < 18; j++)
		{
			file >> d;
			data.push_back(d);
        }
        Q_approach.row(i) = Eigen::VectorXd::Map(&data[0], data.size());

	}
    std::cout << "\rLoading Q_approach: row " << states_approach << "/" << states_approach << std::endl;

    for (int i = 0; i < states_shoot; i++)
    {
        if (!((i + 1) % 1000))
            std::cout << "\rLoading Q_shoot: row " << i + 1 << "/" << states_shoot << std::flush;
        std::vector<double> data;
        for (int j = 0; j < 18; j++)
        {
            file >> d;
            data.push_back(d);
        }
        Q_shoot.row(i) = Eigen::VectorXd::Map(&data[0], data.size());

        if (i<5)
        {
            std::cout << i << " " << Q_shoot.row(i) << std::endl;
        }
    }
    std::cout << "\rLoading Q_shoot: row " << states_shoot << "/" << states_shoot << std::flush;

	file.close();
	std::cout << "\n" << std::endl;
}

double AgentTDSituative::score(){
	long double sum = 0;
    for(int i = 0; i < SIZE_F_SPACE_APPROACH; i++)
        sum += Q_approach(i, policy(i, SITUATION_APPROACH))/SIZE_F_SPACE_APPROACH;
    for(int i = 0; i < SIZE_F_SPACE_SHOOT; i++)
        sum += Q_shoot(i, policy(i, SITUATION_SHOOT))/SIZE_F_SPACE_SHOOT;
	return sum;
}

int AgentTDSituative::get_situation(const Eigen::Ref<const Eigen::VectorXd> &obs) const
{
        Eigen::Vector2d player_pos = obs.segment(0, 2);
        Eigen::Vector2d ball_pos = obs.segment(2, 2);
        // Calculate euclidean distance from player to ball
        Eigen::Vector2d diff = ball_pos - player_pos;
        double distance = diff.norm();

        if ((distance < 0.15 + 0.25 + 0.3) && (player_pos[0] < ball_pos[0] - 0.15 - 0.25))
            return SITUATION_SHOOT;
        else
            return SITUATION_APPROACH;
}
