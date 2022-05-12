#include "AgentDP.h"

#include <cmath>
#include <iostream>
#include <algorithm>

#include <QDebug>

#include <omp.h>

using namespace std;


AgentDP::AgentDP()
{

}
AgentDP::~AgentDP()
{

}

int AgentDP::policy(int state) const
{
  return pi_star[state];

  /*Eigen::VectorXd q = Q.row(state);
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
  return maxi;*/
}

int AgentDP::policy(const Eigen::Ref<const Eigen::VectorXd>& obs) const
{
  int stateIdx = obs_to_feat_idx(m_world, obs);
  return policy(stateIdx);
}

double AgentDP::training(int length, int trajectories, int threads)
{
  double cum_reward = 0.0;
  omp_set_num_threads(threads);

#pragma omp parallel for
  for (int i = 0; i < trajectories; ++i)
  {
    cum_reward += training_worker(length);
  }
  return cum_reward;
}
double AgentDP::training_worker(int length)
{
  // Training code goes here, be aware that multiple threads are active in here
    HaxBall env;
    env.reset();
    return V_star_calc(length, env);
    //ValueIteration(env, 1e-4, 0.5, length);
    // cout << V_star << endl;
    //return{ max_expectation, maximizing_action };

}

double AgentDP::V_star_calc(int i, HaxBall& env, int nextStatei) {
    double cum_reward = 0.0;
    if (nextStatei != -1)
    {
        if (V_star[nextStatei] != 0)
            return V_star[nextStatei];
    }
    if (i == 0) {
        return 0.0;//V_star[nextStatei];
    }
	else {
		i--;
		Eigen::VectorXd tempObs = Eigen::VectorXd(env.getNO());
        env.getState(tempObs);
        int stateIdx = obs_to_feat_idx(env, tempObs);
        //cout << stateIdx << endl;
        int maximizing_action = 0;
        double max_expectation = -numeric_limits<double>::infinity();
        double tmp_reward  = 0.0;
        for (int a = 0; a < env.getNA(); a++) {
            env.setState(tempObs);
            //for (int x = 0; x < 100; x++)
            env.step(A_set[a]);
            Eigen::VectorXd nextObs = Eigen::VectorXd(env.getNO());
			env.getState(nextObs);
            int temp = obs_to_features(env, nextObs);
            //if ((featnext(1) != featnext(3)) || (featnext(2)!=featnext(4)))
            //    std::cout << "curr and future ball pos differ" << std::endl;
            // int nextStateIdx = obs_to_feat_idx(env, nextObs);
            // cout << nextStateIdx << endl;
            //double expectation = 0.0;

            double expectation = reward(tempObs, A_set[a], nextObs, env.ballWasInLeftGoal(), env.ballWasInRightGoal()) + V_star_calc(i, env, obs_to_feat_idx(env, nextObs));
            if (expectation > max_expectation) {
                max_expectation = expectation;
                maximizing_action = a;
                tmp_reward = expectation;
            }
        }
        cum_reward = tmp_reward;
        //cout << "new value: " << stateIdx << endl;
		V_star[stateIdx] = max_expectation;
        pi_star[stateIdx] = A_set[maximizing_action];

        if (nextStatei != -1)
        {
            return V_star[nextStatei];
        }
        else
        {
            return cum_reward;
        }
    }
}

double AgentDP::ValueIteration(HaxBall& env, double delta, double discount_factor, int length)
{
    double theta = 0.0;
    double theta2 = numeric_limits<double>::infinity();
    int count = 0;
    do
    {
        count += 1;
        if (count % 10000000 == 0)
        {
            cout << "Theta 1: " << theta << endl;
            //cout << "Theta 2: " << theta2 << endl;
            cout << "Nonero V values: " << count_nonzero_values() << endl;
            theta=0.0;
            count = 0;
        }
        env.reset();
        Eigen::VectorXd tempObs = Eigen::VectorXd(env.getNO());
        env.getState(tempObs);
        int s = obs_to_feat_idx(env, tempObs);
        //cout << stateIdx << endl;
        int maximizing_action = 0;
        double max_expectation = -numeric_limits<double>::infinity();

        double v = V[s];
        //Eigen::MatrixXd q = Q;
        for (int a = 1; a < env.getNA(); a++) {
            env.setState(tempObs);
            env.step(A_set[a]);
            Eigen::VectorXd nextObs = Eigen::VectorXd(env.getNO());
            env.getState(nextObs);
            int s_prime = obs_to_feat_idx(env, nextObs);
            //cout << reward(tempObs, A_set[a], nextObs) << endl;
            double expectation = reward(tempObs, A_set[a], nextObs) + discount_factor * V[s_prime];
            if (expectation > max_expectation) {
                V[s] = expectation;
                max_expectation = expectation;
                maximizing_action = a;

            }
            //cout << maximizing_action << endl;

        }
        //cout << maximizing_action << endl;
        /*for (int i = 0; i < length; i++) {
            env.getState(tempObs);
            int s = obs_to_feat_idx(env, tempObs);
            //env.setState(tempObs);
            int a = policy(s);
            env.step(A_set[a]);
            Eigen::VectorXi featnext = Eigen::VectorXi(NO_FEATURES);
            Eigen::VectorXd nextObs = Eigen::VectorXd(env.getNO());
            env.getState(nextObs);
            int s_prime = obs_to_feat_idx(env, nextObs);
            //cout << reward(tempObs, A_set[a], nextObs) << endl;
            // double expectation = reward(tempObs, A_set[a], nextObs) + discount_factor * Q.row(s_prime).maxCoeff();
            // if (expectation > Q(s, a)) {
            //    Q(s, a) = expectation;
            //    max_expectation = expectation;
            //    maximizing_action = a;

            // }
            Q(s, a) = reward(tempObs, A_set[a], nextObs) + discount_factor * Q.row(s_prime).maxCoeff();
        }*/
        //cout << "new value: " << stateIdx << endl;
        //Q(s, a) = max_expectation;
        pi[s] = A_set[maximizing_action];
        /*if (abs(v-V[s]) > theta)
        {
            cout << v << ", " << V[s] << ", " << s << endl;
        }*/
        //Eigen::MatrixXd diff = q-Q;
        //theta = max(theta, diff.cwiseAbs().maxCoeff());
        theta = max(theta, abs(v-V[s]));
        //theta2 = min(theta2, abs(v-V[s]));
        //cout << theta << endl;
        // cout << (theta >= delta) << endl;
    } while (theta >= delta);
    return 0.0;
}

double AgentDP::reward(const Eigen::Ref<const Eigen::VectorXd>& o, int a, const Eigen::Ref<const Eigen::VectorXd>& o_prime, bool was_in_left_goal, bool was_in_right_goal) const
{
  // Put your reward here

  return r.calc_reward(o, a, o_prime, was_in_left_goal, was_in_right_goal);
  //return r.calc_reward(o, a, o_prime);
}

void AgentDP::load(std::string path){}

int AgentDP::count_nonzero_values()
{
    int count = 0;
    for (int i = 0; i < SIZE_F_SPACE; i++)
        count += (V[i] != 0);
    return count;
}

int AgentDP::count_nonzero_Q_values()
{
    int count = 0;
    for (int i = 0; i < SIZE_F_SPACE; i++)
    {
        for (int j = 0; j < m_world.getNA(); j++)
            count += (Q(i, j) != 0);
    }
    return count;
}

void AgentDP::save(std::string path)
{
    std::ofstream file;
    file.open(path.c_str(), std::ios::trunc);

    //std::cout << "Saving hyperparameters" << std::endl;
    //file << SIZE_F_SPACE << " " << lr << " " << eligibility << " " << discount << " " << eGreedy << "\n";

    Eigen::IOFormat save(Eigen::FullPrecision, Eigen::DontAlignCols, " ", "\n");
    file << V_star.format(save);
}

double AgentDP::score()
{
    long double sum = 0;
    for(int i = 0; i < SIZE_F_SPACE; i++)
        sum += V_star[i]/SIZE_F_SPACE;
    return sum;
}
