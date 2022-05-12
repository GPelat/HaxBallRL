#include "Rewards.h"
#include "Features.h"
#include <iostream>

Reward::Reward(int mode, HaxBall &haxball) : mode(mode), haxball(haxball)
{
    if ((mode != SCALING_MODE) && (mode != SEQUENTIAL_MODE))
        throw std::invalid_argument("This mode is not available. Please choose either SCALING_MODE (0) or SEQUENTIAL_MODE (1).");
}

Reward::~Reward()
{
}

double Reward::calc_reward_new(const Eigen::Ref<const Eigen::VectorXd> &o, int a, const Eigen::Ref<const Eigen::VectorXd> &o_prime, bool was_in_left_goal, bool was_in_right_goal) const
{
    // Check for configured mode since this functionality is only supported in combination with the scaling approach
    if (mode != SCALING_MODE)
        throw std::invalid_argument("Sequential mode is active, please supply the current training stage.");

    // std::cout << "action : " << a << "   ";
    double reward = 0.0;
    HaxBall m_world;
    Eigen::VectorXi stateF = Eigen::VectorXi(NO_FEATURES);
    int stateIdx = obs_to_features(m_world, o);
    // std::cout << "\r" << a << "  \t" << stateF.transpose() << "  \t";

    // Extract necessary vectors from observations
    Eigen::Vector2d current_player_pos = o.segment(0, 2);
    Eigen::Vector2d current_ball_pos = o.segment(2, 2);
    Eigen::Vector2d current_ball_speed = o.segment(4, 2);


    // Extract necessary vectors from observations
    Eigen::Vector2d next_player_pos = o_prime.segment(0, 2);
    Eigen::Vector2d next_ball_pos = o_prime.segment(2, 2);
    Eigen::Vector2d next_ball_speed = o_prime.segment(4, 2);

    Eigen::Vector2d rightGoalPos;
    rightGoalPos << 0.9 * 4, 0;
    // Reward the agent for moving closer to the next ball position
    /*Guillaume: reward += REWARD_BALL * 2000 * ((next_ball_pos - current_player_pos).norm() - (next_ball_pos - next_player_pos).norm());
    // std::cout << reward << "  ";

    // // Reward the agent for moving closer to the target position
    // reward += REWARD_BALL * 2000 * ((target - current_player_pos).norm() - (target - next_player_pos).norm());

    // // Eigen::Vector2d current_pos_grid, next_pos_grid;
    // // current_pos_grid << (current_player_pos[0] + 4) / 8.0 * 20 , (current_player_pos[1] + 2) / 4.0 * 10;
    // // next_pos_grid << (next_player_pos[0] + 4) / 8.0 * 20 , (next_player_pos[1] + 2) / 4.0 * 10;
    // // reward += REWARD_BALL * 2000 * ((next_ball_pos - current_pos_grid).norm() - (next_ball_pos - next_pos_grid).norm());
    // // std::cout << reward << "  ";

    // // penalize for going further right than the ball
    // if (current_ball_pos(0) < current_player_pos(0))
    // {
    //     int malus = (abs(current_player_pos[1]) < abs(current_ball_pos[1]) + 0.1 + haxball.getRadiusBall() + haxball.getRadiusPlayer()) ? 0.1 : 0;
    //     reward += REWARD_BALL * 0.3 * (current_ball_pos(0) - current_player_pos(0)) - malus;
    // }
    // // std::cout << reward << "  ";

    // reward += REWARD_BALL * 100 * ((current_ball_pos - right_goal_pos).norm() - (next_ball_pos - right_goal_pos).norm());
    // // std::cout << reward << "  ";

    // Reward the agent for scoring into the right goal
    if (next_ball_pos.squaredNorm() < 0.01 && next_ball_speed.squaredNorm() < 0.01)
    {
        if (current_ball_pos[0] > 0)
        {
            reward += REWARD_GOAL;
        }
        // Punish the agent for scoring into the left goal
        else
        {
            reward -= 0.2*REWARD_GOAL;
        }
    }
    // std::cout << reward << "  ";

    if (a < 2 && (current_ball_pos - next_ball_pos).squaredNorm() < 0.1)
        reward -= 10;
    // std::cout << reward << std::endl;*/

    //Eigen::Vector2d offset;
    //offset << 0.25 + 0.15 + 0.2, 0;
    //current_ball_pos[0] = current_ball_pos[0] - offset[0];
    //next_ball_pos[0] = next_ball_pos[0] - offset[0];
    Eigen::Vector2d diff = current_ball_pos - current_player_pos;
    double distance = diff.norm();

    // Reward the agent for moving closer to the next ball position
    // if (((next_ball_pos[0] > 3.7) || (next_ball_pos[0] < -3.7)) && ((next_ball_pos[1] > 1.9) || (next_ball_pos[1] < -1.9)))
    // {
    //     reward -=  0.0001;
    // }
    // else
    {
        // reward += REWARD_BALL * 2000 * ((next_ball_pos - current_player_pos).norm() - (next_ball_pos - next_player_pos).norm());
        // // std::cout << reward << "  ";

        // if(current_ball_pos(0) < current_player_pos(0)){
        //     //int coef = (abs(current_player_pos[1]) > abs(current_ball_pos[1]) + haxball.getRadiusPlayer() + haxball.getRadiusBall()) ? 1 : 10;
        //     if (abs(current_player_pos[1] - current_ball_pos[1]) > haxball.getRadiusPlayer() + haxball.getRadiusBall())
        //     {
        //         int coef = 1;
        //         reward += REWARD_BALL * 0.3 * 1 * (current_ball_pos(0) - current_player_pos(0));
        //         //std::cout << "Punish normal move :" << reward << "  " << std::endl;

        //     }
        //     else {
        //         reward += REWARD_BALL * 0.3 * 1 * (current_ball_pos(0) - current_player_pos(0)) - 2;
        //         //std::cout << "Punish corridor move :" << reward << "  " << std::endl;
        //     }
        // }
    }


    // std::cout << reward << "  ";

    // penalize for going further right than the ball

    // std::cout << reward << "  ";

    // if(next_ball_pos(0) > current_ball_pos(0))
    //     reward += REWARD_BALL * 1000 * ((current_ball_pos - rightGoalPos).norm() - (next_ball_pos - rightGoalPos).norm());
    // std::cout << reward << "  ";

    // Reward the agent for scoring into the right goal
    if (was_in_right_goal)
    {
        //std::cout << "Right goal" << std::endl;
        reward += REWARD_GOAL;
        //std::cout << "Reward goal: " << reward << "  " << std::endl;
    }

    // Punish the agent for scoring into the left goal
    if (was_in_left_goal)
    {
        //std::cout << "Left goal" << std::endl;
        reward -= 0.5*REWARD_GOAL;
        //std::cout << "Punish goal: " << reward << "  " << std::endl;
    }
    // std::cout  << reward << "  ";
    // std::cout << reward << "  ";

    // if(a < 2 && (current_ball_pos-next_ball_pos).squaredNorm() < 0.001)
    //     reward -= 10;

    // std::cout  << reward << "  " << std::endl;

    return reward;
}

double Reward::calc_reward(const Eigen::Ref<const Eigen::VectorXd> &o, int a, const Eigen::Ref<const Eigen::VectorXd> &o_prime, bool was_in_left_goal, bool was_in_right_goal) const
{
    // Check for configured mode since this functionality is only supported in combination with the scaling approach
    if (mode != SCALING_MODE)
        throw std::invalid_argument("Sequential mode is active, please supply the current training stage.");

    double reward = 0.0;
    HaxBall m_world;
    int stateIdx = obs_to_features(m_world, o);
    // std::cout << "\r" << a << "  \t" << stateF.transpose() << "  \t";

    // Extract necessary vectors from observations
    Eigen::Vector2d current_player_pos = o.segment(0, 2);
    Eigen::Vector2d current_ball_pos = o.segment(2, 2);

    // Extract necessary vectors from observations
    Eigen::Vector2d next_player_pos = o_prime.segment(0, 2);
    Eigen::Vector2d next_ball_pos = o_prime.segment(2, 2);
    Eigen::Vector2d next_ball_speed = o_prime.segment(4, 2);

    // Eigen::Vector2d future_ball_pos = current_ball_pos + 2 * m_world.getTimeDelta() * current_ball_speed;
    // Eigen::Vector2d right_goal_pos(m_world.getGoalRight().center().rx(), m_world.getGoalRight().center().ry());
    // double d_target = 0.1 + m_world.getRadiusBall() + m_world.getRadiusPlayer();
    // Eigen::Vector2d target = d_target * (future_ball_pos - right_goal_pos).normalized() + future_ball_pos;

    Eigen::Vector2d rightGoalPos;
    rightGoalPos << 0.9 * 4, 0;
    // Reward the agent for moving closer to the next ball position
    reward += REWARD_BALL * 2000 * ((next_ball_pos - current_player_pos).norm() - (next_ball_pos - next_player_pos).norm());
    // std::cout << reward << "  ";

    // penalize for going further right than the ball
    if(current_ball_pos(0) < current_player_pos(0)){
        int coef = (abs(current_player_pos[1]) > abs(current_ball_pos[1]) + haxball.getRadiusPlayer()) ? 2 : 1;
        reward += REWARD_BALL * 0.3 * coef * (current_ball_pos(0) - current_player_pos(0));
        }
    // std::cout << reward << "  ";

    reward += REWARD_BALL * 100 * ((current_ball_pos - rightGoalPos).norm() - (next_ball_pos - rightGoalPos).norm());
    // std::cout << reward << "  ";

    // Reward the agent for scoring into the right goal
    if (next_ball_pos.squaredNorm() < 0.01 && next_ball_speed.squaredNorm() < 0.01)
    {
        if (current_ball_pos[0] > 0)
        {
            reward += REWARD_GOAL;
        }
        // Punish the agent for scoring into the left goal
        // else
        // {
        //     reward -= REWARD_GOAL;
        // }
    }
    // std::cout << reward << "  ";

    if(a < 2 && (current_ball_pos-next_ball_pos).squaredNorm() < 0.1)
        reward -= 10;
    // std::cout << reward << std::endl;

}

double Reward::calc_reward(int training_stage, const Eigen::Ref<const Eigen::VectorXd> &o, int a, const Eigen::Ref<const Eigen::VectorXd> &o_prime) const
{
    // Check for configured mode since this functionality is only supported in combination with the sequential approach
    if (mode != SEQUENTIAL_MODE)
        throw std::invalid_argument("Scaling mode is active, please supply the current training stage.");

    // Extract necessary vectors from observations
    Eigen::Vector2d current_player_pos = o.segment(0, 2);
    Eigen::Vector2d current_ball_pos = o.segment(2, 2);

    // Extract necessary vectors from observations
    Eigen::Vector2d next_player_pos = o_prime.segment(0, 2);
    Eigen::Vector2d next_ball_pos = o_prime.segment(2, 2);

    if (training_stage == TRAINING_STAGE_MOVE)
    {
        // Reward the agent for moving towards the left upper corner
        Eigen::Vector2d left_corner(-4, -2);
        if ((left_corner - next_player_pos).norm() < (left_corner - current_player_pos).norm())
        {
            return REWARD_MOVE;
        }
    }
    else if (training_stage == TRAINING_STAGE_BALL)
    {
        // Reward the agent for moving closer to the next ball position
        if ((next_ball_pos - next_player_pos).norm() < (next_ball_pos - current_player_pos).norm())
        {
            return REWARD_BALL * 10 * ((next_ball_pos - current_player_pos).norm() - (next_ball_pos - next_player_pos).norm());
        }
    }
    else if (training_stage == TRAINING_STAGE_SHOOT)
    {
        // Reward the agent for reaching shooting ranged combined with shooting towards the goal
        if (((current_ball_pos - current_player_pos).norm() < 0.15 + haxball.getRadiusBall() + haxball.getRadiusPlayer()) and a % 4 == 1)
        {
            Eigen::Hyperplane<double, 2> ball_moving_dir = Eigen::Hyperplane<double, 2>::Through(current_ball_pos, next_ball_pos);
            Eigen::Vector2d right_goal_top_left_corner = Eigen::Vector2d(haxball.getGoalRight().topLeft().rx(), haxball.getGoalRight().topLeft().ry());
            Eigen::Vector2d right_goal_bottom_left_corner = Eigen::Vector2d(haxball.getGoalRight().bottomLeft().rx(), haxball.getGoalRight().bottomLeft().ry());
            Eigen::Hyperplane<double, 2> goal_line = Eigen::Hyperplane<double, 2>::Through(right_goal_top_left_corner, right_goal_bottom_left_corner);
            Eigen::Vector2d intersec_right = Eigen::Vector2d(goal_line.intersection(ball_moving_dir));
            Eigen::Vector2d left_goal_top_right_corner = Eigen::Vector2d(haxball.getGoalRight().topRight().rx(), haxball.getGoalRight().topRight().ry());
            Eigen::Vector2d left_goal_bottom_right_corner = Eigen::Vector2d(haxball.getGoalRight().bottomRight().rx(), haxball.getGoalRight().bottomRight().ry());
            Eigen::Hyperplane<double, 2> goal_line_left = Eigen::Hyperplane<double, 2>::Through(left_goal_top_right_corner, left_goal_bottom_right_corner);
            Eigen::Vector2d intersec_left = Eigen::Vector2d(goal_line_left.intersection(ball_moving_dir));
            if ((intersec_right(1) >= right_goal_top_left_corner(1)) && (intersec_right(1) <= right_goal_bottom_left_corner(1)))
                return REWARD_SHOOT;
            else if ((intersec_left(1) >= left_goal_top_right_corner(1)) && (intersec_left(1) <= left_goal_bottom_right_corner(1)))
                return -REWARD_SHOOT;
            else
                return 0.0;
        }
        else
        {
            return 0.0;
        }
    }
    else if (training_stage == TRAINING_STAGE_GOAL)
    {
        // Reward the agent for scoring into the right goal
        if (haxball.getGoalRight().contains(current_ball_pos[0], current_ball_pos[1]))
        {
            return REWARD_GOAL;
        }
        // Punish the agent for scoring into the left goal
        else if (haxball.getGoalLeft().contains(current_ball_pos[0], current_ball_pos[1]))
        {
            return -REWARD_GOAL;
        }
    }
    else
    {
        return 0.0;
    }
}

double Reward::calc_reward_approach(const Eigen::Ref<const Eigen::VectorXd> &o, int a, const Eigen::Ref<const Eigen::VectorXd> &o_prime) const
{
    // Check for configured mode since this functionality is only supported in combination with the scaling approach
    if (mode != SCALING_MODE)
        throw std::invalid_argument("Sequential mode is active, please supply the current training stage.");

    double reward = 0.0;
    HaxBall m_world;
    int stateIdx = obs_to_features(m_world, o);
    // std::cout << "\r" << a << "  \t" << stateF.transpose() << "  \t";

    // Extract necessary vectors from observations
    Eigen::Vector2d current_player_pos = o.segment(0, 2);
    Eigen::Vector2d current_ball_pos = o.segment(2, 2);
    Eigen::Vector2d current_ball_speed = o.segment(4, 2);

    // Extract necessary vectors from observations
    Eigen::Vector2d next_player_pos = o_prime.segment(0, 2);
    Eigen::Vector2d next_ball_pos = o_prime.segment(2, 2);
    Eigen::Vector2d next_ball_speed = o_prime.segment(4, 2);

    // Eigen::Vector2d future_ball_pos = current_ball_pos + 2 * m_world.getTimeDelta() * current_ball_speed;
	// Eigen::Vector2d right_goal_pos(m_world.getGoalRight().center().rx(), m_world.getGoalRight().center().ry());
	// double d_target = 0.1 + m_world.getRadiusBall() + m_world.getRadiusPlayer();
	// Eigen::Vector2d target = d_target * (future_ball_pos - right_goal_pos).normalized() + future_ball_pos;

    // // Reward the agent for moving closer to the target position
    // reward += REWARD_BALL * 2000 * ((target - current_player_pos).norm() - (target - next_player_pos).norm());

    // // Eigen::Vector2d current_pos_grid, next_pos_grid;
    // // current_pos_grid << (current_player_pos[0] + 4) / 8.0 * 20 , (current_player_pos[1] + 2) / 4.0 * 10;
    // // next_pos_grid << (next_player_pos[0] + 4) / 8.0 * 20 , (next_player_pos[1] + 2) / 4.0 * 10;
    // // reward += REWARD_BALL * 2000 * ((next_ball_pos - current_pos_grid).norm() - (next_ball_pos - next_pos_grid).norm());
    // // std::cout << reward << "  ";

    // // penalize for going further right than the ball
    // if (current_ball_pos(0) < current_player_pos(0))
    // {
    //     int malus = (abs(current_player_pos[1]) < abs(current_ball_pos[1]) + 0.1 + haxball.getRadiusBall() + haxball.getRadiusPlayer()) ? 0.1 : 0;
    //     reward += REWARD_BALL * 0.3 * (current_ball_pos(0) - current_player_pos(0)) - malus;
    // }
    // // std::cout << reward << "  ";

    // reward += REWARD_BALL * 100 * ((current_ball_pos - right_goal_pos).norm() - (next_ball_pos - right_goal_pos).norm());
    // // std::cout << reward << "  ";

    // Reward the agent for scoring into the right goal
    //if (next_ball_pos.squaredNorm() < 0.01 && next_ball_speed.squaredNorm() < 0.01)

    Eigen::Vector2d rightGoalPos;
    rightGoalPos << 0.9 * 4, 0;

    Eigen::Vector2d offset;
    offset << 0.25 + 0.15 + 0.2, 0;
    current_ball_pos[0] = current_ball_pos[0] - offset[0];
    next_ball_pos[0] = next_ball_pos[0] - offset[0];
    Eigen::Vector2d diff = current_ball_pos - current_player_pos;
    double distance = diff.norm();


    //if (current_player_pos[0] < current_player_pos[0] - 0.15 - 0.25)
    //{
    // Reward the agent for moving closer to the next ball position
    if (((next_ball_pos[0] > 3.7) || (next_ball_pos[0] < -3.7)) && ((next_ball_pos[1] > 1.9) || (next_ball_pos[1] < -1.9)))
    {
        reward -=  0.0001;
    }
    else
    {
        reward += REWARD_BALL * 2000 * ((next_ball_pos - current_player_pos).norm() - (next_ball_pos - next_player_pos).norm());
        //std::cout << "Reward move :" << reward << "  " << std::endl;

        if(current_ball_pos(0) < current_player_pos(0)){
            //int coef = (abs(current_player_pos[1]) > abs(current_ball_pos[1]) + haxball.getRadiusPlayer() + haxball.getRadiusBall()) ? 1 : 10;
            if ((current_player_pos[1] > current_ball_pos[1] + haxball.getRadiusPlayer() + haxball.getRadiusBall()) || (current_player_pos[1] + haxball.getRadiusPlayer() + haxball.getRadiusBall() < current_ball_pos[1] ))
            {
                int coef = 1;
                reward += REWARD_BALL * 0.3 * coef * (current_ball_pos(0) - current_player_pos(0));
                //std::cout << "Punish normal move :" << reward << "  " << std::endl;

            }
            else {
                int coef = 2;
                reward += REWARD_BALL * 0.3 * coef * (current_ball_pos(0) - current_player_pos(0));
                //std::cout << "Punish corridor move :" << reward << "  " << std::endl;
            }
        }
    }

    if ((distance < 0.15 + 0.25 + 0.3) && (current_player_pos[0] < current_ball_pos[0] - 0.15 - 0.25))
        reward += 0;
    // std::cout << reward << "  ";

    // penalize for going further right than the ball

    // std::cout << reward << "  ";

    // reward += REWARD_BALL * 100 * ((current_ball_pos - rightGoalPos).norm() - (next_ball_pos - rightGoalPos).norm());
    // std::cout << reward << "  ";


    /*if(a < 2 && (current_ball_pos-next_ball_pos).squaredNorm() < 0.1)
        reward -= 0;//10;*/
    // std::cout << reward << std::endl;



    return reward;
}

double Reward::calc_reward_shoot(const Eigen::Ref<const Eigen::VectorXd> &o, int a, const Eigen::Ref<const Eigen::VectorXd> &o_prime) const
{
    // Check for configured mode since this functionality is only supported in combination with the scaling approach
    if (mode != SCALING_MODE)
        throw std::invalid_argument("Sequential mode is active, please supply the current training stage.");

    double reward = 0.0;
    HaxBall m_world;
    int stateIdx = obs_to_features(m_world, o);
    // std::cout << "\r" << a << "  \t" << stateF.transpose() << "  \t";

    // Extract necessary vectors from observations
    Eigen::Vector2d current_player_pos = o.segment(0, 2);
    Eigen::Vector2d current_ball_pos = o.segment(2, 2);

    // Extract necessary vectors from observations
    Eigen::Vector2d next_player_pos = o_prime.segment(0, 2);
    Eigen::Vector2d next_ball_pos = o_prime.segment(2, 2);
    Eigen::Vector2d next_ball_speed = o_prime.segment(4, 2);

    Eigen::Vector2d rightGoalPos;
    rightGoalPos << 0.9 * 4, 0;

    if (((next_ball_pos[0] > 3.8) || (next_ball_pos[0] < -3.8)) && ((next_ball_pos[1] > 1.9) || (next_ball_pos[1] < -1.9)))
    {
        reward -=  0.0001;
    }
    else
    {
        reward += REWARD_BALL * 2000 * ((next_ball_pos - current_player_pos).norm() - (next_ball_pos - next_player_pos).norm());
        //std::cout << "Reward move :" << reward << "  " << std::endl;

        if(current_ball_pos(0) < current_player_pos(0)){
            //int coef = (abs(current_player_pos[1]) > abs(current_ball_pos[1]) + haxball.getRadiusPlayer() + haxball.getRadiusBall()) ? 1 : 10;
            if ((current_player_pos[1] > current_ball_pos[1] + haxball.getRadiusPlayer() + haxball.getRadiusBall()) || (current_player_pos[1] + haxball.getRadiusPlayer() + haxball.getRadiusBall() < current_ball_pos[1] ))
            {
                int coef = 1;
                reward += REWARD_BALL * 0.3 * coef * (current_ball_pos(0) - current_player_pos(0));
                //std::cout << "Punish normal move :" << reward << "  " << std::endl;

            }
            else {
                int coef = 5;
                reward += REWARD_BALL * 0.3 * coef * (current_ball_pos(0) - current_player_pos(0));
                //std::cout << "Punish corridor move :" << reward << "  " << std::endl;
            }
        }

    }
    // std::cout << "Reward and punish move :" << reward << "  " << std::endl;

    reward += REWARD_BALL * 100 * ((current_ball_pos - rightGoalPos).norm() - (next_ball_pos - rightGoalPos).norm());
    // std::cout << reward << "  ";

    // Reward the agent for scoring into the right goal
    /*if (next_ball_pos.squaredNorm() < 0.01 && next_ball_speed.squaredNorm() < 0.01)
    {
        if (current_ball_pos[0] > 0)
        {
            reward += REWARD_GOAL;
            //std::cout << "Reward goal: " << reward << "  " << std::endl;
        }
        // Punish the agent for scoring into the left goal
        else
        {
            reward -= REWARD_GOAL;
            //std::cout << "Punish goal: " << reward << "  " << std::endl;
        }
    }*/
    //std::cout << "Reward goal: " << reward << "  " << std::endl;
    // std::cout << reward << "  ";

    if(a < 2 && (current_ball_pos-next_ball_pos).squaredNorm() < 0.1)
        reward -= 10;
    // std::cout << reward << std::endl;

    return reward;
}

double Reward::calc_reward_db(const Eigen::Ref<const Eigen::VectorXd> &o, int a, const Eigen::Ref<const Eigen::VectorXd> &o_prime) const
{
    // Check for configured mode since this functionality is only supported in combination with the scaling approach
    if (mode != SCALING_MODE)
        throw std::invalid_argument("Sequential mode is active, please supply the current training stage.");

    double reward = 0.0;
    // HaxBall m_world;
    // Eigen::VectorXi stateF = Eigen::VectorXi(NO_FEATURES);
    // int stateIdx = obs_to_features(m_world, o);
    // // std::cout << "\r" << a << "  \t" << stateF.transpose() << "  \t";

    // // Extract necessary vectors from observations
    // Eigen::Vector2d current_player_pos = o.segment(0, 2);
    Eigen::Vector2d current_ball_pos = o.segment(2, 2);
    // Eigen::Vector2d current_ball_speed = o.segment(4, 2);


    // // Extract necessary vectors from observations
    // Eigen::Vector2d next_player_pos = o_prime.segment(0, 2);
    Eigen::Vector2d next_ball_pos = o_prime.segment(2, 2);
    Eigen::Vector2d next_ball_speed = o_prime.segment(4, 2);

    // Eigen::Vector2d future_ball_pos = current_ball_pos + 2 * m_world.getTimeDelta() * current_ball_speed;
    // Eigen::Vector2d right_goal_pos(m_world.getGoalRight().center().rx(), m_world.getGoalRight().center().ry());
    // double d_target = 0.1 + m_world.getRadiusBall() + m_world.getRadiusPlayer();
    // Eigen::Vector2d target = d_target * (future_ball_pos - right_goal_pos).normalized() + future_ball_pos;

    // // Reward the agent for moving closer to the target position
    // reward += REWARD_BALL * 2000 * ((target - current_player_pos).norm() - (target - next_player_pos).norm());

    // Eigen::Vector2d current_pos_grid, next_pos_grid;
    // current_pos_grid << (current_player_pos[0] + 4) / 8.0 * 20 , (current_player_pos[1] + 2) / 4.0 * 10;
    // next_pos_grid << (next_player_pos[0] + 4) / 8.0 * 20 , (next_player_pos[1] + 2) / 4.0 * 10;
    // reward += REWARD_BALL * 2000 * ((next_ball_pos - current_pos_grid).norm() - (next_ball_pos - next_pos_grid).norm());
    // std::cout << reward << "  ";

    // penalize for going further right than the ball
    // if (current_ball_pos(0) < current_player_pos(0))
    // {
    //     int malus = (abs(current_player_pos[1] - current_ball_pos[1]) < 0.1 + haxball.getRadiusBall() + haxball.getRadiusPlayer()) ? 1 : 0;
    //     reward += REWARD_BALL * 0.3 * (current_ball_pos(0) - current_player_pos(0)) - malus;
    // }
    // // std::cout << reward << "  ";

    // reward += REWARD_SHOOT * 100 * ((current_ball_pos - right_goal_pos).norm() - (next_ball_pos - right_goal_pos).norm());
    // std::cout << reward << "  ";

    // Reward the agent for scoring into the right goal
    if (current_ball_pos.squaredNorm() > 1 && next_ball_pos.squaredNorm() < 0.01 && next_ball_speed.squaredNorm() < 0.01)
    {
        if (current_ball_pos[0] > 0)
        {
            reward += REWARD_GOAL;
        }
        // Punish the agent for scoring into the left goal
        else
        {
            reward -= 0.5*REWARD_GOAL;
        }
    }
    // std::cout << reward << "  ";

    return reward;
}

