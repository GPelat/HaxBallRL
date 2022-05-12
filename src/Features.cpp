
#include "Features.h"

int obs_to_features(const HaxBall &haxball, const Eigen::Ref<const Eigen::VectorXd> obs)
{
    // Extract vectors from observations
    Eigen::Vector2d player_pos = obs.segment(0, 2);
    Eigen::Vector2d ball_pos = obs.segment(2, 2);
    Eigen::Vector2d ball_vel = obs.segment(4, 2);

    // Calculate euclidean distance from player to ball
    Eigen::Vector2d diff = ball_pos - player_pos;
    double distance = diff.norm();

    // Check if the player is in shooting range
    bool shoot = (distance < 0.15 + haxball.getRadiusBall() + haxball.getRadiusPlayer());

    // Calculate future ball position
    Eigen::Vector2d future_ball_pos = ball_pos + 2 * haxball.getTimeDelta() * ball_vel;
    float distfut = std::max(abs(player_pos(0) - future_ball_pos(0)), abs(player_pos(0) - future_ball_pos(0)));
    bool closer = distfut < distance;
    int dist = distance / 2;

    // Horizontal compass directions to current ball pos
    int current_relative_horizontal_pos_to_ball = 0;
    if (player_pos[0] < ball_pos[0] - haxball.getRadiusPlayer())
        current_relative_horizontal_pos_to_ball = -1;
    else if (player_pos[0] > ball_pos[0] + haxball.getRadiusPlayer())
        current_relative_horizontal_pos_to_ball = 1;

    // Vertical compass directions to current ball pos
    int current_relative_vertical_pos_to_ball = 0;
    if (player_pos[1] < ball_pos[1] - haxball.getRadiusPlayer())
        current_relative_vertical_pos_to_ball = -1;
    else if (player_pos[1] > ball_pos[1] + haxball.getRadiusPlayer())
        current_relative_vertical_pos_to_ball = 1;

    // Horizontal compass directions to future ball pos
    int future_relative_horizontal_pos_to_ball = 0;
    if (player_pos[0] < future_ball_pos[0] - haxball.getRadiusPlayer())
        future_relative_horizontal_pos_to_ball = -1;
    else if (player_pos[0] > future_ball_pos[0] + haxball.getRadiusPlayer())
        future_relative_horizontal_pos_to_ball = 1;

    // Vertical compass directions to future ball pos
    int future_relative_vertical_pos_to_ball = 0;
    if (player_pos[1] < future_ball_pos[1] - haxball.getRadiusPlayer())
        future_relative_vertical_pos_to_ball = -1;
    else if (player_pos[1] > future_ball_pos[1] + haxball.getRadiusPlayer())
        future_relative_vertical_pos_to_ball = 1;

    // Obtain positions of left and right goal
    Eigen::Vector2d right_goal_pos(haxball.getGoalRight().center().rx(), haxball.getGoalRight().center().ry());

    int p_x_position = (player_pos[0] + 4) / 8.0 * 9.99;
    int p_y_position = (player_pos[1] + 2) / 4.0 * 4.99;
    int b_x_position = (ball_pos[0] + 4) / 8.0 * 9.99;
    int b_y_position = (ball_pos[1] + 2) / 4.0 * 4.99;
    int fb_x_position = (future_ball_pos[0] + 4) / 8.0 * 9.99;
    int fb_y_position = (future_ball_pos[1] + 2) / 4.0 * 4.99;

    // bool right_side = false;
    // if (player_pos[0] > 0)
    // {
    //     right_side = true;
    // }
    Eigen::VectorXi feat(NO_FEATURES);
    feat << p_x_position, p_y_position, b_x_position, b_y_position, fb_x_position, fb_y_position;
    // feat << shoot, current_relative_horizontal_pos_to_ball, current_relative_vertical_pos_to_ball,
    //     future_relative_horizontal_pos_to_ball, future_relative_vertical_pos_to_ball,
    //     discretized_y_position, discretized_x_position;

    return feat2idx(feat);
}

int feat2idx(const Eigen::Ref<const Eigen::VectorXi> &feat)
{
    // int total = SIZE_F_SPACE / 2; // shoot : 0, 1
    // int idx = total * feat(0);

    // // total /= 5; // distance to ball : in [[0,4]]
    // // idx += total * feat(1);

    // // total /= 2; // closer : 0, 1
    // // idx += total * feat(2);

    // total /= 3; // ball WE : -1,0,1
    // idx += total * (feat(1) + 1);

    // total /= 3; // ball NS : -1,0,1
    // idx += total * (feat(2) + 1);

    // total /= 3; // nextball WE : -1,0,1
    // idx += total * (feat(3) + 1);

    // total /= 3; // nextball NS : -1,0,1
    // idx += total * (feat(4) + 1);

    // total /= 11; // y position : in [[0, 10]]
    // idx += total * feat(5);

    // // // total /= 2; // right side : 0, 1
    // // // idx += total * feat(6);

    // total /= 21; //x position : in [[0, 20]]
    // idx += total * (feat(6));

    // return idx;

    int total = SIZE_F_SPACE / 10;
    int idx = total * feat(0);
    total /= 5;
    idx += total * feat(1);
    total /= 10;
    idx += total * feat(2);
    total /= 5;
    idx += total * feat(3);
    total /= 10;
    idx += total * feat(4);
    total /= 5;
    idx += total * feat(5);

    return idx;
}

int obs_to_feat_idx(const HaxBall &haxball, const Eigen::Ref<const Eigen::VectorXd> obs)
{
    /*// Extract vectors from observations
    Eigen::Vector2d player_pos = obs.segment(0, 2);
    Eigen::Vector2d ball_pos = obs.segment(2, 2);
    Eigen::Vector2d ball_vel = obs.segment(4, 2);

    // Calculate euclidean distance from player to ball
    Eigen::Vector2d diff = ball_pos - player_pos;
    double distance = diff.norm();

    // Check if the player is in shooting range
    bool shoot = (distance < 0.15 + haxball.getRadiusBall() + haxball.getRadiusPlayer());

    // Calculate future ball position
    Eigen::Vector2d future_ball_pos = ball_pos + 2 * haxball.getTimeDelta() * ball_vel;
    int dist = std::max(abs(player_pos(0) - ball_pos(0)), abs(player_pos(0) - ball_pos(0)));
    int distfut = std::max(abs(player_pos(0) - future_ball_pos(0)), abs(player_pos(0) - future_ball_pos(0)));

    // Horizontal compass directions to current ball pos
    int current_relative_horizontal_pos_to_ball = 0;
    if (player_pos[0] < ball_pos[0] - haxball.getRadiusPlayer())
        current_relative_horizontal_pos_to_ball = -1;
    else if (player_pos[0] > ball_pos[0] + haxball.getRadiusPlayer())
        current_relative_horizontal_pos_to_ball = 1;

    // Vertical compass directions to current ball pos
    int current_relative_vertical_pos_to_ball = 0;
    if (player_pos[1] < ball_pos[1] - haxball.getRadiusPlayer())
        current_relative_vertical_pos_to_ball = -1;
    else if (player_pos[1] > ball_pos[1] + haxball.getRadiusPlayer())
        current_relative_vertical_pos_to_ball = 1;

    // Horizontal compass directions to future ball pos
    int future_relative_horizontal_pos_to_ball = 0;
    if (player_pos[0] < future_ball_pos[0] - haxball.getRadiusPlayer())
        future_relative_horizontal_pos_to_ball = -1;
    else if (player_pos[0] > future_ball_pos[0] + haxball.getRadiusPlayer())
        future_relative_horizontal_pos_to_ball = 1;

    // Vertical compass directions to future ball pos
    int future_relative_vertical_pos_to_ball = 0;
    if (player_pos[1] < future_ball_pos[1] - haxball.getRadiusPlayer())
        future_relative_vertical_pos_to_ball = -1;
    else if (player_pos[1] > future_ball_pos[1] + haxball.getRadiusPlayer())
        future_relative_vertical_pos_to_ball = 1;

    // Obtain positions of left and right goal
    Eigen::Vector2d right_goal_pos(haxball.getGoalRight().center().rx(), haxball.getGoalRight().center().ry());

    int discretized_y_position = (player_pos[1] + 2) / 4.0 * 10;
    int discretized_x_position = (player_pos[0] + 4) / 8.0 * 20;

    int total = SIZE_F_SPACE / 2; // shoot : 0, 1
    int idx = total * shoot;

    total /= 8; // ball distance : in [[0, 7]]
    idx += total * dist;

    total /= 8; // future ball distance : in [[0, 7]]
    idx += total * distfut;

    total /= 3; // ball NS : -1,0,1
    idx += total * (current_relative_horizontal_pos_to_ball + 1);

    total /= 3; // ball WE : -1,0,1
    idx += total * (current_relative_vertical_pos_to_ball + 1);

    total /= 3; // nextball NS : -1,0,1
    idx += total * (future_relative_horizontal_pos_to_ball + 1);

    total /= 3; // nextball WE : -1,0,1
    idx += total * (future_relative_vertical_pos_to_ball + 1);

    total /= 11; // y position : in [[0, 10]]
    idx += total * discretized_y_position;

    // total /= 2; // right side : 0, 1
    // idx += total * feat(6);

    total /= 21; //rightgoal : in [[0, 20]]
    idx += total * (discretized_x_position);*/

    /*// Extract vectors from observations
    Eigen::Vector2d player_pos = obs.segment(0, 2);
    Eigen::Vector2d ball_pos = obs.segment(2, 2);
    Eigen::Vector2d ball_vel = obs.segment(4, 2);

    // Calculate euclidean distance from player to ball
    Eigen::Vector2d diff = ball_pos - player_pos;
    double distance = diff.norm();

    // Check if the player is in shooting range
    bool shoot = (distance < 0.15 + haxball.getRadiusBall() + haxball.getRadiusPlayer());

    // Calculate future ball position
    Eigen::Vector2d future_ball_pos = ball_pos + 2 * haxball.getTimeDelta() * ball_vel;
    float distfut = std::max(abs(player_pos(0) - future_ball_pos(0)), abs(player_pos(0) - future_ball_pos(0)));
    bool closer = distfut < distance;
    int dist = distance / 2;

    // Horizontal compass directions to current ball pos
    int current_relative_horizontal_pos_to_ball = 0;
    if (player_pos[0] < ball_pos[0] - haxball.getRadiusPlayer())
        current_relative_horizontal_pos_to_ball = -1;
    else if (player_pos[0] > ball_pos[0] + haxball.getRadiusPlayer())
        current_relative_horizontal_pos_to_ball = 1;

    // Vertical compass directions to current ball pos
    int current_relative_vertical_pos_to_ball = 0;
    if (player_pos[1] < ball_pos[1] - haxball.getRadiusPlayer())
        current_relative_vertical_pos_to_ball = -1;
    else if (player_pos[1] > ball_pos[1] + haxball.getRadiusPlayer())
        current_relative_vertical_pos_to_ball = 1;

    // Horizontal compass directions to future ball pos
    int future_relative_horizontal_pos_to_ball = 0;
    if (player_pos[0] < future_ball_pos[0] - haxball.getRadiusPlayer())
        future_relative_horizontal_pos_to_ball = -1;
    else if (player_pos[0] > future_ball_pos[0] + haxball.getRadiusPlayer())
        future_relative_horizontal_pos_to_ball = 1;

    // Vertical compass directions to future ball pos
    int future_relative_vertical_pos_to_ball = 0;
    if (player_pos[1] < future_ball_pos[1] - haxball.getRadiusPlayer())
        future_relative_vertical_pos_to_ball = -1;
    else if (player_pos[1] > future_ball_pos[1] + haxball.getRadiusPlayer())
        future_relative_vertical_pos_to_ball = 1;

    // Obtain positions of left and right goal
    Eigen::Vector2d right_goal_pos(haxball.getGoalRight().center().rx(), haxball.getGoalRight().center().ry());

    int p_x_position = (player_pos[0] + 4) / 8.0 * 10;
    int p_y_position = (player_pos[1] + 2) / 4.0 * 5;
    int b_x_position = (ball_pos[0] + 4) / 8.0 * 10;
    int b_y_position = (ball_pos[1] + 2) / 4.0 * 5;
    int fb_x_position = (future_ball_pos[0] + 4) / 8.0 * 10;
    int fb_y_position = (future_ball_pos[1] + 2) / 4.0 * 5;

    int total = SIZE_F_SPACE/11;
    int idx = total * p_x_position;
    total /= 6;
    idx += total * p_y_position;
    total /= 11;
    idx += total * b_x_position;
    total /= 6;
    idx += total * b_y_position;
    total /= 11;
    idx += total * fb_x_position;
    total /= 6;
    idx += total * fb_y_position;*/

    // Extract vectors from observations
    Eigen::Vector2d player_pos = obs.segment(0, 2);
    Eigen::Vector2d ball_pos = obs.segment(2, 2);
    Eigen::Vector2d ball_vel = obs.segment(4, 2);

    // Calculate euclidean distance from player to ball
    Eigen::Vector2d diff = ball_pos - player_pos;
    double distance = diff.norm();

    // Check if the player is in shooting range
    bool shoot = (distance < 0.15 + haxball.getRadiusBall() + haxball.getRadiusPlayer());

    // Calculate future ball position
    Eigen::Vector2d future_ball_pos = ball_pos + 2 * haxball.getTimeDelta() * ball_vel;
    float distfut = std::max(abs(player_pos(0) - future_ball_pos(0)), abs(player_pos(0) - future_ball_pos(0)));
    bool closer = distfut < distance;
    int dist = distance / 2;

    // Horizontal compass directions to current ball pos
    int current_relative_horizontal_pos_to_ball = 0;
    if (player_pos[0] < ball_pos[0] - haxball.getRadiusPlayer())
        current_relative_horizontal_pos_to_ball = -1;
    else if (player_pos[0] > ball_pos[0] + haxball.getRadiusPlayer())
        current_relative_horizontal_pos_to_ball = 1;

    // Vertical compass directions to current ball pos
    int current_relative_vertical_pos_to_ball = 0;
    if (player_pos[1] < ball_pos[1] - haxball.getRadiusPlayer())
        current_relative_vertical_pos_to_ball = -1;
    else if (player_pos[1] > ball_pos[1] + haxball.getRadiusPlayer())
        current_relative_vertical_pos_to_ball = 1;

    // Horizontal compass directions to future ball pos
    int future_relative_horizontal_pos_to_ball = 0;
    if (player_pos[0] < future_ball_pos[0] - haxball.getRadiusPlayer())
        future_relative_horizontal_pos_to_ball = -1;
    else if (player_pos[0] > future_ball_pos[0] + haxball.getRadiusPlayer())
        future_relative_horizontal_pos_to_ball = 1;

    // Vertical compass directions to future ball pos
    int future_relative_vertical_pos_to_ball = 0;
    if (player_pos[1] < future_ball_pos[1] - haxball.getRadiusPlayer())
        future_relative_vertical_pos_to_ball = -1;
    else if (player_pos[1] > future_ball_pos[1] + haxball.getRadiusPlayer())
        future_relative_vertical_pos_to_ball = 1;

    // Obtain positions of left and right goal
    Eigen::Vector2d right_goal_pos(haxball.getGoalRight().center().rx(), haxball.getGoalRight().center().ry());

    // int p_x_position = (player_pos[0] + 4) / 8.0 * 10;
    // int p_y_position = (player_pos[1] + 2) / 4.0 * 5;
    // int b_x_position = (ball_pos[0] + 4) / 8.0 * 10;
    // int b_y_position = (ball_pos[1] + 2) / 4.0 * 5;
    // int fb_x_position = (future_ball_pos[0] + 4) / 8.0 * 10;
    // int fb_y_position = (future_ball_pos[1] + 2) / 4.0 * 5;

    int discretized_x_position = (player_pos[0] + 4) / 8.0 * 10;
    int discretized_y_position = (player_pos[1] + 2) / 4.0 * 5;

    int total = SIZE_F_SPACE / 2; // shoot : 0, 1
    int idx = total * shoot;

    // total /= 5; // distance to ball : in [[0,4]]
    // idx += total * dist;

    // total /= 2; // closer : 0, 1
    // idx += total * feat(2);

    total /= 3; // ball WE : -1,0,1
    idx += total * (current_relative_horizontal_pos_to_ball + 1);

    total /= 3; // ball NS : -1,0,1
    idx += total * (current_relative_vertical_pos_to_ball + 1);

    total /= 3; // nextball WE : -1,0,1
    idx += total * (future_relative_horizontal_pos_to_ball + 1);

    total /= 3; // nextball NS : -1,0,1
    idx += total * (future_relative_vertical_pos_to_ball + 1);

    total /= 11; // y position : in [[0, 10]]
    idx += total * discretized_x_position;

    // // total /= 2; // right side : 0, 1
    // // idx += total * feat(6);

    total /= 21; //x position : in [[0, 20]]
    idx += total * (discretized_y_position);

    return idx;
}


int obs_to_approach_feat_idx(const HaxBall &haxball, const Eigen::Ref<const Eigen::VectorXd> obs)
{
    // Extract vectors from observations
    Eigen::Vector2d player_pos = obs.segment(0, 2);
    Eigen::Vector2d ball_pos = obs.segment(2, 2);
    Eigen::Vector2d ball_vel = obs.segment(4, 2);

    // Calculate euclidean distance from player to ball
    Eigen::Vector2d diff = ball_pos - player_pos;
    double distance = diff.norm();

    // Check if the player is in shooting range
    bool shoot = (distance < 0.15 + haxball.getRadiusBall() + haxball.getRadiusPlayer());

    // Calculate future ball position
    Eigen::Vector2d future_ball_pos = ball_pos + 2 * haxball.getTimeDelta() * ball_vel;
    float distfut = std::max(abs(player_pos(0) - future_ball_pos(0)), abs(player_pos(0) - future_ball_pos(0)));
    bool closer = distfut < distance;
    int dist = distance / 2;

    // Horizontal compass directions to current ball pos
    int current_relative_horizontal_pos_to_ball = 0;
    if (player_pos[0] < ball_pos[0] - haxball.getRadiusPlayer())
        current_relative_horizontal_pos_to_ball = -1;
    else if (player_pos[0] > ball_pos[0] + haxball.getRadiusPlayer())
        current_relative_horizontal_pos_to_ball = 1;

    // Vertical compass directions to current ball pos
    int current_relative_vertical_pos_to_ball = 0;
    if (player_pos[1] < ball_pos[1] - haxball.getRadiusPlayer())
        current_relative_vertical_pos_to_ball = -1;
    else if (player_pos[1] > ball_pos[1] + haxball.getRadiusPlayer())
        current_relative_vertical_pos_to_ball = 1;

    // Horizontal compass directions to future ball pos
    int future_relative_horizontal_pos_to_ball = 0;
    if (player_pos[0] < future_ball_pos[0] - haxball.getRadiusPlayer())
        future_relative_horizontal_pos_to_ball = -1;
    else if (player_pos[0] > future_ball_pos[0] + haxball.getRadiusPlayer())
        future_relative_horizontal_pos_to_ball = 1;

    // Vertical compass directions to future ball pos
    int future_relative_vertical_pos_to_ball = 0;
    if (player_pos[1] < future_ball_pos[1] - haxball.getRadiusPlayer())
        future_relative_vertical_pos_to_ball = -1;
    else if (player_pos[1] > future_ball_pos[1] + haxball.getRadiusPlayer())
        future_relative_vertical_pos_to_ball = 1;

    // Obtain positions of left and right goal
    Eigen::Vector2d right_goal_pos(haxball.getGoalRight().center().rx(), haxball.getGoalRight().center().ry());

    // int p_x_position = (player_pos[0] + 4) / 8.0 * 10;
    // int p_y_position = (player_pos[1] + 2) / 4.0 * 5;
    // int b_x_position = (ball_pos[0] + 4) / 8.0 * 10;
    // int b_y_position = (ball_pos[1] + 2) / 4.0 * 5;
    // int fb_x_position = (future_ball_pos[0] + 4) / 8.0 * 10;
    // int fb_y_position = (future_ball_pos[1] + 2) / 4.0 * 5;

    int discretized_x_position = (player_pos[0] + 4) / 8.0 * 10;
    int discretized_y_position = (player_pos[1] + 2) / 4.0 * 5;

    int total = SIZE_F_SPACE_APPROACH;
    int idx = 0;

    // total /= 2; // shoot : 0, 1
    // idx += total * shoot;

    // total /= 5; // distance to ball : in [[0,4]]
    // idx += total * dist;

    // total /= 2; // closer : 0, 1
    // idx += total * feat(2);

    // total /= 3; // ball WE : -1,0,1
    // idx += total * (current_relative_horizontal_pos_to_ball + 1);

    // total /= 3; // ball NS : -1,0,1
    // idx += total * (current_relative_vertical_pos_to_ball + 1);

    total /= 3; // nextball WE : -1,0,1
    idx += total * (future_relative_horizontal_pos_to_ball + 1);

    total /= 3; // nextball NS : -1,0,1
    idx += total * (future_relative_vertical_pos_to_ball + 1);

    total /= 11; // y position : in [[0, 10]]
    idx += total * discretized_x_position;

    // total /= 2; // right side : 0, 1
    // idx += total * feat(6);

    total /= 21; //x position : in [[0, 20]]
    idx += total * (discretized_y_position);

    return idx;
}


int obs_to_shoot_feat_idx(const HaxBall &haxball, const Eigen::Ref<const Eigen::VectorXd> obs)
{
    // Extract vectors from observations
    Eigen::Vector2d player_pos = obs.segment(0, 2);
    Eigen::Vector2d ball_pos = obs.segment(2, 2);
    Eigen::Vector2d ball_vel = obs.segment(4, 2);

    // Calculate euclidean distance from player to ball
    Eigen::Vector2d diff = ball_pos - player_pos;
    double distance = diff.norm();

    // Check if the player is in shooting range
    bool shoot = (distance < 0.15 + haxball.getRadiusBall() + haxball.getRadiusPlayer());

    // Calculate future ball position
    Eigen::Vector2d future_ball_pos = ball_pos + 2 * haxball.getTimeDelta() * ball_vel;
    float distfut = std::max(abs(player_pos(0) - future_ball_pos(0)), abs(player_pos(0) - future_ball_pos(0)));
    bool closer = distfut < distance;
    int dist = distance / 2;

    // Horizontal compass directions to current ball pos
    int current_relative_horizontal_pos_to_ball = 0;
    if (player_pos[0] < ball_pos[0] - haxball.getRadiusPlayer())
        current_relative_horizontal_pos_to_ball = -1;
    else if (player_pos[0] > ball_pos[0] + haxball.getRadiusPlayer())
        current_relative_horizontal_pos_to_ball = 1;

    // Vertical compass directions to current ball pos
    int current_relative_vertical_pos_to_ball = 0;
    if (player_pos[1] < ball_pos[1] - haxball.getRadiusPlayer())
        current_relative_vertical_pos_to_ball = -1;
    else if (player_pos[1] > ball_pos[1] + haxball.getRadiusPlayer())
        current_relative_vertical_pos_to_ball = 1;

    // Horizontal compass directions to future ball pos
    int future_relative_horizontal_pos_to_ball = 0;
    if (player_pos[0] < future_ball_pos[0] - haxball.getRadiusPlayer())
        future_relative_horizontal_pos_to_ball = -1;
    else if (player_pos[0] > future_ball_pos[0] + haxball.getRadiusPlayer())
        future_relative_horizontal_pos_to_ball = 1;

    // Vertical compass directions to future ball pos
    int future_relative_vertical_pos_to_ball = 0;
    if (player_pos[1] < future_ball_pos[1] - haxball.getRadiusPlayer())
        future_relative_vertical_pos_to_ball = -1;
    else if (player_pos[1] > future_ball_pos[1] + haxball.getRadiusPlayer())
        future_relative_vertical_pos_to_ball = 1;

    // Obtain positions of left and right goal
    Eigen::Vector2d right_goal_pos(haxball.getGoalRight().center().rx(), haxball.getGoalRight().center().ry());

    // int p_x_position = (player_pos[0] + 4) / 8.0 * 10;
    // int p_y_position = (player_pos[1] + 2) / 4.0 * 5;
    // int b_x_position = (ball_pos[0] + 4) / 8.0 * 10;
    // int b_y_position = (ball_pos[1] + 2) / 4.0 * 5;
    // int fb_x_position = (future_ball_pos[0] + 4) / 8.0 * 10;
    // int fb_y_position = (future_ball_pos[1] + 2) / 4.0 * 5;

    int discretized_x_position = (player_pos[0] + 4) / 8.0 * 10;
    int discretized_y_position = (player_pos[1] + 2) / 4.0 * 5;

    int total = SIZE_F_SPACE_SHOOT / 2; // shoot : 0, 1
    int idx = total * shoot;

    // total /= 5; // distance to ball : in [[0,4]]
    // idx += total * feat(1);

    // total /= 2; // closer : 0, 1
    // idx += total * feat(2);

    total /= 3; // ball WE : -1,0,1
    idx += total * (current_relative_horizontal_pos_to_ball + 1);

    total /= 3; // ball NS : -1,0,1
    idx += total * (current_relative_vertical_pos_to_ball + 1);

    total /= 3; // nextball WE : -1,0,1
    idx += total * (future_relative_horizontal_pos_to_ball + 1);

    total /= 3; // nextball NS : -1,0,1
    idx += total * (future_relative_vertical_pos_to_ball + 1);

    total /= 11; // y position : in [[0, 10]]
    idx += total * discretized_x_position;

    // // total /= 2; // right side : 0, 1
    // // idx += total * feat(6);

    total /= 21; //x position : in [[0, 20]]
    idx += total * (discretized_y_position);

    return idx;
}

int obs_to_dumbbot(const HaxBall &m_world, const Eigen::Ref<const Eigen::VectorXd> obs)
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

    bool must_go_left = false;
    int left_vertical = 0;

    if (p2t(0) < 0 && target(0) < right_goal_pos(0))
        must_go_left = true;
    if (abs(player_pos[1] - future_ball_pos[1]) > d_target)
    {
        left_vertical = (target(1) < 0) ? 1 : 3;
        if (abs(player_pos[1] - future_ball_pos[1]) > d_target + 0.2)
            left_vertical++;
    }

    int relative_horizontal = 0;
    if (p2t(0) < -0.02)
        relative_horizontal = 1;
    else if (p2t(0) > 0.02)
        relative_horizontal = 2;

    int relative_vertical = 0;
    if (p2t(1) < -0.02)
        relative_vertical = 1;
    else if (p2t(1) > 0.02)
        relative_vertical = 2;

    bool shoot = false;
    if (p2t.squaredNorm() < 0.005)
        shoot = true; // Shoot

    bool close_to_wall_x = false, close_to_wall_y = false, bcw_x = false, bcw_y = false;
    if(ball_pos(0) < -3.9 || ball_pos(0) > 3.9) bcw_x = true;
    if(ball_pos(1) < -1.9 || ball_pos(1) > 1.9) bcw_y = true;
    if(player_pos(0) < -3.9 || player_pos(0) > 3.9) close_to_wall_x = true;
    if(player_pos(1) < -1.9 || player_pos(1) > 1.9) close_to_wall_y = true;

    // must_go_left, left_vertical, relative_horizontal, relative_vertical, shoot, pcw_xy, bcw_xy: 2 x 5 x 3 x 3 x 2 x 4 x 4 = 2880
    int total = 2880 / 2;
    int idx = total * must_go_left;

    total /= 5;
    idx += total * left_vertical;
    total /= 3;
    idx += total * relative_horizontal;
    total /= 3;
    idx += total * relative_vertical;
    total /= 2;
    idx += total * shoot;
    total /= 2;
    idx += total * bcw_x;
    total /= 2;
    idx += total * bcw_y;
    total /= 2;
    idx += total * close_to_wall_x;
    total /= 2;
    idx += total * close_to_wall_y;
    return idx;
}