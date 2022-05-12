#include <iostream>
#include <ctime>
#include <sstream>
#include <fstream>

#include <QApplication>

#include "HaxBall.h"
#include "HaxBallGui.h"

// #include "AgentTD.h"
#include "AgentDP.h"
#include "AgentTDmaster.h"
#include "AgentTDSituative.h"

void render(BaseAgent &agent, int argc, char **argv)
{
  QApplication app(argc, argv);
  HaxBallGui gui(agent);
  gui.show();
  gui.playGame(10.0, 5e4);
  app.exec();
}

int main(int argc, char **argv)
{
  std::cout << "Hello Group Group-5!" << std::endl;

  std::stringstream ss;
  ss << "/media/tergaim/Data/Travail/experiment/db" << time(0);
  std::string savePath = ss.str();
  std::string loadPath = "/media/tergaim/Data/Travail/experiment/db1625561168";

  ss << "_trainingdata.csv";
  std::string saveTrainingDataPath = ss.str();

  // Create your own agent and provide the parameters
  //  Agent agent(/*alpha*/   0.001,
  //              /*gamma*/   0.99,
  //              /*epsilon*/ 0.01);

  // To compile the code use the random agent
  AgentTDmaster agent;
  // agent.save(savePath);

  // agent.load(loadPath);
  // render(agent, argc, argv);


  // This could be a learning loop, extend it as required and make sure, that your
  // agent stores everything on the disk
  const int epochs = 100;

  std::ofstream trainingSave;
  trainingSave.open(saveTrainingDataPath.c_str(), std::ios::trunc);

  int length = 1e4;
  int trajectories = 1e4;
  int threads = 9;

  trainingSave << "length;trajectories;threads;\n" << std::flush;
  trainingSave << length << ";" << trajectories << ";" << threads << ";\n" << std::flush;
  trainingSave << "epoch;score;time;\n" << std::flush;

  int t_render = 0;
  for (int i = agent.trainingStage; i < epochs; ++i)
  {
    time_t begin = time(0);
    std::cout << "\nRound " << i + 1 << " of " << epochs << std::endl;

    std::cout << "Training started" << std::endl;
    agent.setHyperParam(0.00003, 0.8, 0.8, 1.0 / (2*i+1));
    // if(i < 15)
    //   agent.setHyperParam(0.00003, 0.8, 0.8, 0);

    // if(i%5 == 0) 
    //   agent.setHyperParam(0.001, 0.8, 0.8, 0.2);


    double score = agent.training(length,
                   trajectories,
                   threads) / (length * trajectories);
    double score_mean = agent.score();

    int t = time(0) - begin;
    std::cout << "Training round finished after " << t << "s." << std::endl;
    int tfinish = t*(epochs-i) + t_render*((int) (epochs-i)/10);
    std::cout << "Estimated time before finish: " << tfinish/60 << "'" << tfinish%60 << std::endl;
    std::cout << "Cumulative reward for the policy: " << score << std::endl;
    std::cout << "Mean reward for the policy: " << score_mean << std::endl;
    if (i % 5 == 0)
    {
      time_t beginrender = time(0);
      agent.save(savePath);
      // std::cout << "Testing and rendering" << std::endl;
      // render(agent, argc, argv);
      t_render = time(0) - beginrender;
    }
    trainingSave << i << ";" << score << ";" << score_mean << ";" << t << ";\n" << std::flush;
    agent.trainingStage ++;
  }
  trainingSave.close();
  agent.save(savePath);
  std::cout << "Testing and rendering" << std::endl;
  render(agent, argc, argv);

  return 0;
}
