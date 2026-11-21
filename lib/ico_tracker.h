#pragma once
#include <vector>
#include "robot.h"
#include "targettracker.h"

class ICO_tracker : public TargetTrackerInterface
{

  public:
    ICO_tracker (const int _numNeurons, const float _learningRate);
    void onTargetDetected (float r, float phi, int index = 0);
    void onGyroTurn (float dphi);
    void onLIDARworld (std::shared_ptr<b2World> world, Robot robot);

  private:
    int numNeurons{};
    float learningRate{};
    std::vector<float> w{};
    std::vector<float> x{};
    std::vector<float> accInputs{};
    void setDphi (const float dphi);
    void accumulateDPhi ();
    void learningStep (const float phi);
    void setTargetAngle (const float phi);
};