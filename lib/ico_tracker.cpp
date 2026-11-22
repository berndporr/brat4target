#include <vector>

#include "ico_tracker.h"

ICO_tracker::ICO_tracker (const int _numNeurons, const float _learningRate)
    : numNeurons (_numNeurons), learningRate (_learningRate)
{
    for (int i = 0; i < numNeurons; i++)
    {
        w.push_back (1);
        x.push_back (0);
        accInputs.push_back (0);
    }
    // Gyro DC Offset is a constant input of 1
    x[1] += 1;
}

void ICO_tracker::onTargetDetected (float phi)
{
    learningStep (phi);
    setTargetAngle (phi);
    for (int i = 0; i < numNeurons; i++)
    {
        accInputs[i] = 0;
    }
}

void ICO_tracker::onGyroTurn (float dphi)
{
    if (!targetAngle) return;
    setDphi (dphi);
    accumulateDPhi ();
}

void ICO_tracker::setDphi (const float dphi) { x[0] = dphi; }

void ICO_tracker::accumulateDPhi ()
{
    for (int i = 0; i < numNeurons; i++)
    {
        targetAngle = targetAngle.value () + x[i] * w[i];
        accInputs[i] += x[i];
    }
}

void ICO_tracker::learningStep (const float phi)
{
    if (!targetAngle) return;
    float error = phi - targetAngle.value();
    for (int i = 0; i < numNeurons; i++)
    {
        w[i] += learningRate * error * accInputs[i];
    }
}

void ICO_tracker::setTargetAngle (const float phi) { targetAngle = phi; }
