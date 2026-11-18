#include "lib/robot.h"
class AbstractTargetTracker
{

  public:
    AbstractTargetTracker ();
    virtual void onTargetDetected (float r, float phi, int index = 0) = 0;
    virtual void onGyroTurn (float dphi) = 0;
    virtual void onLIDARworld (std::shared_ptr<b2World> world, Robot robot)
        = 0;
    using UpdateTargetEvent = std::function<void (float)>;
    void registerUpdateTargetEvent (UpdateTargetEvent te) { targetEvent = te; }

  protected:
    UpdateTargetEvent targetEvent;
};

class SimpleTargetTracker : public AbstractTargetTracker
{
  public:
    virtual void onTargetDetected (float r, float phi, int index = 0)
    {
        targetAngle = phi;
        if (targetEvent)
            targetEvent (targetAngle.value ());
    };
    virtual void onGyroTurn (float dphi)
    {
        if (targetAngle)
        {
            targetAngle = targetAngle.value () - dphi;
            if (targetEvent)
                targetEvent (targetAngle.value ());
        }
    }
    virtual void onLIDARworld (std::shared_ptr<b2World> world, Robot robot) {};

  private:
    std::optional<float> targetAngle = 0;
};