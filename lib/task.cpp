#include "task.h"
#include "const.h"
#include "debug.h"
#include "robot.h"
#include <box2d/b2_distance.h>
#include <box2d/b2_math.h>
#include <box2d/box2d.h>

////////////////////////////////////////////////////////////////////////////////////////////////

void AbstractTask::onLIDARworld (std::shared_ptr<b2World> world,
                                 std::shared_ptr<Robot> robot)
{
    // Do we need to terminate the task?
    CloseObjectDetector nearbyBody (nearbyObstacleDetectionRadius,
                                    nearbyMaxObstacleDetectionAngle);

    threatAngle = nearbyBody.detect (world, robot);

    if (threatAngle)
    {
        if (terminatedEvent)
        {
            bratlogger.log ("Task terminating because of obstacle at: %f",
                            threatAngle.value ());
            TerminationMessage tm (TerminationMessage::ObstacleDetected,
                                   shared_from_this ());
            terminatedEvent (tm);
        }
    }
}

void AbstractTask::onTargetDetected (float r, float phi, int)
{
    targetDistance = r;
    targetAngle = phi;
    bratlogger.log ("Camera target detection: r=%, phi=%", r, phi);
    if (targetDistance <= targetArrivedDistance)
    {
        if (terminatedEvent)
        {
            TerminationMessage tm (TerminationMessage::TargetReached,
                                   shared_from_this ());
            terminatedEvent (tm);
        }
    }
}

void AbstractTask::onGyroTurn (float dphi)
{
    bratlogger.log ("onGyroTurn: dphi=%f", dphi);
    if (targetAngle.has_value ())
    {
        bratlogger.log ("Updating target angle: %f.", targetAngle.value ());
        targetAngle = targetAngle.value () - dphi;
        bratlogger.log ("To target angle: %f.", targetAngle.value ());
    }
}

std::optional<float>
AbstractTask::CloseObjectDetector::detect (std::shared_ptr<b2World> world,
                                           std::shared_ptr<Robot> _robot)
{
    robot = _robot;
    robotPos = robot->getPosition ();
    bratlogger.log ("Running: AbstractTask::CloseObjectDetector::detect.");
    b2AABB aabb;
    aabb.lowerBound
        = robotPos - b2Vec2 (maxDetectionRadius, maxDetectionRadius);
    aabb.upperBound
        = robotPos + b2Vec2 (maxDetectionRadius, maxDetectionRadius);
    nThreats = 0;
    avgThreatAngle = 0;
    world->QueryAABB (this, aabb);
    std::optional<float> threatAngle;
    if (nThreats > 0)
    {
        threatAngle = avgThreatAngle / nThreats;
        bratlogger.log ("Detected more than two threats at: %f",
                        threatAngle.value ());
    }
    return threatAngle;
}

bool AbstractTask::CloseObjectDetector::ReportFixture (b2Fixture *fixture)
{
    bratlogger.log (
        "Running: AbstractTask::CloseObjectDetector::ReportFixture.");
    b2Body *body = fixture->GetBody ();

    // Skip sensors or the robot's own chassis
    if ((fixture->IsSensor ()) || (body == robot->body ())
        || (body->GetUserData ().pointer == ROBOT_FLAG))
    {
        return true; // Keep searching
    }

    b2Vec2 obstaclePos = body->GetPosition ();
    b2Vec2 delta = obstaclePos - robotPos;
    float distance = delta.Length ();
    bratlogger.log (
        "AbstractTask::CloseObjectDetector::ReportFixture: Distance = "
        "%f, bodyflag=%d",
        distance, (int)(body->GetUserData ().pointer));

    // beyond detection radius
    if (distance > maxDetectionRadius)
        return true;

    bratlogger.log ("AbstractTask::CloseObjectDetector: We are close!");

    // Transform world delta vector into robot's local space
    b2Vec2 localDelta = b2MulT (robot->body ()->GetTransform ().q, delta);

    // Calculate relative angle (-PI to +PI). 0 is dead ahead.
    const float tmpRelativeThreatAngle = b2Atan2 (localDelta.y, localDelta.x);

    if (fabs (tmpRelativeThreatAngle) > maxDetectionAngle)
    {
        bratlogger.log (
            "AbstractTask::CloseObjectDetector: But out of sight: %f > %f.",
            tmpRelativeThreatAngle, maxDetectionAngle);
        return true;
    }

    avgThreatAngle += tmpRelativeThreatAngle;
    nThreats++;

    bratlogger.log (
        "AbstractTask::CloseObjectDetector: In field of view: detected!");

    return false;
}

//////////////////////////////////////////////////////////////////////////////////////////

bool AvoidTask::BraitenbergAvoider::detect (std::shared_ptr<b2World> world,
                                            std::shared_ptr<Robot> _robot)
{
    robot = _robot;
    robotPos = robot->getPosition ();
    robotAngle = robot->getAngle ();

    b2AABB aabb;
    aabb.lowerBound
        = robotPos - b2Vec2 (maxDetectionRadius, maxDetectionRadius);
    aabb.upperBound
        = robotPos + b2Vec2 (maxDetectionRadius, maxDetectionRadius);

    assert (aabb.IsValid ());

    world->QueryAABB (this, aabb);

    if (totalWeight > 0.0f)
    {
        averageThreatAngle = totalWeightedAngle / totalWeight;
        return true;
    }
    return false;
}

bool AvoidTask::BraitenbergAvoider::ReportFixture (b2Fixture *fixture)
{
    b2Body *body = fixture->GetBody ();

    // Skip sensors or the robot's own chassis
    if ((fixture->IsSensor ()) || (body == robot->body ())
        || (body->GetUserData ().pointer == ROBOT_FLAG))
    {
        return true; // Keep searching
    }

    b2Vec2 obstaclePos = body->GetPosition ();
    b2Vec2 delta = obstaclePos - robotPos;
    // distance = delta.Length ();

    b2DistanceInput input;
    input.proxyA.Set (robot->getFixtureList ()->GetShape (),
                      0); // Sets up GJK shape representation
    input.proxyB.Set (fixture->GetShape (), 0);
    input.transformA = robot->body ()->GetTransform ();
    input.transformB = fixture->GetBody ()->GetTransform ();
    input.useRadii = true;

    b2SimplexCache cache;
    cache.count = 0; // Fresh cache triggers the base GJK loop

    b2DistanceOutput output;

    // This is the geometry-based GJK execution call
    b2Distance (&output, &cache, &input);

    // This now yields the accurate shape boundary distance!
    const float distance = output.distance;

    // beyond detection radius
    if (distance > maxDetectionRadius)
        return true;

    // Transform world delta vector into robot's local space
    b2Vec2 localDelta = b2MulT (robot->body ()->GetTransform ().q, delta);

    // Calculate relative angle (-PI to +PI). 0 is dead ahead.
    float relativeAngle = b2Atan2 (localDelta.y, localDelta.x);

    if (relativeAngle > maxDetectionAngle)
        return true;

    float weight = 1.0f - (distance / maxDetectionRadius);
    if (weight < 0)
        weight = 0;
    totalWeightedAngle += relativeAngle * weight;
    totalWeight += weight;
    if (distance < minThreatDistance)
    {
        minThreatDistance = distance;
    }
    return true;
}

void AvoidTask::onLIDARworld (std::shared_ptr<b2World> world,
                              std::shared_ptr<Robot> robot)
{
    // We create a Braitenberg escape behaviour
    BraitenbergAvoider braitenbergAvoider (nearbyObstacleDetectionRadius,
                                           M_PI / 8);

    const bool threatsDetected = braitenbergAvoider.detect (world, robot);

    if (threatsDetected)
    {
        float escapeAngularVel
            = RELATIVE_AVOID_STEERING_GAIN * getMaxAngularVelocity ();

        if (RIGHT == avoidDirection)
        {
            escapeAngularVel = -escapeAngularVel;
        }

        bratlogger.log ("====> avoid(%d, %f %f): %f degrees\n", avoidDirection,
                        nearbyObstacleDetectionRadius,
                        braitenbergAvoider.getminThreatDistance (),
                        escapeAngularVel);

        setAngularVelocity (escapeAngularVel);
        nSamplesWithoutThreat = 0;
    }
    else
    {
        nSamplesWithoutThreat++;
        if (nSamplesWithoutThreat > (minLidarAvoidSamples / 2))
            setAngularVelocity (0);
        if (nSamplesWithoutThreat > minLidarAvoidSamples)
        {
            TerminationMessage tm (TerminationMessage::ObstacleAvoided,
                                   shared_from_this ());
            terminatedEvent (tm);
        }
    }
}
