#include "task.h"
#include "const.h"
#include "debug.h"
#include "robot.h"
#include <box2d/b2_distance.h>
#include <box2d/b2_math.h>
#include <box2d/box2d.h>
#include <cstdio>

////////////////////////////////////////////////////////////////////////////////////////////////

void AbstractTask::onLIDARworld (std::shared_ptr<b2World> world,
                                 std::shared_ptr<Robot> robot)
{
    // Do we need to terminate the task?
    CloseObjectDetector nearbyBody (nearbyObstacleDetectionRadius,
                                    nearbyMaxObstacleDetectionAngle);

    if (nearbyBody.detect (world, robot))
    {
        if (terminatedEvent)
        {
            TerminationMessage tm;
            tm.terminationReason = TerminationMessage::ObstacleDetected;
            terminatedEvent (tm);
        }
    }
}

void AbstractTask::onTargetDetected (float r, float phi) { targetAngle = phi; }

void AbstractTask::onGyroTurn (float dphi)
{
    if (targetAngle.has_value ())
    {
        fprintf (stderr, "Target angle: %f\n", targetAngle.value ());
        auto v = getLinearVelocity();
        targetAngle = targetAngle.value () - dphi;
    }
}

bool AbstractTask::CloseObjectDetector::detect (std::shared_ptr<b2World> world,
                                                std::shared_ptr<Robot> _robot)
{
    robot = _robot;
    robotPos = robot->getPosition ();
    if (DEBUG)
        fprintf (stderr, "AbstractTask::CloseObjectDetector::detect.\n");
    b2AABB aabb;
    aabb.lowerBound
        = robotPos - b2Vec2 (maxDetectionRadius, maxDetectionRadius);
    aabb.upperBound
        = robotPos + b2Vec2 (maxDetectionRadius, maxDetectionRadius);
    if (DEBUG)
        fprintf (stderr, "QueryAABB\n");
    world->QueryAABB (this, aabb);
    return nullptr != detectedBody;
}

bool AbstractTask::CloseObjectDetector::ReportFixture (b2Fixture *fixture)
{
    if (DEBUG)
        fprintf (stderr,
                 "AbstractTask::CloseObjectDetector::ReportFixture.\n");
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
    if (DEBUG)
        fprintf (stderr, "Distance = %f, bodyflag=%d\n", distance,
                 (int)(body->GetUserData ().pointer));

    // beyond detection radius
    if (distance > maxDetectionRadius)
        return true;

    if (DEBUG)
        fprintf (stderr, "We are close.\n");

    // Transform world delta vector into robot's local space
    b2Vec2 localDelta = b2MulT (robot->body ()->GetTransform ().q, delta);

    // Calculate relative angle (-PI to +PI). 0 is dead ahead.
    relativeThreatAngle = b2Atan2 (localDelta.y, localDelta.x);

    if (fabs (relativeThreatAngle) > maxDetectionAngle)
    {
        fprintf (stderr, "But out of sight: %f > %f\n", relativeThreatAngle,
                 maxDetectionAngle);
        return true;
    }

    detectedBody = body;

    if (DEBUG)
        fprintf (stderr, "In field of view: detected!!!!\n");

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

    // check if we are within the robot
    if (distance < ROBOT_HALFLENGTH)
        return true;
    if (distance < ROBOT_HALFWIDTH)
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
    return true;
}

void AvoidTask::onLIDARworld (std::shared_ptr<b2World> world,
                              std::shared_ptr<Robot> robot)
{
    // We create a Braitenberg escape behaviour
    BraitenbergAvoider braitenbergAvoider (nearbyObstacleDetectionRadius,
                                           M_PI / 2);

    const bool threatsDetected = braitenbergAvoider.detect (world, robot);

    if (threatsDetected)
    {
        const float averageThreatAngle
            = braitenbergAvoider.getAverageThreatAngle ();

        // REPULSION: Steer in the OPPOSITE direction of the threat
        const float targetSteeringAngle
            = -averageThreatAngle * avoidTaskSteeringGain;
        setAngularVelocity (targetSteeringAngle);
        nSamplesWithoutThreat = 0;
    }
    else
    {
        nSamplesWithoutThreat++;
        if (nSamplesWithoutThreat > minLidarAvoidSamples)
        {
            TerminationMessage tm;
            tm.terminationReason = TerminationMessage::ObstacleAvoided;
            terminatedEvent (tm);
        }
    }
}
