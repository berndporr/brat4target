#include "task.h"
#include "const.h"
#include "debug.h"
#include "robot.h"
#include <box2d/b2_distance.h>
#include <box2d/b2_math.h>
#include <box2d/box2d.h>

#include <box2d/box2d.h>
#include <cmath>

float getFixtureArea (const b2Fixture *fixture)
{
    if (!fixture)
        return 0.0f;

    const b2Shape *shape = fixture->GetShape ();

    switch (shape->GetType ())
    {
    case b2Shape::e_circle:
    {
        const b2CircleShape *circle
            = static_cast<const b2CircleShape *> (shape);
        return b2_pi * circle->m_radius * circle->m_radius;
    }

    case b2Shape::e_polygon:
    {
        const b2PolygonShape *poly
            = static_cast<const b2PolygonShape *> (shape);
        float area = 0.0f;
        int32 count = poly->m_count;

        // Shoelace formula
        for (int32 i = 0; i < count; ++i)
        {
            b2Vec2 p1 = poly->m_vertices[i];
            b2Vec2 p2 = poly->m_vertices[(i + 1) % count];
            area += b2Cross (p1, p2); // p1.x * p2.y - p1.y * p2.x
        }

        return std::abs (area) * 0.5f;
    }

    case b2Shape::e_edge:
    case b2Shape::e_chain:
    default:
        return 0.0f;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////

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
        bratlogger.logVariable ("targetAngle", targetAngle.value ());
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
        bratlogger.log ("Detected threats at: %f", threatAngle.value ());
        bratlogger.logVariable ("threatAngle", threatAngle.value ());
        bratlogger.logVariable ("nThreats", nThreats);
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
        "%f, bodyflag=%d, area=%f",
        distance, (int)(body->GetUserData ().pointer),
        getFixtureArea (fixture));

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

std::optional<float>
AvoidTask::BraitenbergAvoider::detect (std::shared_ptr<b2World> world,
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

    accDistance = 0;
    nDistance = 0;

    world->QueryAABB (this, aabb);

    std::optional<float> normdistance;

    if (nDistance > 1)
    {
        normdistance = accDistance / nDistance;
    }
    return normdistance;
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

    accDistance += (1.0f - (distance / maxDetectionRadius));
    nDistance++;
    return true;
}

void AvoidTask::onLIDARworld (std::shared_ptr<b2World> world,
                              std::shared_ptr<Robot> robot)
{
    // We create a Braitenberg escape behaviour
    BraitenbergAvoider braitenbergAvoider (nearbyObstacleDetectionRadius,
                                           M_PI / 8);

    auto threatsDetected = braitenbergAvoider.detect (world, robot);

    if (threatsDetected)
    {
        float escapeAngularVel = RELATIVE_AVOID_STEERING_GAIN
                                 * getMaxAngularVelocity ()
                                 * threatsDetected.value ();

        if (RIGHT == avoidDirection)
        {
            escapeAngularVel = -escapeAngularVel;
        }

        bratlogger.log (
            "====> avoid, dir=%d, detrad=%f, threat=%f: ang vel = %f\n",
            avoidDirection, nearbyObstacleDetectionRadius,
            threatsDetected.value (), escapeAngularVel);

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
