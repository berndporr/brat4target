#pragma once
//box2d robot body and kinematic model
#include "const.h"
#include "lib/debug.h"
#include <box2d/b2_body.h>
#include <box2d/b2_fixture.h>
#include <box2d/b2_math.h>
#include <box2d/b2_polygon_shape.h>
#include <box2d/b2_world.h>
#include <math.h>
#include <memory>
#include <stdlib.h>
#include <unistd.h>

/**
 * @brief Robot class for Box2D simulation.
 * Creates the robot in the world.
 * This acts as a container of the 2D robot body and stores the raw pointer to the body.
 * Note that the default constructor creates an instance without a valid robot. Before
 * using it it needs to be overwritten by a robot which has been created in its world.
 */
class Robot
{
  public:
    Robot () = default;
    /**
     * @brief Constructor which adds the robot to the world.
     * The default is that the robot is placed at (0,0) but can
     * be placed at other places with the start argument.
     */
    Robot (std::shared_ptr<b2World> world,
           b2Transform start = b2Transform_zero)
    {
        b2BodyDef m_bodyDef;
        m_bodyDef.type = b2_dynamicBody;
        m_bodyDef.position.Set (0.0f, 0.0f);
        m_body = world->CreateBody (&m_bodyDef);
        if (!m_body)
            return;
        m_body->GetUserData ().pointer
            = reinterpret_cast<uintptr_t> (ROBOT_FLAG);
        b2Vec2 center (ROBOT_BOX_OFFSET_X, ROBOT_BOX_OFFSET_Y);
        b2PolygonShape m_box;
        m_box.SetAsBox (ROBOT_HALFWIDTH, ROBOT_HALFLENGTH, center,
                        ROBOT_BOX_OFFSET_ANGLE);
        b2FixtureDef fixtureDef;
        fixtureDef.shape = &m_box;
        fixtureDef.friction = 0;
        m_body->CreateFixture (&fixtureDef);
        m_body->SetTransform (start.p, start.q.GetAngle ());
    }

    /**
     * @brief Gets the robot position and angle in the world.
     * The transform contains all the info about the robot so that it can be re-created
     * at this position.
     */
    b2Transform getTransform ()
    {
        if (m_body)
            return m_body->GetTransform ();
        bratlogger.log ("BUG: Robot getTransform() but robot body is null!");
        return b2Transform_zero;
    }

    /**
     * @brief Gets a list of the fixtures.
     * Can be a nullptr even if the robot body exists.
     */
    b2Fixture *getFixtureList () const
    {
        if (m_body)
            return m_body->GetFixtureList ();
        bratlogger.log ("BUG: Robot getFixtureList() but robot body is null!");
        return nullptr;
    }

    /**
     * @brief Gets the position of the robot.
     */
    b2Vec2 getPosition () const
    {
        if (m_body)
            return m_body->GetPosition ();
        bratlogger.log ("BUG: Robot getPosition() but robot body is null!");
        return b2Vec2 (0, 0);
    }

    /**
     * @brief Gets the angle of the robot.
     */
    float getAngle () const
    {
        if (m_body)
            return m_body->GetAngle ();
        bratlogger.log ("BUG: Robot getAngle() but robot body is null!");
        return 0;
    }

    /**
     * @brief Sets the angle of the robot.
     */
    void setAngle (float angle)
    {
        if (m_body)
            m_body->SetTransform (m_body->GetPosition (), angle);
    }

    /**
     * @brief Sets the linear velocity of the robot.
     */
    void setLinearVelocity (b2Vec2 v)
    {
        if (m_body)
            m_body->SetLinearVelocity (v);
    }

    /**
     * @brief Sets the angular velocity of the robot.
     */
    void setAngularVelocity (float v)
    {
        if (m_body)
            m_body->SetAngularVelocity (v);
    }

    /**
     * @brief Determines if the body pointers match.
     */
    bool isItMe(const b2Body* b) {
        return b == m_body;
    }

    /**
     * @brief Removes the robot from its world and destroys it.
     */
    void destruct ()
    {
        if (m_body)
        {
            m_body->GetWorld ()->DestroyBody (m_body);
            m_body = nullptr;
        }
    }

  private:
    b2Body *m_body = nullptr;
};
