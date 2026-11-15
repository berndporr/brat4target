#pragma once
//box2d robot body and kinematic model
#include "const.h"
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
 */
class Robot
{
  public:
    Robot() = default;
    Robot (std::shared_ptr<b2World> world,
           b2Transform start = b2Transform_zero)
    {
        b2BodyDef m_bodyDef;
        m_bodyDef.type = b2_dynamicBody;
        m_bodyDef.position.Set (0.0f, 0.0f);
        m_body = world->CreateBody (&m_bodyDef);
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
    b2Transform getTransform() {
      return m_body->GetTransform();
    }

    /**
     * @brief Gets the robot body. Can be a nullptr.
     */
    b2Body *body () const { return m_body; }

    /**
     * @brief Gets a list of the fixtures. Can be a nullptr.
     */
    b2Fixture *getFixtureList () const
    {
        if (m_body)
            return m_body->GetFixtureList ();
        return nullptr;
    }

    /**
     * @brief Gets the position of the robot.
     */
    b2Vec2 getPosition () const
    {
        if (m_body)
            return m_body->GetPosition ();
        return b2Vec2 (0, 0);
    }

    /**
     * @brief Gets the angle of the robot.
     */
    float getAngle () const { return m_body->GetAngle (); }

    /**
     * @brief Removes the robot from its world.
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
