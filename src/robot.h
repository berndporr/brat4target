#pragma once
//box2d robot body and kinematic model
#include "const.h"
#include <box2d/b2_body.h>
#include <box2d/b2_fixture.h>
#include <box2d/b2_polygon_shape.h>
#include <box2d/b2_world.h>
#include <math.h>
#include <memory>
#include <stdlib.h>
#include <unistd.h>

/**
 * @brief Robot class for Box2D simulation.
 * 
 */
class Robot
{
  private:
    b2FixtureDef fixtureDef;
    b2Body *m_body = nullptr;
    b2PolygonShape m_box;

  public:
    Robot () = delete;

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
        m_box.SetAsBox (ROBOT_HALFWIDTH, ROBOT_HALFLENGTH, center,
                        ROBOT_BOX_OFFSET_ANGLE);
        fixtureDef.shape = &m_box;
        fixtureDef.friction = 0;
        m_body->CreateFixture (&fixtureDef);
        body ()->SetTransform (start.p, start.q.GetAngle ());
    }

    b2Body *body () const { return m_body; }
    b2Fixture *getFixtureList () const { return m_body->GetFixtureList (); }
    b2Vec2 getPosition () const { return m_body->GetPosition (); }
    float getAngle () const { return m_body->GetAngle (); }
};
