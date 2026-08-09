#include "box2d_helpers.h"

float angle_subtract (float a1, float a2)
{
    float result = 0;
    if (fabs (a1) > 3 * M_PI_4 || fabs (a2) > 3 * M_PI_4)
    {
        if (a1 < 0 & a2 > 0)
        {
            a2 -= 2 * M_PI;
        }
        else if (a2 < 0 & a1 > 0)
        {
            a2 += 2 * M_PI;
        }
    }
    result = a1 - a2;
    return result;
}

bool operator!= (Transform const &t1, Transform const &t2)
{
    return t1.p.x != t2.p.x || t1.p.y != t2.p.y
           || t1.q.GetAngle () != t2.q.GetAngle ();
}

bool operator== (Transform const &t1, Transform const &t2)
{
    return (t1.p.x == t2.p.x) && (t1.p.y == t2.p.y)
           && (t1.q.GetAngle () == t2.q.GetAngle ());
}

b2Transform b2help::InvMul (const b2Transform &t1, const b2Transform &t2)
{
    b2Transform result;
    b2Rot rot (-t1.q.GetAngle ());
    result.q = b2Mul (rot, t2.q);
    result.p = b2Mul (rot, t2.p - t1.p);
    return result;
}

void operator-= (Transform &t1, Transform const &t2)
{
    t1.p.x -= t2.p.x;
    t1.p.y -= t2.p.y;
    t1.q.Set (angle_subtract (t1.q.GetAngle (), t2.q.GetAngle ()));
}

void operator+= (Transform &t1, Transform const &t2)
{
    t1.p.x += t2.p.x;
    t1.p.y += t2.p.y;
    t1.q.Set (t1.q.GetAngle () + t2.q.GetAngle ());
}

Transform operator+ (Transform const &t1, Transform const &t2)
{
    b2Transform result;
    result.p.x = t1.p.x + t2.p.x;
    result.p.y = t1.p.y + t2.p.y;
    result.q.Set (t1.q.GetAngle () + t2.q.GetAngle ());
    return result;
}

Transform operator+ (Transform const &t1, b2Vec2 const &v2)
{
    b2Transform result;
    result.p.x = t1.p.x + v2.x;
    result.p.y = t1.p.y + v2.y;
    result.q.Set (t1.q.GetAngle ());
    return result;
}

Transform operator- (Transform const &t1, Transform const &t2)
{
    b2Transform result;
    result.p.x = t1.p.x - t2.p.x;
    result.p.y = t1.p.y - t2.p.y;
    result.q.Set (angle_subtract (t1.q.GetAngle (), t2.q.GetAngle ()));
    return result;
}

Transform operator- (Transform const &t)
{
    b2Transform result;
    result.p.x = -(t.p.x);
    result.p.y = -(t.p.y);
    result.q.Set (-t.q.GetAngle ());
    return result;
}
