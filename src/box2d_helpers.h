#ifndef BOX2D_HELPERS_H
#define BOX2D_HELPERS_H
#include <box2d/b2_math.h>
#include <vector>

/**
 * @brief Searches vector or a certain item
 * 
 * @param vector 
 * @param item 
 * @return auto iterator to element
 */
template <class I>
auto check_vector_for (std::vector<I> &vector, const I &item)
{
    for (int i = 0; i < vector.size (); i++)
    {
        if (vector[i] == item)
        {
            return vector.begin () + i;
        }
    }
    return vector.end ();
}

template <class P, class I>
auto check_vector_for (std::vector<I> &vector, const P &predicate)
{
    for (int i = 0; i < vector.size (); i++)
    {
        if (predicate (vector[i]))
        {
            return vector.begin () + i;
        }
    }
    return vector.end ();
}

/**
 * @brief Finds item in vector and erases it
 */
template <typename I> void erase_from_vector (std::vector<I> &vec, const I &i)
{
    auto it = check_vector_for (vec, i);
    if (it != vec.end ())
    {
        vec.erase (it);
    }
}

namespace b2help
{

b2Transform InvMul (const b2Transform &t1, const b2Transform &t2);

};

float angle_subtract (float a1, float a2);

typedef b2Transform Transform;
bool operator!= (Transform const &, Transform const &);
bool operator== (Transform const &, Transform const &);
void operator-= (Transform &, Transform const &);
void operator+= (Transform &, Transform const &);
Transform operator+ (Transform const &, Transform const &);
Transform operator- (Transform const &, Transform const &);
Transform operator- (Transform const &);
Transform operator+ (Transform const &, b2Vec2 const &);

#endif