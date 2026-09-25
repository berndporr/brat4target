#include "brat_math.h"
#include "box2d_helpers.h"
#include "const.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

void math::MulT (const b2Transform &deltaPose, b2Transform &pose)
{
    pose = b2MulT (deltaPose, pose);
}

void math::InvMul (const b2Transform &deltaPose, b2Transform &pose)
{
    pose = b2help::InvMul (deltaPose, pose);
}

cv::Mat math::cv_affine_matrix33 (const b2Transform &t)
{
    cv::Point2f p (t.p.x, t.p.y);
    double angle = double (t.q.GetAngle ()) * double (1 / DEG_TO_RAD_K),
           scale = 1.0;
    cv::Mat result = cv::getRotationMatrix2D (p, angle, scale);
    return result;
}

b2Transform math::solveAxB (const b2Transform &x, const b2Transform &B)
{ //
    cv::Point2f p (x.p.x, x.p.y);
    cv::Mat x_matrix (3, 3, CV_32F);
    x_matrix.at<float> (0, 0) = x.q.c;
    x_matrix.at<float> (1, 1) = x.q.c;
    x_matrix.at<float> (0, 1) = x.q.s;
    x_matrix.at<float> (1, 0) = -x.q.s;
    x_matrix.at<float> (0, 2) = x.p.x;
    x_matrix.at<float> (1, 2) = x.p.y;
    x_matrix.at<float> (2, 0) = 0;
    x_matrix.at<float> (2, 1) = 0;
    x_matrix.at<float> (2, 2) = 1;
    cv::Mat x_inv_matrix = x_matrix.inv ();
    //cv::invertAffineTransform(x_matrix, x_inv_matrix);
    b2Transform x_inv = math::transform_2d (x_inv_matrix);

    return b2Mul (B, x_inv);
}

void calc_transform (b2Transform &result, b2Transform t_new,
                     b2Transform t_prev)
{
    float dot = b2Dot (t_new.p, t_prev.p);
    float denom = (t_new.p.Length () * t_prev.p.Length ());
    float cos_angle = dot / denom;
    float angle = 0;
    if (fabs (cos_angle) <= 1)
    {
        angle = acosf (cos_angle); //[0, pi]
        b2Transform pov_prev
            = b2MulT (t_prev, t_new); //position of new d from prev perspective
        if (pov_prev.p.y < 0)
        {
            angle = -angle;
        }
    }
    float distance = t_new.p.Length () - t_prev.p.Length ();
    result.q.Set (angle);
    result.p.x = result.q.c * distance;
    result.p.y = result.q.s * distance;
}
