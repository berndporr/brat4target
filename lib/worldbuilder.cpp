#include "worldbuilder.h"
#include "debug.h"
#include <algorithm>
#include <box2d/box2d.h>
#include <fstream> // std::ofstream
#include <iostream>
#include <memory>

#include <box2d/box2d.h>
#include <opencv2/opencv.hpp>
#include <vector>

WorldBuilder::SpeedResult
WorldBuilder::calculateSpeed (std::shared_ptr<b2World> worldA,
                              std::shared_ptr<b2World> worldB, float dt)
{
    std::vector<cv::Point2f> pointsA;
    std::vector<cv::Point2f> pointsB;

    // 1. Extract coordinates from Box2D to OpenCV point formats
    for (b2Body *b = worldA->GetBodyList (); b; b = b->GetNext ())
    {
        if (b->GetFixtureList ())
        {
            pointsA.push_back (
                cv::Point2f (b->GetPosition ().x, b->GetPosition ().y));
        }
    }
    for (b2Body *b = worldB->GetBodyList (); b; b = b->GetNext ())
    {
        if (b->GetFixtureList ())
        {
            pointsB.push_back (
                cv::Point2f (b->GetPosition ().x, b->GetPosition ().y));
        }
    }

    SpeedResult result;

    // We need at least 2 points to compute translation + rotation, but more is better for noise
    if (pointsA.size () < 3 || pointsB.size () < 3)
        return result;

    // Fast-track optimization: Ensure the point sets match in size for the estimator.
    // In real LIDAR data, if sizes differ due to occlusions, truncate or pad,
    // or use a Nearest Neighbors matching pass first.
    size_t minSize = std::min (pointsA.size (), pointsB.size ());
    pointsA.resize (minSize);
    pointsB.resize (minSize);

    // 2. Compute the Rigid 2D Transformation (Translation + Rotation) using native RANSAC
    std::vector<uchar> inliers;
    cv::Mat affineMatrix = cv::estimateAffinePartial2D (
        pointsA, // Source points
        pointsB, // Target points
        inliers, // Output vector indicating which points were considered "good data"
        cv::RANSAC, // Robust estimation method
        0.3,  // RANSAC inlier threshold (maximum distance allowance for noise)
        2000, // Maximum iterations
        0.99  // Confidence level
    );

    // If a valid matrix couldn't be calculated (e.g. noise completely broke consensus)
    if (affineMatrix.empty ())
        return result;

    // 3. Extract Translation and Rotation from the 2x3 Affine Matrix
    // The matrix structure is:
    // [  cos(theta)   -sin(theta)   tx ]
    // [  sin(theta)    cos(theta)   ty ]
    double cosTheta = affineMatrix.at<double> (0, 0);
    double sinTheta = affineMatrix.at<double> (1, 0);

    result.angSpeed = std::atan2 (sinTheta, cosTheta) / dt;
    result.linSpeed.x = linearSpeedXfilter.process (
        (float)affineMatrix.at<double> (0, 2) / dt);
    result.linSpeed.y = linearSpeedYfilter.process (
        (float)affineMatrix.at<double> (1, 2) / dt);
    if (DEBUG)
    {
        fprintf (stderr, "Lin speed = %f,%f\n", result.linSpeed.x, result.linSpeed.y);
    }
    return result;
}

float getBodyBoundingBoxArea (b2Body *body)
{
    b2AABB totalAABB;

    // Initialize with extreme inverted values so the first fixture expands it
    totalAABB.lowerBound.Set (b2_maxFloat, b2_maxFloat);
    totalAABB.upperBound.Set (-b2_maxFloat, -b2_maxFloat);

    bool hasFixtures = false;

    // Loop through all fixtures attached to this body
    for (b2Fixture *f = body->GetFixtureList (); f; f = f->GetNext ())
    {
        hasFixtures = true;
        b2AABB fixtureAABB;

        // Get the AABB for child index 0 (standard shapes only have 1 child)
        f->GetShape ()->ComputeAABB (&fixtureAABB, body->GetTransform (), 0);

        // Combine into the total bounding box
        totalAABB.Combine (fixtureAABB);
    }

    // Fallback if the body has no fixtures (uses the body center position)
    if (!hasFixtures)
    {
        b2Vec2 pos = body->GetPosition ();
        return 0;
    }

    const float area
        = fabs ((totalAABB.upperBound.x - totalAABB.lowerBound.x)
                * (totalAABB.upperBound.y - totalAABB.lowerBound.y));

    return area;
}

Bundle Bundle::operator* (const Bundle &b) const
{
    Bundle result = *this;
    result.x *= b.x;
    result.y *= b.y;
    result.angle *= b.angle;
    result.width *= b.width;
    result.length *= b.length;
    return result;
}

Bundle Bundle::operator* (float f) const
{
    Bundle result = *this;
    result.x *= f;
    result.y *= f;
    result.angle *= f;
    result.width *= f;
    result.length *= f;
    return result;
}

bool Bundle::operator< (const Bundle &bf)
{
    return bf.radius () < radius () && bf.get_angle () < angle
           && bf.get_width () < width && bf.get_length () < length;
}

Bundle Bundle::operator+ (const Bundle &b) const
{
    Bundle result = *this;
    result.x += b.get_x ();
    result.y += b.get_y ();
    result.angle += b.get_angle ();
    result.width += b.get_width ();
    result.length += b.get_length ();
    return result;
}

Bundle Bundle::operator- (const Bundle &b) const
{
    Bundle result = *this;
    result.x -= b.get_x ();
    result.y -= b.get_y ();
    result.angle -= b.get_angle ();
    result.width -= b.get_width ();
    result.length -= b.get_length ();
    return result;
}

bool BodyFeatures::match (const BodyFeatures &bf, Bundle *bundle,
                          b2Transform t) const
{
    float hypothenuse_square
        = pow (bf.pose.p.Length (), 2); //assumes robot-centric perspective
    float adj_side_square = pow (bf.pose.p.Length () * t.q.c, 2);
    float distance_adjust = sqrt (hypothenuse_square - adj_side_square);
    float diff_x = pose.p.x - bf.pose.p.x; //-t.q.s*distance_adjust
    float diff_y = pose.p.y - bf.pose.p.y; //+t.q.c*distance_adjust
    //InvMul float diff_transform=bf.pose.p.Length()- pose.p.Length();
    float diff_w = halfWidth - bf.halfWidth;
    float diff_l = halfLength - bf.halfLength;
    bool match_x
        = fabs (diff_x) < D_POSE_MARGIN + fabs (t.q.s * distance_adjust);
    bool match_y
        = fabs (diff_y) < D_POSE_MARGIN + fabs (t.q.c * distance_adjust);
    bool match_distance
        = pose.p.Length () - bf.pose.p.Length () < D_POSE_MARGIN;
    bool match_w = fabs (diff_w) < D_DIMENSIONS_MARGIN;
    bool match_h = fabs (diff_l) < D_DIMENSIONS_MARGIN;
    if (bundle != NULL)
    {
        *bundle = Bundle (diff_x, diff_y, 0, diff_w, diff_l);
    }
    //return match_x && match_y && match_w && match_h;
    return match_w && match_h && match_distance;
}

std::vector<b2Vec2> BodyFeatures::vertices () const
{
    std::vector<b2Vec2> result;
    result.push_back (b2Vec2 (halfWidth, halfLength));
    result.push_back (b2Vec2 (halfWidth, -halfLength));
    result.push_back (b2Vec2 (-halfWidth, halfLength));
    result.push_back (b2Vec2 (-halfWidth, -halfLength)); //make upright box
    for (b2Vec2 &v : result)
    {
        v = b2Mul (pose, v);
    }
    return result;
}

std::vector<cv::Point2f> BodyFeatures::vertices_cv () const
{
    std::vector<b2Vec2> vb2d = vertices ();
    std::vector<cv::Point2f> result;
    for (const b2Vec2 &v : vb2d)
    {
        result.push_back (cv::Point2f (v.x, v.y));
    }
    return result;
}

bool Pointf::isin (Pointf tl, Pointf br)
{
    bool result
        = this->x > tl.x & this->x<br.x &this->y> br.y & this->y < tl.y;
    return result;
}

float length (cv::Point2f const &p)
{
    return sqrt (pow (p.x, 2) + pow (p.y, 2));
}

float angle (cv::Point2f const &p) { return atan2 (p.y, p.x); }

bool operator< (Pointf const &p1, Pointf const &p2)
{
    float a1 = angle (p1);
    float l1 = length (p1);
    float a2 = angle (p2);
    float l2 = length (p2);
    return std::tie (a1, l1) < std::tie (a2, l2);
}

bool operator> (const Pointf &p1, const Pointf &p2) { return p2 < p1; }

b2Vec2 getb2Vec2 (cv::Point2f p) { return b2Vec2 (p.x, p.y); }

Pointf Polar2f (float radius, float angle)
{
    float x = radius * cos (angle);
    float y = radius * sin (angle);
    return Pointf (x, y);
}

std::pair<bool, BodyFeatures>
bounding_rotated_box (std::vector<cv::Point2f> nb)
{
    for (cv::Point2f &p : nb)
    {
        p.x = round (p.x * 100) / 100;
        p.y = round (p.y * 100) / 100;
    }
    std::pair<bool, BodyFeatures> result (0, BodyFeatures ());
    if (nb.empty ())
    {
        return result;
    }
    cv::RotatedRect rotated_rect = cv::minAreaRect (nb);
    if (rotated_rect.size.width > rotated_rect.size.height)
    {
        result.second.setHalfLength (
            rotated_rect.size.width
            / 2); //NVM THIS ///THEY ARE SWAPPED IN OPENCV DO NOT TOUCH
        result.second.setHalfWidth (rotated_rect.size.height / 2);
        if (rotated_rect.angle > 90)
        {
            rotated_rect.angle -= 90;
        }
        else
        {
            rotated_rect.angle += 90;
        }
    }
    else
    {
        result.second.setHalfLength (
            rotated_rect.size.height
            / 2); //NVM THIS ///THEY ARE SWAPPED IN OPENCV DO NOT TOUCH
        result.second.setHalfWidth (rotated_rect.size.width / 2);
    }
    result.second.pose.p
        = b2Vec2 (rotated_rect.center.x, rotated_rect.center.y);
    float angle_rad = rotated_rect.angle * DEG_TO_RAD_K;
    result.second.pose.q.Set (angle_rad);
    result.second.pose.q.Set (
        atan (result.second.pose.q.s / result.second.pose.q.c));
    result.first = true;
    return result;
}

void WorldBuilder::object_dump (const char *filename)
{
    FILE *f = fopen (filename, "wt");
    for (auto &bf : world_objects)
    {
        for (auto v : bf.vertices ())
        {
            fprintf (f, "%.3f\t%.3f\n", v.x, v.y);
        }
    }
    fclose (f);
}

void WorldBuilder::bodies_dump (const char *filename)
{
    FILE *file = fopen (filename, "wt");
    for (b2Body *b = world->GetBodyList (); b != NULL; b = b->GetNext ())
    {
        const float a = getBodyBoundingBoxArea (b);
        fprintf (file, "%f\t%f\t%f\n", b->GetPosition ().x,
                 b->GetPosition ().y, a);
    }
    fclose (file);
}

void WorldBuilder::exportWorldToSVG (const char *filename, float scale)
{
    std::ofstream svg (filename, std::ios::out | std::ios::trunc);
    if (!svg.is_open ())
    {
        std::cerr << "Failed to open file for writing SVG." << std::endl;
        return;
    }

    // Define canvas boundaries (Adjust these dimensions as needed)
    int width = 800;
    int height = 600;
    float offsetX = width / 2.0f; // Centers the Box2D (0,0) origin
    float offsetY = height / 2.0f;

    // Write SVG Header
    svg << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    svg << "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\" "
           "\"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n";
    svg << "<svg xmlns=\"http://www.w3.org/2000/svg\" "
           "xmlns:xlink=\"http://www.w3.org/1999/xlink\" viewBox=\"0 0 "
        << width << " " << height << "\" width=\"" << width << "\" height=\""
        << height << "\">\n";
    // Background
    svg << "  <rect width=\"100%\" height=\"100%\" fill=\"#1e1e24\" />\n";

    // Lambda helper to map Box2D meters to SVG pixels
    auto mapX = [scale, offsetX] (float x) { return (x * scale) + offsetX; };
    auto mapY = [scale, offsetY] (float y) {
        return (-y * scale) + offsetY;
    }; // Flips Y-axis

    // Loop through all bodies in the world
    for (b2Body *body = world->GetBodyList (); body; body = body->GetNext ())
    {
        b2Transform xf = body->GetTransform ();

        // Loop through all fixtures on the current body
        for (b2Fixture *fixture = body->GetFixtureList (); fixture;
             fixture = fixture->GetNext ())
        {
            b2Shape *shape = fixture->GetShape ();

            if (shape->GetType () == b2Shape::e_polygon)
            {
                b2PolygonShape *poly = (b2PolygonShape *)shape;
                svg << "  <polygon points=\"";

                for (int32 i = 0; i < poly->m_count; ++i)
                {
                    // Multiply local vertex by body transform to get world coordinates
                    b2Vec2 worldVert = b2Mul (xf, poly->m_vertices[i]);
                    svg << mapX (worldVert.x) << "," << mapY (worldVert.y);
                    if (i < poly->m_count - 1)
                        svg << " ";
                }

                svg << "\" fill=\"#4a90e2\" stroke=\"#ffffff\" "
                       "stroke-width=\"1.5\" />\n";
            }
            else if (shape->GetType () == b2Shape::e_circle)
            {
                b2CircleShape *circle = (b2CircleShape *)shape;
                // Transform center of the circle to world space
                b2Vec2 worldCenter = b2Mul (xf, circle->m_p);
                float radiusInPixels = circle->m_radius * scale;

                svg << "  <circle cx=\"" << mapX (worldCenter.x) << "\" cy=\""
                    << mapY (worldCenter.y) << "\" r=\"" << radiusInPixels
                    << "\" fill=\"#e056fd\" stroke=\"#ffffff\" "
                       "stroke-width=\"1.5\" />\n";
            }
            // Optional: Handle b2EdgeShape or b2ChainShape here if used in your world
        }
    }

    // Close SVG Tag
    svg << "</svg>\n";
    svg.close ();
}

std::pair<Pointf, Pointf> WorldBuilder::bounds (b2Transform start,
                                                float boxLength,
                                                float halfWindowWidth,
                                                std::vector<Pointf> *_bounds)
{
    std::pair<Pointf, Pointf> result;
    std::vector<Pointf> bds;
    Pointf positionVector, radiusVector, maxFromStart, top, bottom;
    radiusVector = Polar2f (boxLength, start.q.GetAngle ());
    maxFromStart = Pointf (start.p.x, start.p.y) + radiusVector;
    //FIND THE BOUNDS OF THE BOX
    b2Vec2 unitPerpR (-sin (start.q.GetAngle ()), cos (start.q.GetAngle ()));
    b2Vec2 unitPerpL (sin (start.q.GetAngle ()), -cos (start.q.GetAngle ()));
    bds.push_back (Pointf (start.p.x, start.p.y)
                   + Pointf (unitPerpL.x * halfWindowWidth,
                             unitPerpL.y * halfWindowWidth));
    bds.push_back (Pointf (start.p.x, start.p.y)
                   + Pointf (unitPerpR.x * halfWindowWidth,
                             unitPerpR.y * halfWindowWidth));
    bds.push_back (maxFromStart
                   + Pointf (unitPerpL.x * halfWindowWidth,
                             unitPerpL.y * halfWindowWidth));
    bds.push_back (maxFromStart
                   + Pointf (unitPerpR.x * halfWindowWidth,
                             unitPerpR.y * halfWindowWidth));
    CompareY compareY;
    std::sort (bds.begin (), bds.end (), compareY); //sort bottom to top
    result.first = bds[0];                          //bottom
    result.second = bds[3];                         //top
    if (_bounds != NULL)
    { //for debug
        *_bounds = bds;
    }
    return result;
}

b2PolygonShape WorldBuilder::object_filtering_box (float halfWindowWidth,
                                                   float boxLength,
                                                   b2Transform start)
{
    b2PolygonShape box;
    b2Vec2 centroid (0, 0);
    centroid.x = boxLength / 2;
    box.SetAsBox (
        boxLength / 2, halfWindowWidth, centroid,
        0); //this  allows to filter out objects irrelevant to this task (i.e. collision unlikely assumint they're static)
    return box;
}

std::vector<std::vector<cv::Point2f> >
WorldBuilder::kmeans_clusters (std::vector<cv::Point2f> points,
                               std::vector<cv::Point2f> &centers)
{

    const int MAX_CLUSTERS = 3, attempts = 3, flags = cv::KMEANS_PP_CENTERS;
    cv::Mat bestLabels;
    cv::TermCriteria termCriteria (
        cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 10, 1.0);
    cv::kmeans (points, MAX_CLUSTERS, bestLabels, termCriteria, attempts,
                flags, centers);
    std::vector<std::vector<cv::Point2f> > result (centers.size ());

    for (int i = 0; i < bestLabels.rows; i++)
    { //bestlabel[i] gives the index
        auto index = bestLabels.at<int> (i, 0);
        result[index].push_back (points[i]);
    }
    return result;
}

std::vector<std::vector<cv::Point2f> >
WorldBuilder::partition_clusters (std::vector<cv::Point2f> points)
{
    const float maxDistanceThreshold = 0.05;
    const float distThresSq = maxDistanceThreshold * maxDistanceThreshold;
    const float maxAngleDegrees = 30;
    const float cornerThreshold
        = std::cos (maxAngleDegrees * 3.14159265f / 180.0f);

    auto dist = [&] (const cv::Point2f &a, const cv::Point2f &b) {
        // 1. Distance Constraint (Standard Proximity check)
        float dx = b.x - a.x;
        float dy = b.y - a.y;
        float distSq = dx * dx + dy * dy;
        if (distSq > distThresSq)
            return false; // Too far apart -> Split

        // 2. Directional / Angle Constraint
        // Find the index context of point 'a' relative to the main vector
        // to determine the direction the LiDAR beam was travelling.
        auto itA = std::find (points.begin (), points.end (), a);
        size_t idxA = std::distance (points.begin (), itA);

        if (idxA > 0 && idxA < points.size () - 1)
        {
            // Vector from previous point to current point (Incoming trajectory)
            cv::Point2f vecIncoming = points[idxA] - points[idxA - 1];
            // Vector from current point to next point (Outgoing trajectory)
            cv::Point2f vecOutgoing = b - points[idxA];

            float lenIncoming = std::sqrt (vecIncoming.x * vecIncoming.x
                                           + vecIncoming.y * vecIncoming.y);
            float lenOutgoing = std::sqrt (vecOutgoing.x * vecOutgoing.x
                                           + vecOutgoing.y * vecOutgoing.y);

            if (lenIncoming > 0.001f && lenOutgoing > 0.001f)
            {
                // Calculate Dot Product to find the cosine of the angle between vectors
                float dot = (vecIncoming.x * vecOutgoing.x
                             + vecIncoming.y * vecOutgoing.y);
                float cosAngle = dot / (lenIncoming * lenOutgoing);
                if (cosAngle < cornerThreshold)
                {
                    return false; // Sharp corner detected -> Force Split!
                }
            }
        }
        return true; // Points are close and continue in a relatively straight line
    };
    std::vector<int> labels;
    cv::partition (points, labels, dist);
    int n_clusters = *(std::max_element (labels.begin (), labels.end ())) + 1;
    std::vector<std::vector<cv::Point2f> > result (n_clusters);
    for (int i = 0; i < points.size (); i++)
    { //bestlabel[i] gives the index
        int label = labels[i];
        result[label].push_back (points[i]);
    }
    return result;
}

std::vector<BodyFeatures>
WorldBuilder::processData (const CoordinateContainer &points,
                           const b2Transform &start)
{
    std::vector<BodyFeatures> result;
    std::vector<Pointf> ptset = points;
    std::pair<bool, BodyFeatures> feature = bounding_box (ptset);
    if (feature.first)
    {
        feature.second.pose.q.Set (start.q.GetAngle ());
        result.push_back (feature.second);
    }
    return result;
}

std::vector<BodyFeatures>
WorldBuilder::cluster_data (const CoordinateContainer &pts,
                            const b2Transform &start, CLUSTERING clustering)
{
    std::vector<BodyFeatures> result;
    std::vector<cv::Point2f> points, centers;
    for (const Pointf &p : pts)
    {
        points.push_back (cv::Point2f (float (p.x), float (p.y)));
    }
    std::vector<std::vector<cv::Point2f> > clusters;
    if (clustering == KMEANS)
    {
        clusters = kmeans_clusters (points, centers);
    }
    else if (clustering == PARTITION)
    {
        clusters = partition_clusters (points);
    }
    for (int c = 0; c < clusters.size (); c++)
    {
        if (std::pair<bool, BodyFeatures> feature
            = bounding_rotated_box (clusters[c]);
            feature.first)
        {
            result.push_back (feature.second);
        }
    }
    return result;
}

b2Body *WorldBuilder::makeBody (const BodyFeatures &features)
{
    b2Body *body;
    b2BodyDef bodyDef;
    b2FixtureDef fixtureDef;
    bodyDef.type = features.bodyType;
    bodyDef.position.Set (features.pose.p.x, features.pose.p.y);
    bodyDef.angle = features.pose.q.GetAngle ();
    body = world->CreateBody (&bodyDef);
    body->GetUserData ().pointer = 0;
    switch (features.shape)
    {
    case b2Shape::e_polygon:
    {
        b2PolygonShape fixture;
        fixtureDef.shape = &fixture;
        fixture.SetAsBox (features.halfWidth, features.halfLength);
        body->CreateFixture (fixtureDef.shape, features.shift);
        break;
    }
    default:
        throw std::invalid_argument ("not a valid shape\n");
        break;
    }
    return body;
}

std::pair<CoordinateContainer, bool>
WorldBuilder::salientPoints (b2Transform start,
                             const CoordinateContainer &current,
                             std::pair<Pointf, Pointf> bt)
{
    std::pair<CoordinateContainer, bool> result (CoordinateContainer (), 0);
    float qBottomH = 0, qTopH = 0, qBottomP = 0, qTopP = 0, mHead = 0,
          mPerp = 0, ceilingY = 0, floorY = 0, frontX = 0, backX = 0;
    if (sin (start.q.GetAngle ()) != 0 && cos (start.q.GetAngle ()) != 0)
    {
        //FIND PARAMETERS OF THE LINES CONNECTING THE a
        mHead = sin (start.q.GetAngle ())
                / cos (start.q.GetAngle ()); //slope of heading direction
        mPerp = -1 / mHead;
        qBottomH = bt.first.y - mHead * bt.first.x;
        qTopH = bt.second.y - mHead * bt.second.x;
        qBottomP = bt.first.y - mPerp * bt.first.x;
        qTopP = bt.second.y - mPerp * bt.second.x;
        for (Pointf p : current)
        {
            ceilingY = mHead * p.x + qTopH;
            floorY = mHead * p.x + qBottomH;
            float frontY = mPerp * p.x + qBottomP;
            float backY = mPerp * p.x + qTopP;
            if (p.y >= floorY && p.y <= ceilingY && p.y >= frontY
                && p.y <= backY)
            {
                result.first.push_back (p);
            }
        }
    }
    else
    {
        ceilingY = std::max (bt.second.y, bt.first.y);
        floorY = std::min (bt.second.y, bt.first.y);
        frontX = std::min (bt.second.x, bt.first.x);
        backX = std::max (bt.second.x, bt.first.x);
        for (Pointf p : current)
        {
            if (p.y >= floorY && p.y <= ceilingY && p.x >= frontX
                && p.x <= backX)
            {
                result.first.push_back (p);
            }
        }
    }
    return result;
}

std::vector<BodyFeatures>
WorldBuilder::getFeatures (const CoordinateContainer &current,
                           b2Transform start, CLUSTERING clustering)
{
    std::vector<BodyFeatures> features;
    for (const auto &p : current)
    {
        features.push_back (
            BodyFeatures (b2Transform (b2Vec2 (p.x, p.y), b2Rot (0))));
    }
    return features;
}

std::shared_ptr<b2World> WorldBuilder::buildWorld (CoordinateContainer &coords,
                                                   b2Transform start,
                                                   float halfWindowWidth,
                                                   CLUSTERING clustering)
{
    world = std::make_shared<b2World> (GRAVITY);
    world_objects
        = getFeatures (coords, b2Transform_zero, WorldBuilder::PARTITION);
    for (const BodyFeatures &bf : world_objects)
    {
        makeBody (bf);
    }
    return world;
}

b2Body *WorldBuilder::get_robot ()
{
    for (b2Body *b = world->GetBodyList (); b; b = b->GetNext ())
    {
        if (b->GetUserData ().pointer == ROBOT_FLAG)
        {
            return b;
        }
    }
    return NULL;
}

b2Fixture *WorldBuilder::get_chassis (b2Body *r)
{
    if (r->GetUserData ().pointer != ROBOT_FLAG)
    {
        return NULL;
    }
    for (b2Fixture *f = r->GetFixtureList (); f; f = f->GetNext ())
    {
        if (!f->IsSensor ())
        {
            return f;
        }
    }
    return NULL;
}

std::vector<BodyFeatures>
WorldClusterBuilder::getFeatures (const CoordinateContainer &current,
                                  b2Transform start, CLUSTERING clustering)
{
    std::vector<BodyFeatures> features;
    // std::pair<Pointf, Pointf> bt = bounds(d, start, boxLength, halfWindowWidth);
    //std::pair <CoordinateContainer, bool> salient = salientPoints(start,current, bt);
    if (current.empty ())
    {
        return features;
    }
    if (clustering == BOX)
    {
        features = processData (current, start);
    }
    else
    {
        features = cluster_data (current, start, clustering);
    }
    return features;
}

std::shared_ptr<b2World>
FocusedBuilder::buildWorld (CoordinateContainer &coords, b2Transform start,
                            float halfWindowWidth, CLUSTERING clustering)
{
    world = std::make_shared<b2World> (GRAVITY);
    world_objects
        = getFeatures (coords, b2Transform_zero, WorldBuilder::PARTITION);
    float boxLength = simulationStep - ROBOT_BOX_OFFSET_X;
    std::vector<cv::Point2f> points_to_track;
    b2PolygonShape filter_box
        = object_filtering_box (halfWindowWidth, boxLength, start);
    for (BodyFeatures f : world_objects)
    {
        b2PolygonShape feature_shape;
        feature_shape.SetAsBox (f.halfWidth, f.halfLength);
        if (b2TestOverlap (&filter_box, 0, &feature_shape, 0, start, f.pose))
        {
            makeBody (f);
        }
    }
    int _count = world->GetBodyCount ();
    return world;
}
