#pragma once
#include "const.h"
#include <algorithm>
#include <atomic>
#include <box2d/b2_math.h>
#include <box2d/box2d.h>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp> //useful down the line! (graphTools)
#include <opencv2/tracking.hpp>
#include <opencv2/video/tracking.hpp>
#include <thread>

//
// Anything here is about creating an effective representation of the world
// for box2d so that the mental simulations can run with minimal effort.
//

/**
 * @brief Wrapper around cv::Point2f for customisation purposes
 */
struct Pointf : public cv::Point2f
{

    Pointf () = default;

    Pointf (float _x, float _y)
    {
        x = _x;
        y = _y;
    }

    Pointf operator+ (const Pointf p)
    {
        Pointf result;
        result.x = x + p.x;
        result.y = y + p.y;
        return result;
    }

    Pointf operator- (const Pointf p)
    {
        Pointf result;
        result.x = x - p.x;
        result.y = y - p.y;
        return result;
    }

    bool isin (Pointf, Pointf);
};

/**
 * @brief container for LIDAR coordinates
 */
typedef std::vector<Pointf> CoordinateContainer;

/**
 * Gets the area of the bounding box of a body. That's useful to determine how
 * big an obstacle is for Braitenberg avoidance.
 */
float getBodyBoundingBoxArea(b2Body* body);

struct CompareY
{
    template <typename T> bool operator() (T a, T b)
    { //
        return a.y <= b.y;
    }
};

struct CompareX
{
    template <typename T> bool operator() (T a, T b) { return a.x <= b.x; }
};

template <typename C> std::vector<C> arrayToVec (C *c, int ct)
{
    std::vector<C> result;
    for (int i = 0; i < ct; i++)
    {
        result.push_back (*c);
        c++;
    }
    return result;
}

/**
*Struct for conveniently grouping weights/threshold associated to BodyFeatures
**/
class Bundle
{
    float x = 1;
    float y = 1;
    float angle = 1;
    float width = 1;
    float length = 1;

  public:
    Bundle () = default;

    Bundle (float _x, float _y, float _a, float _w, float _l)
        : x (_x), y (_y), angle (_a), width (_w), length (_l)
    {
    }

    /**
      * @brief Dot product between two bundles
      * 
      * @param b the other bundle
      */
    Bundle operator* (const Bundle &b) const;

    /**
     * @brief Multiply bundle for a scalar      
    */
    Bundle operator* (float) const;

    bool operator< (const Bundle &bf);

    Bundle operator+ (const Bundle &b) const;

    Bundle operator- (const Bundle &b) const;

    float radius () const { return sqrt (pow (x, 2) + pow (y, 2)); }

    float get_x () const { return x; }

    float get_y () const { return y; }

    float get_angle () const { return angle; }

    float get_width () const { return width; }

    float get_length () const { return length; }

    float sum_squares () const
    {
        return pow (x, 2) + pow (y, 2) + pow (angle, 2) + pow (width, 2)
               + pow (length, 2);
    }

    std::vector<float> get_vector () const
    {
        return { x, y, angle, width, length };
    }
};

struct BodyFeatures
{
    BodyFeatures () = default;
    b2Transform pose{ b2Transform (b2Vec2 (0, 0), b2Rot (0)) };
    float halfLength = MIN_BODY_DIMENSION; //x
    float halfWidth = MIN_BODY_DIMENSION;  //y
    float shift = 0.0f;
    b2BodyType bodyType = b2_dynamicBody;
    b2Shape::Type shape = b2Shape::e_polygon;

    BodyFeatures (b2Transform _pose) : pose (_pose) {}

    void setHalfLength (const float &f)
    {
        if (f < MIN_BODY_DIMENSION)
        {
            halfLength = MIN_BODY_DIMENSION;
        }
        else
        {
            halfLength = f;
        }
    }

    void setHalfWidth (const float &f)
    {
        if (f < MIN_BODY_DIMENSION)
        {
            halfWidth = MIN_BODY_DIMENSION;
        }
        else
        {
            halfWidth = f;
        }
    }

    float width () const { return halfWidth * 2; }

    float length () const { return halfLength * 2; }

    float area () const { return width () * length (); }

    std::vector<b2Vec2> vertices () const;

    std::vector<cv::Point2f> vertices_cv () const; //global vertices

    bool is_point () const
    {
        return halfWidth == MIN_BODY_DIMENSION
               && halfLength == MIN_BODY_DIMENSION;
    }

    bool match (const BodyFeatures &bf, Bundle *bundle, b2Transform t) const;
};

template <> struct cv::traits::Depth<Pointf>
{
    enum
    {
        value = Depth<cv::Point2f>::value
    };
};

template <> struct cv::traits::Type<Pointf>
{
    enum
    {
        value = CV_MAKETYPE (cv::traits::Depth<Pointf>::value, 2)
    };
};

float length (cv::Point2f const &p);

float angle (cv::Point2f const &);

bool operator< (Pointf const &, Pointf const &);

bool operator> (const Pointf &, const Pointf &);

/**
 * @brief Gets an opencv point in b2VEc2 format
 * 
 * @return b2Vec2 
 */
b2Vec2 getb2Vec2 (cv::Point2f);

/**
 * @brief Get the Pointf object from a 2d point/vector
 * 
 * @tparam T 
 * @param v 
 * @return Pointf 
 */
template <typename T> Pointf getPointf (T v) { return Pointf (v.x, v.y); }

/**
 * @brief Gets Pointf from polar coordinates
 * 
 * @param radius 
 * @param angle 
 * @return Pointf 
 */
Pointf Polar2f (float radius, float angle);

/**
 * @brief Casts a vector of 2d points to a vector of box2d b2Vec2
 * 
 * @tparam T 2d point/2d vector
 * @param v 2d point or vector
 * @return std::vector<b2Vec2> 
 */
template <typename T>
inline std::vector<b2Vec2> cast_b2Vec2 (const std::vector<T> &v)
{
    std::vector<b2Vec2> result;
    for (const T &t : v)
    {
        result.push_back (b2Vec2 (t.x, t.y));
    }
    return result;
}

/**
 * @brief Casts a vector of 2d points to a vector of cv::Point2f
 * 
 * @tparam T 2d point/2d vector
 * @param v 2d point or vector
 * @return std::vector<b2Vec2> 
 */
template <typename T>
inline std::vector<cv::Point2f> cast_Point2f (const std::vector<T> &v)
{
    std::vector<cv::Point2f> result;
    for (const T &t : v)
    {
        result.push_back (cv::Point2f (t.x, t.y));
    }
    return result;
}

/**
 * @file
 * @brief Given points, makes rotated bounding box
 * 
 * @param nb points
 * @return std::pair <bool, BodyFeatures> : <are features valid?, features>
 */
std::pair<bool, BodyFeatures>
bounding_rotated_box (std::vector<cv::Point2f> nb);

/**
 * @brief Makes an upright bounding box around points
 * 
 * @tparam Pt template for point (Box2D, OpenCV or similar)
 * @param nb points
 * @return std::pair<bool,BodyFeatures> (is the object valid, object)
 */
template <class Pt>
std::pair<bool, BodyFeatures> bounding_box (std::vector<Pt> &nb)
{ //gets bounding box of points
    float l = (0.0005 * 2), w = (0.0005 * 2);
    float x_glob = 0.0f, y_glob = 0.0f;
    std::pair<bool, BodyFeatures> result (0, BodyFeatures ());
    if (nb.empty ())
    {
        return result;
    }
    CompareX compareX;
    CompareY compareY;
    typename std::vector<Pt>::iterator maxx
        = std::max_element (nb.begin (), nb.end (), compareX);
    typename std::vector<Pt>::iterator miny
        = std::min_element (nb.begin (), nb.end (), compareY);
    typename std::vector<Pt>::iterator minx
        = std::min_element (nb.begin (), nb.end (), compareX);
    typename std::vector<Pt>::iterator maxy
        = std::max_element (nb.begin (), nb.end (), compareY);
    if (minx->x != maxx->x)
    {
        w = fabs ((*maxx).x - (*minx).x);
    }
    if (miny->y != maxy->y)
    {
        l = fabs ((*maxy).y - (*miny).y);
    }
    x_glob = ((*maxx).x + (*minx).x) / 2;
    y_glob = ((*maxy).y + (*miny).y) / 2;
    result.second.halfLength = l / 2;
    result.second.halfWidth = w / 2;
    result.second.pose.p = b2Vec2 (x_glob, y_glob);
    result.first = true;
    return result;
}

/*
 * Each point is an object and turns into a tiny body. Not very effective for
 * simulation or collision detection.
 */
class WorldBuilder
{
  public:
    WorldBuilder () = default;

    using OnWorldReady = std::function<void (std::shared_ptr<b2World>)>;

  protected:
    float simulationStep = BOX2DRANGE;
    int bodies = 0;
    std::vector<BodyFeatures> world_objects;
    friend class Configurator;
    std::shared_ptr<b2World> world;
    std::thread asyncThread;
    std::atomic<bool> isWorldBuilding;
    OnWorldReady onWorldReady;

  public:
    void registerWorldReadyCallback (OnWorldReady cb) { onWorldReady = cb; }

  public:
    enum CLUSTERING
    {
        BOX = 0,
        KMEANS = 1,
        PARTITION = 2
    }; //BOX: bounding box around points

    void doAsyncBuildWorld (CoordinateContainer &coords, b2Transform start,
                            float halfWindowWidth = HALF_WINDOW_WIDTH,
                            CLUSTERING clustering = PARTITION)
    {
        if (isWorldBuilding)
        {
            fprintf (stderr, "Still worldbuilding.\n");
            return;
        }
        asyncThread = std::thread ([&] () {
            isWorldBuilding = true;
            auto world
                = buildWorld (coords, start, halfWindowWidth, clustering);
            if (onWorldReady)
            {
                onWorldReady (world);
            }
            isWorldBuilding = false;
        });
    }

    struct CompareCluster
    {
        CompareCluster () = default;

        bool operator() (const BodyFeatures &b1, const BodyFeatures &b2)
        { //distances of centre from start
            bool result = false;
            if (fabs (atan2 (b1.pose.p.y, b1.pose.p.x))
                    < fabs (atan2 (b2.pose.p.y, b2.pose.p.x))
                && b1.pose.p.Length () <= b2.pose.p.Length ())
            {
                result = true;
            }
            return result;
        }
    };

    static std::pair<CoordinateContainer, bool> salientPoints (
        b2Transform, const CoordinateContainer &,
        std::pair<
            Pointf,
            Pointf>); //gets points from the raw data that are relevant to the task based on bounding boxes

    /**
     * @brief Creates a body in the box2d world, and if the features represent a disturbance to which the attention window needs to
     * be assigned, a flag is assigned to the body user data
     * 
     * @param w the box2d world
     * @param features features of the body to be created
     * @return * b2Body* 
     */
    b2Body *makeBody (const BodyFeatures &features);

    /**
     * @brief returns a bounding box encompassing all points provided
     * 
     * @return std::vector <BodyFeatures> 
     */
    std::vector<BodyFeatures> processData (const CoordinateContainer &,
                                           const b2Transform &);

    /**
     * @brief Cluster point cloud data using a custom algorithm
     * @param pts point cloud
     * @param start start robot transform
     * @param clustering the clustering algorithm
     * return a vector of bodyfeatures
     */
    std::vector<BodyFeatures> cluster_data (const CoordinateContainer &pts,
                                            const b2Transform &start,
                                            CLUSTERING clustering = PARTITION);

    /**
     * @brief clusters point clouds into objects and returns a vector of body features
     * @param current point cloud
     * @param start robot position
     * @param partition algorithm used for partition
     */
    virtual std::vector<BodyFeatures>
    getFeatures (const CoordinateContainer &current, b2Transform start,
                 CLUSTERING clustering = PARTITION);

    /**
     * @brief Creates bodies (objects) in the box2d world
     */
    virtual std::shared_ptr<b2World>
    buildWorld (CoordinateContainer &coords, b2Transform,
                float halfWindowWidth = 0.15,
                CLUSTERING clustering = CLUSTERING::PARTITION);

    //returns top and bottom of rotated rectangle (not side-specific)
    static std::pair<Pointf, Pointf>
    bounds (b2Transform t, float boxLength, float halfWindowWidth,
            std::vector<Pointf> *_bounds
            = NULL); //returns bottom and top of bounding box

    /**
     * @brief Makes a box 
     * 
     * @param halfWindowWidth 
     * @param boxLength 
     * @param start 
     * @param d 
     * @return b2PolygonShape 
     */
    b2PolygonShape object_filtering_box (float halfWindowWidth,
                                         float boxLength, b2Transform start);

    std::pair<bool, BodyFeatures>
    bounding_approx_poly (std::vector<cv::Point2f> nb);

    /**
     * @brief Clusters points using k-means algorithm
     * 
     * @return std::vector <std::vector<cv::Point2f>> 
     */
    std::vector<std::vector<cv::Point2f> >
    kmeans_clusters (std::vector<cv::Point2f>, std::vector<cv::Point2f> &);

    /**
     * @brief Clusters points using the partition algorithm
     * 
     * @return std::vector <std::vector<cv::Point2f>> 
     */
    std::vector<std::vector<cv::Point2f> >
        partition_clusters (std::vector<cv::Point2f>);

    b2Vec2 averagePoint (
        const CoordinateContainer &,
        float rad
        = 0.025); //finds centroid of a poitn cluster, return position vec difference

    b2Body *get_robot ();

    b2Fixture *get_chassis (b2Body *);

    std::vector<BodyFeatures> &get_world_objects () { return world_objects; }

    void setSimulationStep (float f) { simulationStep = f; }

    /**
     * Gets the LIDAR coordinates after clustering or other filtering.
     */
    void object_dump (const char *filename);

    /**
     * Saves the bodies in the box2D world. Dataformat: x,y,area.
     */
    void bodies_dump (const char *filename);

    /**
     * Saves SVG
     */
     void exportWorldToSVG(const char* filename, float scale = 100.0f);

};

/**
 * The LIDAR scan is clustered and differently sized
 * bodies are created containing these clusters.
 */
class WorldClusterBuilder : public WorldBuilder
{
  protected:
    virtual std::vector<BodyFeatures>
    getFeatures (const CoordinateContainer &current, b2Transform start,
                 CLUSTERING clustering) override;
};

/**
 * The LIDAR scan is clustered and only those bodies
 * are exported which are in the way.
 */
class FocusedBuilder : public virtual WorldClusterBuilder
{
  protected:
    virtual std::shared_ptr<b2World>
    buildWorld (CoordinateContainer &coords, b2Transform start,
                float halfWindowWidth = HALF_WINDOW_WIDTH,
                CLUSTERING clustering = CLUSTERING::PARTITION) override;
};
