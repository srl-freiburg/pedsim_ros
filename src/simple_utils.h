/// Simple utilities use in the simulator 

#ifndef SIMPLE_UTILS_H
#define SIMPLE_UTILS_H

#include <boost/random.hpp>
#include <boost/math/distributions/normal.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_int.hpp>
#include <boost/random/variate_generator.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <list>
#include <vector>
#include <ctime>
#include <algorithm>

#include <ped_agent.h>
#include <grid.h>
#include <config.h>
#include <trajectory.h>

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

// #include "cell.h"


// forward declarations
class QGraphicsRectItem;



/// \brief Sampling a distributions
class Sampler
{
public:
    explicit Sampler(double mean, double sigma) 
    : rng(), resampler(rng, boost::normal_distribution<>(mean, sigma)) 
    {
        rng.seed(std::time(0));
    }
    
    ~Sampler() {}

    
    // sample from a Normal distribution
    double sampleFromNormalDistribution();

    // pick a random agent
    int pickInt(int min, int max);

private:

    boost::mt19937 rng;
    boost::normal_distribution<> distribution;
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> > resampler;
};


/// ==================================================
/// Feature Containers
/// ==================================================

/// simple 3 level feature
typedef std::array<size_t, 3> SingleFeature;

/// okalICRA14 features (3 binarized into 12 bins)
typedef std::array<double, 13> FeatureICRA;

/// Kim and Pineau (4 features binarized into 12 bins)
typedef std::array<size_t, 12> FeatureKIM;


/// ==================================================
/// Policy container
/// ==================================================
typedef std::array<double, 13> PolicyICRA;
typedef std::array<double, 12> PolicyKIM;

PolicyICRA readICRAPolicyFromFile(std::string filename);
PolicyKIM readKIMPolicyFromFile(std::string filename);


/// ==================================================
/// Proxemics and Anisotropic statistics
/// ==================================================
typedef struct ProxemicsStatistics
{
    unsigned int intimate;
    unsigned int personal;
    unsigned int social;
    unsigned int publik;

    ProxemicsStatistics() {
        intimate = 0;
        personal = 0;
        social = 0;
        publik = 0;
    }

    ProxemicsStatistics(int i, int p, int s, int k) {
        intimate = i;
        personal = p;
        social = s;
        publik = k;
    }

} Proxemics;

typedef struct AnisotropicStatistics
{
    unsigned int towards;
    unsigned int orthogonal;
    unsigned int behind;

    AnisotropicStatistics() {
        towards = 0;
        orthogonal = 0;
        behind = 0;
    }

    AnisotropicStatistics(int i, int p, int s) {
        towards = i;
        orthogonal = p;
        behind = s;
    }

} Anisotropic;

/// Direction of robot intrusion
typedef enum AngleDirection
{
    TOWARDS,
    ORTHOGONAL,
    AWAY,
    NODIR
} ADirection;


/// ==================================================
/// Euclidean distance
/// ==================================================
double eDist(double x1, double y1, double x2, double y2);


/// ==================================================
/// Get all the agents within a range
/// ==================================================
std::vector<Ped::Tagent*> getAgentsInRange(std::vector<Ped::Tagent*> agents, double gx, double gy, double range);
int countAgentsInZone(std::vector<Ped::Tagent*> agents, Tpoint position, double range_lower, double range_upper, int id);

/// ==================================================
/// Check if a cell is in grid
/// ==================================================
bool withinGrid(double cx, double cy, Grid* grid);


/// ==================================================
/// Get enclosing cell
/// ==================================================
Tpoint getEnclosingCell(Grid* grid, double x, double y);



/// angle between vectors
double computeAngleBetweenVectors(Ped::Tvector a, Ped::Tvector b);
double diffAngle2(double a1, double a2);
double normAngle2(double a, double mina);
double normangle(double alpha, double min);
double diffangle(double alpha1, double alpha2);


/// ==================================================
/// Check proxemics
/// ==================================================
int robotIntruding(Ped::Tagent* agent, Ped::Tagent* robot, double density);

/// compute the general categorization of the direction of an agent
/// relative to the robot
ADirection computeAngleDirection(Ped::Tvector a, Ped::Tvector b);

/// find out the direction of intrusion of the robot onto the agent
ADirection getIntrusionDirection(Ped::Tagent* agent, Ped::Tagent* robot);


/// flipping features
inline FeatureICRA flipFeature(FeatureICRA f, PolicyICRA p)
{
    FeatureICRA fn; fn.fill(0);
    for (size_t i = 0; i < f.size(); i++)  {
        fn[i] = 1 - f[i];
    }

    fn[12] = 1 - p[12];

    return fn;
}

inline FeatureICRA helbingFlip(FeatureICRA f)
{
    float sum = 0;
    for (size_t i = 0; i < f.size(); i++)  {
        sum += f[i];
    }

    FeatureICRA fn; fn.fill(0);
    for (size_t i = 0; i < 3; i++)  {
        fn[i] = sum - f[i];
    }
    return fn;
}



/// vector operation helpers
inline double vecDot(Ped::Tvector a, Ped::Tvector b)
{
    return (a.x * b.x) + (a.y * b.y) + (a.z*b.z);
}

inline bool vecNaN(Ped::Tvector a)
{
    if (std::isnan(abs(a.x)) || std::isnan(abs(a.y)) || std::isnan(abs(a.z)))
        return true;
    else
        return false;
}


inline double vecLen(const Ped::Tvector& v)
{
    return sqrt((v.x*v.x) + (v.y*v.y) + (v.z*v.z));
}


inline Ped::Tvector vecSub(const Ped::Tvector& u, const Ped::Tvector& v)
{
    return Ped::Tvector(u.x-v.x, u.y-v.y, u.z-v.y);
}

inline Ped::Tvector vecAdd(const Ped::Tvector& u, const Ped::Tvector& v)
{
    return Ped::Tvector(u.x+v.x, u.y+v.y, u.z+v.y);
}


inline size_t maxIdx(double value, const double *reference, size_t length)
{
    size_t result = 0;
    for (size_t i = 0; i < length; ++i) {
        if (value >= reference[i])
            result = i;
    }

    return result;
}



inline int countAgentsInCell(const std::vector<Ped::Tagent*> agents, Tpoint cell)
{
    int count = 0;

    foreach(Ped::Tagent* agent, agents) {
        if ( (agent->getx() >= cell.x && agent->getx() <= cell.x+CONFIG.width) &&
            (agent->gety() >= cell.y && agent->gety() <= cell.y+CONFIG.height) ) 
        {
            count++;
            break; // useful since we only need one agent to block the cell
        }
    }

    return count;
}

inline bool pathIsBlocked(const std::vector<Ped::Tagent*> agents, const std::vector<Tpoint>& path)
{
    bool blocked = false;

    /// check all cells along the path to see if agents belong the them
    
    //version 1 - depedent on cell size
    // foreach(Tpoint p, path) {
    //     if (countAgentsInCell(agents, p) > 0 ) {
    //         blocked = true;
    //         break;
    //     }
    // }

    // version 2 - uses only distances
    // TODO -remove c++11 features
    foreach(Tpoint p, path) {
        foreach(Ped::Tagent* agent, agents) {
            if (eDist(p.x, p.y, agent->getx(), agent->gety()) <= CONFIG.touch_radius) {
                blocked = true;
                break;  // be lazy
            }
        }
    }

    return blocked;
}




/// simple node
struct NodeT
{
    Tpoint p;
    double cost;

    NodeT() {
        p = Tpoint(0.0, 0.0);
        cost = 0.0;
    }

    NodeT(double x, double y, double c) {
        p = Tpoint(x, y);
        cost = c;
    }
};


inline bool myComp(NodeT a, NodeT b) { return (a.cost < b.cost); }


inline Tpoint getLowCellInRange(Tpoint cc, double range, boost::numeric::ublas::matrix<double> cmap, Grid* grid)
{
    QVector<QVector<Cell*> > cells = grid->getCells();

    std:vector<NodeT> range_cells;

    foreach(auto& row, cells) 
    {
        foreach(Cell* cell, row) 
        {
            double rx  = cell->x();
            double ry  = cell->y();

            if (eDist(rx, ry, cc.x*CONFIG.width, cc.y*CONFIG.height) < range)
            {
                double xx = rx / CONFIG.width;
                double yy = ry / CONFIG.height;

                range_cells.push_back(NodeT(xx, yy, cmap(xx,yy)));
            }
        }
    }

    std::sort(range_cells.begin(), range_cells.end(), myComp);

    return range_cells[0].p;
}

inline Tpoint nextLowCell(Tpoint cc, boost::numeric::ublas::matrix<double> cmap)
{
    // check all the eight directions for lowest cost cell

    std::vector<NodeT> costs; costs.resize(8);

    costs.push_back( NodeT(cc.x,   cc.y+1, cmap(cc.x,   cc.y+1) ) );
    costs.push_back( NodeT(cc.x+1, cc.y+1, cmap(cc.x+1, cc.y+1) ) );
    costs.push_back( NodeT(cc.x+1, cc.y  , cmap(cc.x+1, cc.y  ) ) );
    costs.push_back( NodeT(cc.x+1, cc.y-1, cmap(cc.x+1, cc.y-1) ) );
    costs.push_back( NodeT(cc.x,   cc.y-1, cmap(cc.x,   cc.y-1) ) );
    costs.push_back( NodeT(cc.x-1, cc.y-1, cmap(cc.x-1, cc.y-1) ) );
    costs.push_back( NodeT(cc.x-1, cc.y  , cmap(cc.x-1, cc.y  ) ) );
    costs.push_back( NodeT(cc.x-1, cc.y+1, cmap(cc.x-1, cc.y+1) ) );

    std::sort(costs.begin(), costs.end(), myComp);

    return costs[0].p;
}


#endif
