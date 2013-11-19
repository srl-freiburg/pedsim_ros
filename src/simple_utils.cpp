#include "simple_utils.h"

double Sampler::sampleFromNormalDistribution()
{
    double val = resampler();

    if (val < 0.0) val = 0.01;

    if (val > 2.5) val = 2.5;

    return val;
}

int Sampler::pickInt(int min, int max)
{
    boost::uniform_int<> dist(min, max);
    boost::variate_generator<boost::mt19937&, boost::uniform_int<> > die(rng, dist);
    return die();
}


/// Always returns the distance in real world units (metres)
double eDist(double x1, double y1, double x2, double y2)
{
    double dx = (x2-x1) * (1/20.0);
    double dy = (y2-y1) * (1/20.0);

    return sqrt( pow(dx, 2.0) + pow(dy, 2.0) );    // distance in metres
}

std::vector<Ped::Tagent*> getAgentsInRange(std::vector<Ped::Tagent*> agents, double gx, double gy, double range)
{
    std::vector<Ped::Tagent*> ragents; ragents.clear();

    for (vector<Ped::Tagent*>::const_iterator iter = agents.begin(); iter != agents.end(); ++iter) {
        Ped::Tagent *a = (*iter);
        
        // change this to the right robot type
        if (a->gettype() == 1) {
            continue;
        }

        if (eDist(a->getx(), a->gety(), gx, gy) <= range)
            ragents.push_back(a);
    }
    return ragents;
}

int countAgentsInZone(std::vector<Ped::Tagent*> agents, Tpoint position, double range_lower, double range_upper, int id) {
    int count  = 0;
    for (vector<Ped::Tagent*>::const_iterator iter = agents.begin(); iter != agents.end(); ++iter) {
        Ped::Tagent *a = (*iter);

        // ignore our own selves (the robot)
        if (a->gettype() == id) {
            continue;
        }

        double distance = eDist(position.x, position.y, a->getx(), a->gety());

        if (distance >= range_lower && distance < range_upper ) {     // exclude inner zones
        // if (distance < range_upper ) {  // for the case of unions 
            count++;
        }
    }

    return count;
}

bool withinGrid(double cx, double cy, Grid* grid)
{
    if (cx < grid->minx || cy < grid->miny)
        return false;

    if (cx >= (grid->minx + grid->width) || cy >= ( grid->miny + grid->height) )
        return false;

    return true;
}


/// read policy
PolicyICRA readICRAPolicyFromFile(std::string filename)
{
    PolicyICRA weights; weights.fill(1);

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(filename, pt);

    weights[0] = pt.get<double>("Density.Low");
    weights[1] = pt.get<double>("Density.Medium");
    weights[2] = pt.get<double>("Density.High");

    weights[3] = pt.get<double>("Velocity.Towards_Slow");
    weights[4] = pt.get<double>("Velocity.Towards_Normal");
    weights[5] = pt.get<double>("Velocity.Towards_Fast");

    weights[6] = pt.get<double>("Velocity.Orthogonal_Slow");
    weights[7] = pt.get<double>("Velocity.Orthogonal_Normal");
    weights[8] = pt.get<double>("Velocity.Orthogonal_Fast");

    weights[9] = pt.get<double>("Velocity.Away_Slow");
    weights[10] = pt.get<double>("Velocity.Away_Normal");
    weights[11] = pt.get<double>("Velocity.Away_Fast");

    weights[12] = pt.get<double>("DefaultCost.cost");

    return weights;
}

PolicyKIM readKIMPolicyFromFile(std::string filename)
{
    PolicyKIM weight; weight.fill(1);

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(filename, pt);

    weight[0] = pt.get<double>("Density.Low");
    weight[1] = pt.get<double>("Density.Medium");
    weight[2] = pt.get<double>("Density.High");

    weight[3] = pt.get<double>("VelMag.Low");
    weight[4] = pt.get<double>("VelMag.Medium");
    weight[5] = pt.get<double>("VelMag.High");

    weight[6] = pt.get<double>("VelDir.Low");
    weight[7] = pt.get<double>("VelDir.Medium");
    weight[8] = pt.get<double>("VelDir.High");

    weight[9] = pt.get<double>("DistGoal.Low");
    weight[10] = pt.get<double>("DistGoal.Medium");
    weight[11] = pt.get<double>("DistGoal.High");

    return weight;
}



Tpoint getEnclosingCell(Grid* grid, double x, double y)
{
    if (((x-grid->minx) < 0) || ((y-grid->miny) < 0))
        std::cerr << "Cell coordinates out of grid!" << std::endl;

    unsigned int cellx = (x-grid->minx)/CONFIG.width;
    unsigned int celly = (y-grid->miny)/CONFIG.height;

    if ((cellx >= (unsigned int)grid->width || (celly >= (unsigned int)grid->height)))
        std::cerr << "Cell coordinates out of grid!" << std::endl;

    return Tpoint(cellx, celly, 0);
}

double computeAngleBetweenVectors(Ped::Tvector a, Ped::Tvector b)
{
    a.normalize();
    b.normalize();

    double dt = (a.x * b.x) + (a.y * b.y);
    double angle_dot =  acos(dt);
    angle_dot = normangle(angle_dot, 0);

    double ac1, ac2;
    ac1 = atan2(a.y, a.x);
    ac2 = atan2(b.y, b.x);

    double dangle = diffangle(ac1, ac2);
    dangle = normangle(dangle, 0);

    return dangle;
}

// Determines the minimal difference dalpha = alpha1 - alpha2 between two angles alpha1 and alpha2
double diffangle(double alpha1, double alpha2) {
    double delta;

    // normalize angles alpha1 and alpha2
    alpha1 = normangle(alpha1, 0);
    alpha2 = normangle(alpha2, 0);

    // take difference and unwrap
    delta = alpha1 - alpha2;
    if (alpha1 > alpha2) {
        while (delta > M_PI) {
            delta -= 2.0*M_PI;
        }
    }
    else if (alpha2 > alpha1) {
        while (delta < -M_PI) {
            delta += 2.0*M_PI;
        }
    }
    return delta;
};


// Puts angle alpha into the interval [min..min+2*pi[
double normangle(double alpha, double min) {
    while (alpha >= min+2.0*M_PI) {
        alpha -= 2.0*M_PI;
    }
    while (alpha < min) {
        alpha += 2.0*M_PI;
    }
    return alpha;
};

double normAngle2(double a, double mina)
{
    double ar;
    if (a < INFINITY) {
        while(a >= mina + 2*M_PI) {
            a = a - 2*M_PI;
        }

        while(a < mina) {
            a = a - 2*M_PI;
        }

        ar = a;
    }
    else
      ar = INFINITY;

    return ar;
}

double diffAngle2(double a1, double a2)
{
    double delta;

    if ((a1 < INFINITY) && (a2 < INFINITY)) {
        a1 = normAngle2(a1,0);
        a2 = normAngle2(a2,0);

        delta = a1 - a2;

        if (a1 > a2) {
            while (delta >  M_PI)
                delta = delta - 2*M_PI;
        }
        else if (a2 > a1) {
            while (delta < -M_PI)
                delta = delta + 2*M_PI;
        }
    }
    else {
      delta = INFINITY;
    }

    return delta;
}




int robotIntruding(Ped::Tagent* agent, Ped::Tagent* robot, double density)
{
    int ak = 3.2;

    int bk = 1;
    double lmbda = 0.4;     // avoid name clash with c++11 lambda
    double rij = 0.2;
    double dij = 1.0;

    Ped::Tvector ei(agent->getvx(), agent->getvy(), 0);
    ei.normalize();

    Ped::Tvector nij(robot->getx()-agent->getx(), robot->gety()-agent->gety(), 0);
    nij.normalize();

    //dij = sqrt(nij.x*nij.x + nij.y*nij.y);
    double cphi = -nij.x*ei.x + -nij.y*ei.y; // cos

    double alpha = ak*exp((rij-dij)/bk);
    double beta = lmbda + ((1-lmbda) * (1+cphi)/2.0);

    Ped::Tvector fani(alpha*nij.x*beta, alpha*nij.y*beta, 0);

    // check if robot is inside
    double df = eDist(fani.x, fani.y, agent->getx(), agent->gety());
    double dr = eDist(robot->getx(), robot->gety(), agent->getx(), agent->gety());

    if (dr <= df)
        return agent->getid();

    return -1;
}



ADirection computeAngleDirection(Ped::Tvector a, Ped::Tvector b)
{
    // the dot product between the vectors
    a.normalize();
    b.normalize();
    double dot = (a.x * b.x) + (a.y * b.y);

    size_t choice = maxIdx(dot, CONFIG.angles, 3);

    if (choice == 0) return AWAY;
    else if (choice == 1) return ORTHOGONAL;
    else return TOWARDS;
}

ADirection getIntrusionDirection(Ped::Tagent* agent, Ped::Tagent* robot)
{
    Ped::Tvector agent_direction(agent->getvx(), agent->getvy(), 0);
    Ped::Tvector robot_direction(robot->getvx(), robot->getvy(), 0);

    // double alpha = abs(computeAngleBetweenVectors(robot_direction, agent_direction));

    // if (std::isnan(alpha)) return NODIR;

    // if ((alpha >= 7*M_PI/4.0 && alpha < 2*M_PI) || (alpha >= 0 && alpha < M_PI/4.0))
    //     return AWAY;
    // else if ( (alpha >= M_PI/4.0 && alpha < 3*M_PI/4.0) || (alpha >= 5*M_PI/4.0 && alpha < 7*M_PI/4.0) )
    //     return ORTHOGONAL;
    // else if (alpha > 3*M_PI/4.0 && alpha <= 5*M_PI/4.0 )
    //     return TOWARDS;

   return computeAngleDirection(agent_direction, robot_direction);
}


