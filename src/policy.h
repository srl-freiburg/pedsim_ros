#ifndef POLICY_H
#define POLICY_H

#include <string>
#include <vector>
#include <iostream>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>


/// For utilizing the old policy weights based on the Kim and Pineau paper


/// Binary weight of any feature
typedef struct BinaryWeight
{
    double high;
    double medium;
    double low;

    BinaryWeight()
    {
        high = 1.0;
        medium = 1.0;
        low = 1.0;
    }

    BinaryWeight(double h, double m, double l)
        : high(h), medium(m), low(l)
    {}

    BinaryWeight(double *w)
        : high(w[0]), medium(w[1]), low(w[2])
    {}

} Bweight;



/// Navigation policy that contains the learned weights used
/// to drive the robot
typedef struct NavigationPolicy
{
    Bweight density;
    Bweight vel_mag;
    Bweight vel_dir;
    Bweight dist_goal;


    NavigationPolicy() {}

    NavigationPolicy(Bweight d, Bweight vm, Bweight vd, Bweight dg)
        : density(d), vel_mag(vm), vel_dir(vd), dist_goal(dg)
    {}

} Policy;


/// Cell feature (raw unbinarized)
typedef struct CellFeature
{
   double density;
   double vel_mag;
   double vel_dir;
   double dist_goal;

   CellFeature()
   {
       density = 0.0;
       vel_mag = 0.0;
       vel_dir = 0.0;
       dist_goal = 0.0;
   }

   CellFeature(double d, double vm, double vd, double g)
       : density(d), vel_mag(vm), vel_dir(vd), dist_goal(g)
   {}

} CFeature;


/// binarized cell feature vector (len 12 vector)
/// ordered as [Den VelMag VelDir DistGoal]
typedef std::vector<size_t> BinaryCF;





std::vector<double> vectorizePolicy(const Policy& p);

/// \brief Read a policy file (.ini) with learned policies
Policy readPolicyFromFile(std::string filename);

/// simple policy printing
std::ostream& operator<<(std::ostream& output, const Policy& p);

/// binarize all the cell features for the state space an a given time
std::vector<BinaryCF > binarizeCellFeatures(const std::vector<CFeature>& vcf);

double scaleRatio(double x, double max, double min);

#endif // POLICY_H
