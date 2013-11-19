#ifndef FEATURES_KIM_H
#define FEATURES_KIM_H


#include <simple_utils.h>
#include <agent.h>
#include <ped_vector.h>


class CFeatures_KIM
{
public:
    explicit CFeatures_KIM() {}

    /// average velocity magnitudes
    double computeAverageMagnitude(std::vector<Ped::Tagent*> agents, Ped::Tagent *robot);

    /// compute the velocity feature (direction and magnitude)
    std::array<size_t, 6> computeVelocityFeature(std::vector<Ped::Tagent*> agents, Ped::Tagent *robot);

    /// compute the full feature for a cell/location
    FeatureKIM computeCellICRAFeatures(std::vector<Ped::Tagent*> agents,
                                       Ped::Tagent *robot, Tpoint query,
                                       Tpoint goal, double range);

    /// evaluate the cost of a cell based on feature and policy
    double computeFeaturesCost(const FeatureKIM &features, const PolicyKIM &policy);

private:

    inline int maximum(int a, int b, int c)
    {
       int max = ( a < b ) ? b : a;
       return ( ( max < c ) ? c : max );
    }
};

#endif // FEATURES_KIM_H
