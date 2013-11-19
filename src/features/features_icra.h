#ifndef FEATURES_ICRA_H
#define FEATURES_ICRA_H

#include <simple_utils.h>
#include <agent.h>
#include <ped_vector.h>

#include <boost/shared_ptr.hpp>

/// \brief Compute the features defined on the okalICRA14 paper
class CFeatures_ICRA
{
public:
    explicit CFeatures_ICRA() {}

    /// compute velocity feature
    std::array<size_t, 9> computeVelocityFeature(std::vector<Ped::Tagent *> agents, Ped::Tagent *robot);

    /// compute the average rel vel mag for a binning
    double computeAverageMagnitude(std::vector<Ped::Tagent*> agents, Ped::Tagent *robot);

    /// compute the full feature for a cell/location
    FeatureICRA computeCellFeaturesFS1(std::vector<Ped::Tagent *> agents,
                                        Ped::Tagent *robot, double cx, double cy,
                                        double range);

    FeatureICRA computeCellFeaturesFS2(std::vector<Ped::Tagent *> agents,
                                        Ped::Tagent *robot, double cx, double cy,
                                        double range);

    FeatureICRA computeCellFeaturesFS3(std::vector<Ped::Tagent *> agents,
                                        Ped::Tagent *robot, double cx, double cy,
                                        double range);

    FeatureICRA computeCellFeaturesFS4(std::vector<Ped::Tagent *> agents,
                                        Ped::Tagent *robot, double cx, double cy,
                                        double range);

    // Anisotropic (based on Dizan's helbing feature modification)
    FeatureICRA computeCellFeaturesFS5(std::vector<Ped::Tagent *> agents,
                                        Ped::Tagent *robot, double cx, double cy,
                                        double range);

    // helbing feature
    FeatureICRA computeCellFeaturesFS6(std::vector<Ped::Tagent *> agents,
                                        Ped::Tagent *robot, double cx, double cy,
                                        double range);

    std::array<float, 2> computeDizanFeature(std::vector<Ped::Tagent *> agents,
                                              Ped::Tagent *robot, double cx, double cy,
                                              double range);

    /// evaluate the cost of a cell based on feature and policy
    double computeFeaturesCost(const FeatureICRA& features, const PolicyICRA& policy)
    {
        double cost = 0.0;

        for (int i = 0; i < 13; i++)
            cost += features[i] * ( policy[i]);

        return cost;
    }
    

    double computeDizanCost(std::array<double, 2> f, PolicyICRA p)
    {
        return (f[0] * p[0]) + (f[1] * p[1]);
    }


    std::array<size_t, 9> cartesianProduct(const SingleFeature a, const SingleFeature b)
    {
        std::array<size_t, 9> prod; prod.fill(0);
        int elem = 0;

        for (int i = 0; i < a.size(); i++) {
            for (int j = 0; j < b.size(); j++) {
                prod[elem] = (a[i] || b[j]);
                elem++;
            }
        }

        return prod;
    }

    int maximum(int a, int b, int c)
    {
       int max = ( a < b ) ? b : a;
       return ( ( max < c ) ? c : max );
    }


};



typedef boost::shared_ptr<CFeatures_ICRA> CFeaturesICRAPtr;
typedef boost::shared_ptr<CFeatures_ICRA const> CFeaturesICRAConstPtr;

#endif // FEATURES_ICRA_H
