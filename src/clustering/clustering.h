#ifndef CLUSTERS_H
#define CLUSTERS_H 

#include <ped_agent.h>
#include <features/features_icra.h>
#include <simple_utils.h>

#include <opencv2/core/core.hpp>


class CLusters
{
public:

    explicit CLusters() {
        features_.reset(new CFeatures_ICRA());
    }
    virtual ~CLusters() {}

    inline void clusterNeighborHood(std::vector<Ped::Tagent*> agents, Ped::Tagent *robot, double range)
    {
        // compute the features for each agent
        typedef std::pair<Ped::Tagent*, FeatureICRA> afeature_t;

        std::vector<afeature_t> agent_feature_sets;


        foreach(Ped::Tagent* a, agents) {
            FeatureICRA f = features_->computeCellFeaturesFS1(agents, a, a->getx(), a->gety(), range);

            agent_feature_sets.push_back(std::make_pair(a, f));
        }


        // perform clustering on the data
        using namespace cv;

        Mat samples(agent_feature_sets.size(), 13, CV_32F);
        for (int i = 0; i < agent_feature_sets.size(); i++) {
            for (int j = 0; j < 13; j++) {
                samples.at<float>(i, j) = agent_feature_sets[i].second[j];
            }
        }

        int clusterCount = 5;
        Mat labels;
        int attempts = 5;
        Mat centers;

        // do the clustering
        kmeans(samples, clusterCount, labels, 
            TermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 10000, 0.0001), 
            attempts, KMEANS_PP_CENTERS, centers );


        // color the clusters differently
        

    }

private:

    CFeaturesICRAPtr features_;

};

#endif