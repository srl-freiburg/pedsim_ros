#include <features/features_kim.h>

double CFeatures_KIM::computeAverageMagnitude(std::vector<Ped::Tagent*> agents, Ped::Tagent *robot)
{
    if (agents.size() == 0) return 0.0;

    double mag = 0.0;
    for (std::vector<Ped::Tagent*>::const_iterator iter = agents.begin(); iter != agents.end(); ++iter) {
        Ped::Tagent *a = (*iter);

        Ped::Tvector rel_vel(a->getvx()-robot->getvx(), a->getvy()-robot->getvy(), 0);
        mag += vecLen(rel_vel);
    }

    return mag / agents.size();
}

std::array<size_t, 6> CFeatures_KIM::computeVelocityFeature(std::vector<Ped::Tagent*> agents,
                                                            Ped::Tagent *robot)
{
    double magnitude = computeAverageMagnitude(agents, robot);
    int towards = 0, ortho = 0, away = 0;

    for (std::vector<Ped::Tagent*>::const_iterator iter = agents.begin(); iter != agents.end(); ++iter) {
        Ped::Tagent *a = (*iter);

        Ped::Tvector green(a->getvx()-robot->getvx(), a->getvy()-robot->getvy(), 0);
        Ped::Tvector red(a->getx()-robot->getx(), a->gety()-robot->gety(), 0);

        // double alpha = computeAngleBetweenVectors(red, green);

        ADirection direction = computeAngleDirection(red, green);
        if (direction == TOWARDS) towards++;
        else if (direction == ORTHOGONAL) ortho++;
        else away++;
        
        // // bining
        // if (alpha >= -M_PI/4.0 && alpha < M_PI/4.0)
        //     away++;
        // if ( (alpha >= M_PI/4.0 && alpha < 3*M_PI/4.0) || (alpha >= -3*M_PI/4.0 && alpha < -M_PI/4.0) )
        //     ortho++;
        // if (alpha > -3*M_PI/4.0 && alpha <= 3*M_PI/4.0 )
        //     towards++;
    }

    /// pack the feature

    // [low, med, high]
    std::array<size_t, 6> velocity; velocity.fill(0);
    velocity[maxIdx(magnitude, CONFIG.velocities, 3)] = 1;

    // [towads, ortho, away]
    if (maximum(towards, ortho, away) == towards) velocity[3] = 1;
    if (maximum(towards, ortho, away) == ortho) velocity[4] = 1;
    if (maximum(towards, ortho, away) == away) velocity[5] = 1;

    return velocity;
}

/// compute the full feature for a cell/location
FeatureKIM CFeatures_KIM::computeCellICRAFeatures(std::vector<Ped::Tagent*> agents,
                                                Ped::Tagent *robot, Tpoint query,
                                                Tpoint goal, double range)
{
    // extract neighbors and the robot
    std::vector<Ped::Tagent *> neighbors = getAgentsInRange(agents, query.x, query.y, range);

    // compute density feature [Low, Med, High]
    int density = neighbors.size();
    SingleFeature fden; fden.fill(0);
    fden[maxIdx(density*1.0, CONFIG.densities, 3)] = 1;

    // DistGoal feature [Low, Med, High]
    double dist_goal = eDist(robot->getx(), robot->gety(), goal.x, goal.y);
    SingleFeature fdist; fdist.fill(0);
    if (dist_goal <= 10) fdist[0] = 1;
    if (dist_goal > 10 && dist_goal <= 100) fdist[1] = 1;
    if (dist_goal > 100 ) fdist[2] = 1;

    // velocity features
    std::array<size_t, 6> velocity = computeVelocityFeature(neighbors, robot);

    /// pack full feature
    FeatureKIM cell_feature;
    cell_feature.fill(0);

    cell_feature[0] = fden[0];
    cell_feature[1] = fden[1];
    cell_feature[2] = fden[2];
    cell_feature[3] = fdist[0];
    cell_feature[4] = fdist[1];
    cell_feature[5] = fdist[2];
    cell_feature[6] = velocity[0];
    cell_feature[7] = velocity[1];
    cell_feature[8] = velocity[2];
    cell_feature[9] = velocity[3];
    cell_feature[10] = velocity[4];
    cell_feature[11] = velocity[5];

    return cell_feature;
}

/// evaluate the cost of a cell based on feature and policy
double CFeatures_KIM::computeFeaturesCost(const FeatureKIM &features, const PolicyKIM &policy)
{
    double cost = 0.0;

    for (int i = 0; i < features.size(); i++)
        cost += features[i] * policy[i];

    return cost / (double)features.size();
}

