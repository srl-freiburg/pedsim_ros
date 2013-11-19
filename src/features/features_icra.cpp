#include <features/features_icra.h>


double CFeatures_ICRA::computeAverageMagnitude(std::vector<Ped::Tagent*> agents, Ped::Tagent *robot)
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


/// Using the setting in okalICRA14
std::array<size_t, 9> CFeatures_ICRA::computeVelocityFeature(std::vector<Ped::Tagent *> agents, Ped::Tagent* robot)
{
    /// groups the neighbor obstacles into bins a1, a2, a3 based on their approach angles
    std::vector<Ped::Tagent*> a1, a2, a3;

    for (std::vector<Ped::Tagent*>::const_iterator iter = agents.begin(); iter != agents.end(); ++iter) 
    {
        Ped::Tagent *a = (*iter);

        Ped::Tvector green(a->getvx()-robot->getvx(), a->getvy()-robot->getvy(), 0);
        Ped::Tvector red((a->getx()-robot->getx())*(1/20.0), (a->gety()-robot->gety())*(1/20.0), 0);

        // Ped::Tvector green(a->getvx(), a->getvy(), 0);
        // Ped::Tvector red(robot->getvx(), robot->getvy(), 0);

        /// NOTE Old style, keep for sentimental purposes
        // double alpha = abs(computeAngleBetweenVectors(red, green));
        // bining ( assuming alpha is in [0,2pi]
        // if ((alpha >= 7*M_PI/4.0 && alpha < 2*M_PI) || (alpha >= 0 && alpha < M_PI/4.0))
        //     a3.push_back(a);
        // else if ( (alpha >= M_PI/4.0 && alpha < 3*M_PI/4.0) || (alpha >= 5*M_PI/4.0 && alpha < 7*M_PI/4.0) )
        //     a2.push_back(a);
        // else if (alpha > 3*M_PI/4.0 && alpha <= 5*M_PI/4.0 )
        //     a1.push_back(a);

        ADirection direction = computeAngleDirection(red, green);
        if (direction == TOWARDS) a3.push_back(a);
        else if (direction == ORTHOGONAL) a2.push_back(a);
        else a1.push_back(a);
    }

    /// pack and prep the velocity feature
    std::array<size_t, 9> velocity; velocity.fill(0);

    if (a1.size() > 0) {
        double magnitude = computeAverageMagnitude(a1, robot);
        velocity[maxIdx(magnitude, CONFIG.velocities, 3)] = 1;
    }

    if (a2.size() > 0) {
        double magnitude = computeAverageMagnitude(a2, robot);
        velocity[maxIdx(magnitude, CONFIG.velocities, 3) + 3] = 1;
    }

    if (a3.size() > 0) {
        double magnitude = computeAverageMagnitude(a3, robot);
        velocity[maxIdx(magnitude, CONFIG.velocities, 3) + 6] = 1;
    }

    return velocity;
}

FeatureICRA CFeatures_ICRA::computeCellFeaturesFS1(std::vector<Ped::Tagent *> agents, Ped::Tagent* robot, double cx, double cy, double range)
{
    // extract neighbors and the robot
    std::vector<Ped::Tagent *> neighbors = getAgentsInRange(agents, cx+(CONFIG.width/2), cy+(CONFIG.height/2), range);

    // compute density feature
    int density = neighbors.size();

    // bin density (Low, Med, High)
    SingleFeature fden; fden.fill(0);
    fden[maxIdx(density*1.0, CONFIG.densities, 3)] = 1;

    // get the velocity features
    std::array<size_t, 9> full_velocity = computeVelocityFeature(neighbors, robot);

    /// Pack all the resulting features into the 12-dim feature vector
    FeatureICRA cell_feature;
    cell_feature.fill(0);

    cell_feature[0] = fden[0];
    cell_feature[1] = fden[1];
    cell_feature[2] = fden[2];

    cell_feature[3] = full_velocity[0];
    cell_feature[4] = full_velocity[1];
    cell_feature[5] = full_velocity[2];
    cell_feature[6] = full_velocity[3];
    cell_feature[7] = full_velocity[4];
    cell_feature[8] = full_velocity[5];
    cell_feature[9] = full_velocity[6];
    cell_feature[10] = full_velocity[7];
    cell_feature[11] = full_velocity[8];
    cell_feature[12] = 0;

    return cell_feature;
}


FeatureICRA CFeatures_ICRA::computeCellFeaturesFS2(std::vector<Ped::Tagent *> agents,
                                    Ped::Tagent *robot, double cx, double cy,
                                    double range)
{
    // extract neighbors and the robot
    std::vector<Ped::Tagent *> neighbors = getAgentsInRange(agents, cx+(CONFIG.width/2), cy+(CONFIG.height/2), range);

    // compute density feature
    int density = neighbors.size();

    // bin density (Low, Med, High)
    SingleFeature fden; fden.fill(0);
    fden[maxIdx(density*1.0, CONFIG.densities, 3)] = 1;

    // velocities
    int towards = 0, ortho = 0, away = 0;
    for (std::vector<Ped::Tagent*>::const_iterator iter = neighbors.begin(); iter != neighbors.end(); ++iter) {
        Ped::Tagent *a = (*iter);
        Ped::Tvector green(a->getvx(), a->getvy(), 0);
        Ped::Tvector red(robot->getvx(), robot->getvy(), 0);

        ADirection direction = computeAngleDirection(red, green);
        if (direction == TOWARDS) towards++;
        else if (direction == ORTHOGONAL) ortho++;
        else away++;
    }

    double magnitude = 0.0;
    for (std::vector<Ped::Tagent*>::const_iterator iter = neighbors.begin(); iter != neighbors.end(); ++iter) {
        Ped::Tagent *a = (*iter);

        Ped::Tvector rel_vel(a->getvx()-robot->getvx(), a->getvy()-robot->getvy(), 0);
        magnitude += vecLen(rel_vel);
    }
    magnitude /= neighbors.size();

    // [low, med, high]
    std::array<size_t, 6> velocity; velocity.fill(0);
    velocity[maxIdx(magnitude, CONFIG.velocities, 3)] = 1;

    // [towads, ortho, away]
    if (maximum(towards, ortho, away) == towards) velocity[3] = 1;
    if (maximum(towards, ortho, away) == ortho) velocity[4] = 1;
    if (maximum(towards, ortho, away) == away) velocity[5] = 1;


    FeatureICRA cell_feature;
    cell_feature.fill(0);
    cell_feature[0] = fden[0];
    cell_feature[1] = fden[1];
    cell_feature[2] = fden[2];
    cell_feature[3] = velocity[0];
    cell_feature[4] = velocity[1];
    cell_feature[5] = velocity[2];
    cell_feature[6] = velocity[3];
    cell_feature[7] = velocity[4];
    cell_feature[8] = velocity[5];

    return cell_feature;
}

FeatureICRA CFeatures_ICRA::computeCellFeaturesFS3(std::vector<Ped::Tagent *> agents,
                                    Ped::Tagent *robot, double cx, double cy,
                                    double range)
{

    FeatureICRA cell_feature;
    cell_feature.fill(0);

    cell_feature = computeCellFeaturesFS1(agents, robot, cx, cy, range);

    cell_feature[12] = 1;   // activate the default feature

    return cell_feature;
}

FeatureICRA CFeatures_ICRA::computeCellFeaturesFS4(std::vector<Ped::Tagent *> agents,
                                    Ped::Tagent *robot, double cx, double cy,
                                    double range)
{

    FeatureICRA cell_feature;
    cell_feature.fill(0);

    cell_feature = computeCellFeaturesFS2(agents, robot, cx, cy, range);

    // add the default feature
    cell_feature[12] = 1;

    return cell_feature;
}


// new feature set
FeatureICRA CFeatures_ICRA::computeCellFeaturesFS5(std::vector<Ped::Tagent *> agents,
                                    Ped::Tagent *robot, double cx, double cy,
                                    double range)
{
    FeatureICRA cell_feature;
    cell_feature.fill(0);

    std::array<float, 2> df = computeDizanFeature(agents, robot, cx, cy, range);
    cell_feature[0] = df[0];
    cell_feature[1] = df[1];

    return cell_feature;
}

// another new feature set
// anisotropic features (3 element feature)
FeatureICRA CFeatures_ICRA::computeCellFeaturesFS6(std::vector<Ped::Tagent *> agents,
                                    Ped::Tagent *robot, double cx, double cy,
                                    double range)
{
    FeatureICRA cell_feature;
    cell_feature.fill(0);

    float lmbda = 0.2;
    double radius = 0.45;

    std::array<float, 3> feature; feature.fill(0);

    for (std::vector<Ped::Tagent*>::const_iterator iter = agents.begin(); iter != agents.end(); ++iter)
    {
        Ped::Tagent *a = (*iter);
        Ped::Tvector robot_position(robot->getx()*(1/20.0), robot->gety()*(1/20.0), 0.0);
        Ped::Tvector agent_position(a->getx()*(1/20.0), a->gety()*(1/20.0), 0.0);
        Ped::Tvector xRel = vecSub(agent_position, robot_position);

        Ped::Tvector n = xRel; 
        n.normalize();
        
        Ped::Tvector vOther(a->getvx(), a->getvy(), 0);
        vOther.normalize();
        
        Ped::Tvector e = vOther;

        double cosPhi = vecDot(Ped::Tvector(-n.x, -n.y, 0), e);
        double force = (lmbda + 0.5 * (1 - lmbda) * (1 + cosPhi)) * exp( 2 * radius - vecLen(xRel));

        feature[maxIdx(cosPhi, CONFIG.angles, 3)] += 1;

        // TODO - make this force tunable via parameter
        if (force > 0.5) {
            feature[maxIdx(cosPhi, CONFIG.angles, 3)] += 1;
        }
    }

    cell_feature[0] = feature[0];
    cell_feature[1] = feature[1];
    cell_feature[2] = feature[2];

    return cell_feature;
}


// helbing feature (dizan's modification) - 3 element feature
std::array<float, 2> CFeatures_ICRA::computeDizanFeature(std::vector<Ped::Tagent *> agents,
                                          Ped::Tagent *robot, double cx, double cy,
                                          double range)
{
    std::array<float, 2> df; df.fill(0.0);
    float lmbda = 0.2;
    double radius = 0.45;

    for (std::vector<Ped::Tagent*>::const_iterator iter = agents.begin(); iter != agents.end(); ++iter)
    {
        Ped::Tagent *a = (*iter);
        if (a->gettype() == robot->gettype()) continue;

        Ped::Tvector robot_position(robot->getx()*(1/20.0), robot->gety()*(1/20.0), 0.0);
        Ped::Tvector agent_position(a->getx()*(1/20.0), a->gety()*(1/20.0), 0.0);

        Ped::Tvector robot_velocity(robot->getvx(), robot->getvy(), 0);
        Ped::Tvector agent_velocity(a->getvx(), a->getvy(), 0);

        Ped::Tvector xRel = vecSub(agent_position, robot_position);
        Ped::Tvector vRel = vecSub(agent_velocity, robot_velocity);

        Ped::Tvector n = xRel; n.normalize();
        Ped::Tvector e = vRel; e.normalize();
        Ped::Tvector vOther(a->getvx(), a->getvy(), 0);
        vOther.normalize();

        double cosPhi1 = vecDot(Ped::Tvector(-n.x, -n.y, 0), e);
        double cosPhi2 = vecDot(Ped::Tvector(-n.x, -n.y, 0), vOther);

        double force1 = (1 + cosPhi1) * exp( 2 * radius - vecLen(xRel)) * vecLen(vRel);
        double force2 = (lmbda + 0.5 * (1 - lmbda) * (1 + cosPhi2)) * exp( 2 * radius - vecLen(xRel));

//        std::cout << robot->getvx() << " " << robot->getvy() << " | " << a->getvx() << " " << a->getvy() << std::endl;
//        std::cout << cosPhi1 << " " << cosPhi2 << " " << force1 << " " << force2 << " ";
//        std::cout << (force1 > 0.0 )<< " " << vecLen(vRel) << " " << exp( 2 * radius - vecLen(xRel)) << " ";
//        std::cout << std::endl;

        df[0] += force1;
        df[1] += force2;
    }

    return df;
}



