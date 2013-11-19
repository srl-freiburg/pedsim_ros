
#include <iomanip>
#include "policy.h"



std::vector<double> vectorizePolicy(const Policy& p)
{
    std::vector<double> vp;
    vp.push_back(p.density.high);
    vp.push_back(p.density.medium);
    vp.push_back(p.density.low);

    vp.push_back(p.vel_mag.high);
    vp.push_back(p.vel_mag.medium);
    vp.push_back(p.vel_mag.low);

    vp.push_back(p.vel_dir.high);
    vp.push_back(p.vel_dir.medium);
    vp.push_back(p.vel_dir.low);

    vp.push_back(p.dist_goal.high);
    vp.push_back(p.dist_goal.medium);
    vp.push_back(p.dist_goal.low);

    return vp;
}


Policy readPolicyFromFile(std::string filename)
{
    Policy np;

    boost::property_tree::ptree pt;
    boost::property_tree::ini_parser::read_ini(filename, pt);

    np.density.high = pt.get<double>("Density.High");
    np.density.medium = pt.get<double>("Density.Medium");
    np.density.low = pt.get<double>("Density.Low");

    np.vel_mag.high = pt.get<double>("VelMag.High");
    np.vel_mag.medium = pt.get<double>("VelMag.Medium");
    np.vel_mag.low = pt.get<double>("VelMag.Low");

    np.vel_dir.high = pt.get<double>("VelDir.High");
    np.vel_dir.medium = pt.get<double>("VelDir.Medium");
    np.vel_dir.low = pt.get<double>("VelDir.Low");

    np.dist_goal.high = pt.get<double>("DistGoal.High");
    np.dist_goal.medium = pt.get<double>("DistGoal.Medium");
    np.dist_goal.low = pt.get<double>("DistGoal.Low");

    return np;
}


std::ostream& operator<<(std::ostream& output, const Policy& p)
{
    output << "Navigation Policy" << std::endl;

    output << std::setw(10) << " " << std::setw(10) << "Density" << std::setw(10) << "VelMag" << std::setw(10) << "VelDir" << std::setw(10) << "DistGoal" << std::endl;

    output << std::setw(10) << "High" << std::setw(10) << p.density.high << std::setw(10) << p.vel_mag.high << std::setw(10) << p.vel_dir.high << std::setw(10) << p.dist_goal.high << std::endl;

    output << std::setw(10) << "Medium" << std::setw(10) << p.density.medium << std::setw(10) << p.vel_mag.medium << std::setw(10) << p.vel_dir.medium << std::setw(10) << p.dist_goal.medium << std::endl;

    output << std::setw(10) << "Low" << std::setw(10) << p.density.low << std::setw(10) << p.vel_mag.low << std::setw(10) << p.vel_dir.low << std::setw(10) << p.dist_goal.low << std::endl;


    return output;
}


std::vector<BinaryCF > binarizeCellFeatures(const std::vector<CFeature>& vcf)
{
//    assert(vcf.size() > 0);

    std::vector<BinaryCF > vbf;

    CFeature cf0 = vcf[0];
    double max_den = cf0.density; double min_den = cf0.density;
    double max_vmag = cf0.vel_mag; double min_vmag = cf0.vel_mag;
    double max_vdir = cf0.vel_dir; double min_vdir = cf0.vel_dir;
    double max_dgoal = cf0.dist_goal; double min_dgoal = cf0.dist_goal;

    for (size_t i = 1; i < vcf.size(); i++)
    {
        CFeature cf = vcf[i];
        if (cf.density > max_den) max_den = cf.density;
        if (cf.vel_mag > max_vmag) max_vmag = cf.vel_mag;
        if (cf.vel_dir > max_vdir) max_vdir = cf.vel_dir;
        if (cf.dist_goal > max_dgoal) max_dgoal = cf.dist_goal;

        if (cf.density < min_den) min_den = cf.density;
        if (cf.vel_mag < min_vmag) min_vmag = cf.vel_mag;
        if (cf.vel_dir < min_vdir) min_vdir = cf.vel_dir;
        if (cf.dist_goal < min_dgoal) min_dgoal = cf.dist_goal;
    }


    // compute ratios
    for (size_t i = 0; i < vcf.size(); i++)
    {
        CFeature cf = vcf[i];
        BinaryCF bcf; bcf.resize(12);

        double ratio_den = scaleRatio(cf.density, max_den, min_den);
        double ratio_vmag = scaleRatio(cf.vel_mag, max_vmag, min_vmag);
        double ratio_vdir = scaleRatio(cf.vel_dir, max_vdir, min_vdir);
        double ratio_dgoal = scaleRatio(cf.dist_goal, max_dgoal, min_dgoal);

        // density
        if (ratio_den < 0.33) { bcf[0] = 0; bcf[1] = 0; bcf[2] = 1; }
        if (ratio_den > 0.66) { bcf[0] = 1; bcf[1] = 0; bcf[2] = 0; }
        if (ratio_den < 0.66 && ratio_den > 0.33) { bcf[0] = 0; bcf[1] = 1; bcf[2] = 0; }

        // velmag
        if (ratio_vmag < 0.33) { bcf[3] = 0; bcf[4] = 0; bcf[5] = 1; }
        if (ratio_vmag > 0.66) { bcf[3] = 1; bcf[4] = 0; bcf[5] = 0; }
        if (ratio_vmag < 0.66 && ratio_vmag > 0.33) { bcf[3] = 0; bcf[4] = 1; bcf[5] = 0; }

        // veldir
        if (ratio_vdir < 0.33) { bcf[6] = 0; bcf[7] = 0; bcf[8] = 1; }
        if (ratio_vdir > 0.66) { bcf[6] = 1; bcf[7] = 0; bcf[8] = 0; }
        if (ratio_vdir < 0.66 && ratio_vdir > 0.33) { bcf[6] = 0; bcf[7] = 1; bcf[8] = 0; }

        // distgoal
        if (ratio_dgoal < 0.33) { bcf[9] = 0; bcf[10] = 0; bcf[11] = 1; }
        if (ratio_dgoal > 0.66) { bcf[9] = 1; bcf[10] = 0; bcf[11] = 0; }
        if (ratio_dgoal < 0.66 && ratio_dgoal > 0.33) { bcf[9] = 0; bcf[10] = 1; bcf[11] = 0; }

        vbf.push_back(bcf);
    }

    return vbf;
}


double scaleRatio(double x, double max, double min)
{
    double eps = 0.00000001; // to avoid division by zero
    return ((max-min) < eps) ? 0.0 : ((x-min) / (max-min));
}
