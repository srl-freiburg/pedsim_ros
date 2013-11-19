#ifndef TRAJECTORY_H
#define TRAJECTORY_H

#include <QObject>
#include <vector>

/// \brief simple point in a trajectory
typedef struct TrajectoryPoint {
    double x;
    double y;
    double z;

    TrajectoryPoint() { x = 0.0; y = 0.0; z = 0.0; }
    TrajectoryPoint(double xx, double yy) : x(xx), y(yy) { z = 0.0;}
    TrajectoryPoint(double xx, double yy, double zz) : x(xx), y(yy), z(zz) { }

    TrajectoryPoint& operator=(TrajectoryPoint const& copy)
    {
        x = copy.x;
        y = copy.y;
        z = copy.z;
        return *this;
    }

} Tpoint;


/// \class Trajectory
/// \brief A list of point for the robot agent to follow
class Trajectory : public QObject
{
    Q_OBJECT
public:
    explicit Trajectory(QObject *parent = 0);
    ~Trajectory() { path_.clear(); }

    void addPointEnd(Tpoint p);
    void addPointBegin(Tpoint p);
    size_t length() {return path_.size(); }
    void reset() { path_.clear(); }
    std::vector<Tpoint> getPath() {return path_;}

    Tpoint getNextPoint(Tpoint p);

private:
    std::vector<Tpoint> path_;
    std::vector<Tpoint> drive_path_;


};

#endif // TRAJECTORY_H
