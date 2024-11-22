#ifndef STATE_H
#define STATE_H

#include <string>
#include <sstream>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <Eigen/Dense>
#include <cmath>

namespace droneState{
    // Co-ordinate system enum
    enum class CS_type {
        LLA, // Planetary co-ordinate system: Latitude, Longitude, Altitude.
        ENU, // Local tangent plane body CS: East, North, Up
        NED,  // Local tangent plane body CS: North, East, Down
        XYZ // (world/body) General XYZ coordinates where orientation must be carfully defined (Gazebo default)
    };

    // Class to store robot state
    class State {
    public:
        State(std::string frame, CS_type cs_type, Eigen::Vector3d pos = Eigen::Vector3d(0.0, 0.0, 0.0), tf2::Quaternion att = tf2::Quaternion(0.0, 0.0, 0.0, 1.0), Eigen::Vector3d vel = Eigen::Vector3d(0.0, 0.0, 0.0));
        State() : State("default_frame", CS_type::ENU) {}; // Default constructor that delegates to the main constructor

        bool operator==(const State& other) const;
        State operator-(const State &other) const;

        State copy() const;
        std::string to_string() const;

        float distTrans(const State &other) const;
        float distAngGeo(const State &other) const;
        

        // Getters
        std::string getFrame() const { return frame; }
        CS_type getCsType() const { return cs_type; }
        Eigen::Vector3d getPos() const { return pos; }
        tf2::Quaternion getAtt() const { return att; }
        void getAttYPR(double& yaw, double& pitch, double& roll) { // Return YPR around ZYX axes
            tf2::Matrix3x3 m(att);
            m.getEulerYPR(yaw, pitch, roll);
        }
        Eigen::Vector3d getAttYPR() const{ // Return YPR around ZYX axes
            tf2::Matrix3x3 m(att);
            double yaw, pitch, roll;
            m.getEulerYPR(yaw, pitch, roll);

            return Eigen::Vector3d(roll, pitch, yaw);
        }

        Eigen::Vector3d getVel() const { return vel; }

        // Setters
        void setFrame(const std::string& newFrame) { frame = newFrame; }
        void setCsType(CS_type newCsType) { cs_type = newCsType; }
        void setPos(const Eigen::Vector3d& newPos) { pos = newPos; }
        void setAtt(const tf2::Quaternion& newAtt) { att = newAtt; }
        void setVel(const Eigen::Vector3d& newVel) { vel = newVel; }

    private:
        std::string frame;
        CS_type cs_type;
        Eigen::Vector3d pos;
        tf2::Quaternion att;
        Eigen::Vector3d vel;
    };
}

#endif // STATE_H