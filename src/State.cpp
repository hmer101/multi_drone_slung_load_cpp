#include "multi_drone_slung_load_cpp/State.h"
#include "multi_drone_slung_load_cpp/utils.h"
#include <cmath>

namespace droneState{
    // Constructor implementation
    State::State(std::string frame, CS_type cs_type, Eigen::Vector3d pos, tf2::Quaternion att, Eigen::Vector3d vel, Eigen::Vector3d acc)
        : frame(frame), cs_type(cs_type), pos(pos), att(att), vel(vel), acc(acc) {}

    // Operator == implementation
    bool State::operator==(const State& other) const {
        return frame == other.frame && cs_type == other.cs_type &&
            pos.isApprox(other.pos) && att == other.att &&
            vel.isApprox(other.vel) && acc.isApprox(other.acc);
    }

    // Operator - implementation. 
    State State::operator-(const State& other) const {
        // Note that subtractions can only be performed on states with the same frame and CS type
        if (frame != other.frame || cs_type != other.cs_type) {
            throw std::invalid_argument("Subtraction can only be performed on states with the same frame and CS type");
        }else {
            return State(frame, cs_type, pos - other.pos, other.att*att.inverse(), vel - other.vel, acc - other.acc); // TODO: Check relative att
        }
        
    }

    // Copy method implementation
    State State::copy() const {
        return State(frame, cs_type, pos, att, vel, acc);
    }

    // to_string method implementation
    std::string State::to_string() const {
        std::ostringstream ss;
        ss << "Frame: " << frame << ", CS type: " << static_cast<int>(cs_type)
        << ", pos: [" << pos[0] << ", " << pos[1] << ", " << pos[2] << "]"
        << ", att_q: [" << att.w() << ", " << att.x() << ", " << att.y() << ", " << att.z() << "]"
        << ", vel: [" << vel[0] << ", " << vel[1] << ", " << vel[2] << "]"
        << ", acc: [" << acc[0] << ", " << acc[1] << ", " << acc[2] << "]";
        return ss.str();
    }

    // Calculate the translation distance of this state to another state
    float State::distTrans(const State& other) const {
        Eigen::Vector3d t = other.getPos() - pos;
        return t.norm();
    }

    // Calculate the geodesic distance of this state to another state
    float State::distAngGeo(const State& other) const {
        // Convert quaternions to rotation matrices
        tf2::Matrix3x3 R1(this->att);
        tf2::Matrix3x3 R2(other.getAtt());

        // Compute the relative rotation matrix
        tf2::Matrix3x3 relativeRotationMatrix = R1.inverse() * R2;

        // Convert to axis-angle and take magnitude of angle
        float dist_ang = std::fabs(std::acos((utils::getTrace(relativeRotationMatrix) - 1.0)/2.0));

        // The angle is the geodesic distance
        return dist_ang;
    }
} // namespace droneState