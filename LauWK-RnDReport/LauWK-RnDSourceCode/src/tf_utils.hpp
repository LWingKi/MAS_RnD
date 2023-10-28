#ifndef TF_UTILS_HPP
#define TF_UTILS_HPP

/**
 * @brief transformation utils to convert kdl frames between different coordinate systems
 */

// #include "arm_actions/logger.hpp"
#include "kdl/chain.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/frames.hpp"
#include "kdl/jntarray.hpp"

/**
 * @brief enum for different coordinate systems
 */
enum class CoordinateSystem
{
  BASE,
  EE
};

class TfUtils
{
public:
  TfUtils();

  ~TfUtils();

  /**
   * @brief set the chain for the forward kinematics solver
   * @param chain A pointer to a KDL::Chain object representing the robot's kinematic chain.
   */
  void setChain(KDL::Chain *chain);

  /**
   * @brief Converts a KDL::Frame from EE frame to base frame.
   * @param source_frame The KDL::Frame to convert.
   * @param q A pointer to a KDL::JntArray object representing the robot's joint positions.
   * @param source_cs enum for the source coordinate system
   * @param target_cs enum for the target coordinate system
   * @param segment_nr The segment number of the desired link. If not specified, the last segment of the chain is used.
   */
  void transformFrame(KDL::Frame &source_frame, KDL::JntArray *q,
                    CoordinateSystem source_cs, CoordinateSystem target_cs, int segment_nr = -1);

  /**
   * @brief Converts a KDL::Jacobian from EE frame to base frame.
   * @param source_jacobian The KDL::Jacobian to convert.
   * @param q A pointer to a KDL::JntArray object representing the robot's joint positions.
   * @param source_cs enum for the source coordinate system
   * @param target_cs enum for the target coordinate system
   * @param segment_nr The segment number of the desired link. If not specified, the last segment of the chain is used.
   */
  void transformJacobian(KDL::Jacobian &source_jacobian, KDL::JntArray *q,
                       CoordinateSystem source_cs, CoordinateSystem target_cs, int segment_nr = -1);

  /**
   * @brief Converts a KDL::JntArray from EE frame to base frame.
   * @param source_jnt_array The KDL::JntArray to convert.
   * @param q A pointer to a KDL::JntArray object representing the robot's joint positions.
   * @param source_cs enum for the source coordinate system
   * @param target_cs enum for the target coordinate system
   * @param segment_nr The segment number of the desired link. If not specified, the last segment of the chain is used.
   */
  void transformJntArray(KDL::JntArray &source_jnt_array, KDL::JntArray *q,
                       CoordinateSystem source_cs, CoordinateSystem target_cs, int segment_nr = -1);

private:
  // std::shared_ptr<Logger> _logger;
  KDL::ChainFkSolverPos_recursive* _fksolver;
};

#endif  // TF_UTILS_HPP