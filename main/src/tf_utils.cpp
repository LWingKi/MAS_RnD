#include "tf_utils.hpp"

TfUtils::TfUtils() : _fksolver(nullptr){}

TfUtils::~TfUtils() 
{
  if (_fksolver) {
    delete _fksolver;
  }
}

void TfUtils::setChain(KDL::Chain *chain)
{
  if (_fksolver) {
    delete _fksolver; // Clean up previous solver if it exists
  }

  // Allocate a new solver
  _fksolver = new KDL::ChainFkSolverPos_recursive(*chain);
}

void TfUtils::transformFrame(KDL::Frame &source_frame, KDL::JntArray *q,
                           CoordinateSystem source_cs, CoordinateSystem target_cs, int segment_nr)
{
  KDL::Frame target_frame;
  switch (source_cs)
  {
    case CoordinateSystem::BASE:
      switch (target_cs)
      {
        case CoordinateSystem::BASE:
          break;
        case CoordinateSystem::EE:
          // _logger->logInfo("Converting from base to ee frame");
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          source_frame = source_frame * target_frame.Inverse();
          break;
      }
      break;
    case CoordinateSystem::EE:
      switch (target_cs)
      {
        case CoordinateSystem::BASE:
          // _logger->logInfo("Converting from ee to base frame");
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          source_frame = source_frame * target_frame;
          break;
        case CoordinateSystem::EE:
          break;
      }
      break;
  }
}

void TfUtils::transformJacobian(KDL::Jacobian &source_jacobian, KDL::JntArray *q,
                       CoordinateSystem source_cs, CoordinateSystem target_cs, int segment_nr)
{
  KDL::Frame target_frame;
  switch (source_cs)
  {
    case CoordinateSystem::BASE:
      switch (target_cs)
      {
        case CoordinateSystem::BASE:
          break;
        case CoordinateSystem::EE:
          // _logger->logInfo("Converting from base to ee frame");
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          for (int i = 0; i < source_jacobian.columns(); i++)
          {
            source_jacobian.setColumn(i, target_frame.Inverse() * source_jacobian.getColumn(i));
          }
          break;
      }
      break;
    case CoordinateSystem::EE:
      switch (target_cs)
      {
        case CoordinateSystem::BASE:
          // _logger->logInfo("Converting from ee to base frame");
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          for (int i = 0; i < source_jacobian.columns(); i++)
          {
            source_jacobian.setColumn(i, target_frame * source_jacobian.getColumn(i));
          }
          break;
        case CoordinateSystem::EE:
          break;
      }
      break;
  }
}

void TfUtils::transformJntArray(KDL::JntArray &source_jnt_array, KDL::JntArray *q,
                       CoordinateSystem source_cs, CoordinateSystem target_cs, int segment_nr)
{
  KDL::Frame target_frame;
  KDL::Vector source_jnt_array_vector;
  switch (source_cs)
  {
    case CoordinateSystem::BASE:
      switch (target_cs)
      {
        case CoordinateSystem::BASE:
          break;
        case CoordinateSystem::EE:
          // _logger->logInfo("Converting from base to ee frame");
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          // multiply the source_jnt_array with the inverse of the target_frame
          // convert the jnt_array to a vector
          
          for (int i = 0; i < source_jnt_array.rows(); i++)
          {
            source_jnt_array_vector(i) = source_jnt_array(i);
          }
          // multiply the vector with the inverse of the target_frame
          source_jnt_array_vector = target_frame.Inverse() * source_jnt_array_vector;
          // convert the vector back to a jnt_array
          for (int i = 0; i < source_jnt_array.rows(); i++)
          {
            source_jnt_array(i) = source_jnt_array_vector(i);
          }
          break;
      }
      break;
    case CoordinateSystem::EE:
      switch (target_cs)
      {
        case CoordinateSystem::BASE:
          // _logger->logInfo("Converting from ee to base frame");
          _fksolver->JntToCart(*q, target_frame, segment_nr);
          // multiply the source_jnt_array with the target_frame
          // convert the jnt_array to a vector
          for (int i = 0; i < source_jnt_array.rows(); i++)
          {
            source_jnt_array_vector(i) = source_jnt_array(i);
          }
          // multiply the vector with the target_frame
          source_jnt_array_vector = target_frame * source_jnt_array_vector;
          // convert the vector back to a jnt_array
          for (int i = 0; i < source_jnt_array.rows(); i++)
          {
            source_jnt_array(i) = source_jnt_array_vector(i);
          }
        case CoordinateSystem::EE:
          break;
      }
      break;
  }
}