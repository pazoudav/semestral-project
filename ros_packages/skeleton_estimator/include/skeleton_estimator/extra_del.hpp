#pragma once

/* Ported from FC-Planner's rosa/include/rosa/Extra_Del.h (predrecon::Extra_Del),
 * trimmed to the methods actually called from Rosa's reachable pipeline
 * (rowsExtV/colsExtV/colsExtM were never used by ROSA_main::pointCloudCallback's
 * call graph). Pure Eigen row/column extraction and deletion by index list. */

#include <Eigen/Eigen>

namespace skeleton_estimator
{

class ExtraDel
{
public:
  Eigen::MatrixXd rowsExtM(Eigen::MatrixXd ind, Eigen::MatrixXd matrix);
  Eigen::MatrixXd rowsDelM(Eigen::MatrixXd ind, Eigen::MatrixXd matrix);
  Eigen::MatrixXd colsDelM(Eigen::MatrixXd ind, Eigen::MatrixXd matrix);
};

}  // namespace skeleton_estimator
