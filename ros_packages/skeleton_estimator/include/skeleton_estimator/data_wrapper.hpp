#pragma once

/* Ported from FC-Planner's rosa/include/rosa/datawrapper.h (predrecon::DataWrapper),
 * unchanged aside from the namespace. Thin wrapper over a flattened xyz point buffer
 * (column-major: x's, then y's, then z's), used by Rosa::pcloudIsOnCut/distanceQuery. */

#include <cassert>
#include <vector>

namespace skeleton_estimator
{

class DataWrapper
{
public:
  void factory(const double* data, int npoints)
  {
    data_    = data;
    npoints_ = npoints;
  }

  inline double operator()(int a, int b)
  {
    assert(a < npoints_);
    assert(b < ndim_);
    return data_[a + npoints_ * b];
  }

  inline void operator()(int a, std::vector<double>& p)
  {
    assert(a < npoints_);
    assert((int)p.size() == ndim_);
    p[0] = data_[a + 0 * npoints_];
    p[1] = data_[a + 1 * npoints_];
    p[2] = data_[a + 2 * npoints_];
  }

  int length()
  {
    return npoints_;
  }

private:
  const double*    data_    = nullptr;
  int              npoints_ = 0;
  const static int ndim_    = 3;
};

}  // namespace skeleton_estimator
