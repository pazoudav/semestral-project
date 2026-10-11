#include <skeleton_estimator/extra_del.hpp>

#include <limits>

namespace skeleton_estimator
{

/* Extracts the rows listed in `ind` (0-based indices, one per row) into a new matrix, in that order. */
Eigen::MatrixXd ExtraDel::rowsExtM(Eigen::MatrixXd ind, Eigen::MatrixXd matrix)
{
  Eigen::MatrixXd final_matrix(ind.size(), matrix.cols());
  int             num = ind.size();
  for (int k = 0; k < num; k++) {
    final_matrix.row(k) = matrix.row(ind(k, 0));
  }
  return final_matrix;
}

/* Returns `matrix` with the rows listed in `ind` removed, preserving the relative order of the rest. */
Eigen::MatrixXd ExtraDel::rowsDelM(Eigen::MatrixXd ind, Eigen::MatrixXd matrix)
{
  int             num = matrix.rows();
  Eigen::VectorXd xl(num);
  for (int i = 0; i < num; i++) {
    xl(i) = i;
  }
  for (int i = 0; i < ind.size(); i++) {
    xl.coeffRef(ind(i)) = std::numeric_limits<double>::quiet_NaN();
  }
  Eigen::VectorXd out_index(num - ind.size());
  int             index(0);
  for (int i = 0; i < num; i++) {
    if (std::isnan(xl(i))) {
      continue;
    } else {
      out_index(index) = i;
    }
    index++;
  }
  Eigen::MatrixXd zs1(out_index.size(), 1);
  zs1 << (out_index.head(out_index.size())).cast<double>();
  Eigen::MatrixXd final_matrix(zs1.size(), matrix.cols());
  int             num1 = zs1.size();
  for (int k = 0; k < num1; k++) {
    final_matrix.row(k) = matrix.row(zs1(k, 0));
  }
  return final_matrix;
}

/* Returns `matrix` with the columns listed in `ind` removed, preserving the relative order of the rest. */
Eigen::MatrixXd ExtraDel::colsDelM(Eigen::MatrixXd ind, Eigen::MatrixXd matrix)
{
  int             num = matrix.rows();
  Eigen::VectorXd xl(num);
  for (int i = 0; i < num; i++) {
    xl(i) = i;
  }
  for (int i = 0; i < ind.size(); i++) {
    xl.coeffRef(ind(i)) = std::numeric_limits<double>::quiet_NaN();
  }
  Eigen::VectorXd out_index(num - ind.size());
  int             index(0);
  for (int i = 0; i < num; i++) {
    if (std::isnan(xl(i))) {
      continue;
    } else {
      out_index(index) = i;
    }
    index++;
  }
  Eigen::MatrixXd zs1(out_index.size(), 1);
  zs1 << (out_index.head(out_index.size())).cast<double>();
  Eigen::MatrixXd final_matrix(matrix.rows(), zs1.size());
  int             num1 = zs1.size();
  for (int k = 0; k < num1; k++) {
    final_matrix.col(k) = matrix.col(zs1(k, 0));
  }
  return final_matrix;
}

}  // namespace skeleton_estimator
