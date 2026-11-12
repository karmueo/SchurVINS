/*
 *  This file can be used to test the ceres based pose graph optimisation given
 * a g20 file of noisy vertices and constraints.
 *  This code is partly adapted from the examples in ceres github repository.
 *
 *  Created on: Nov 22, 2018
 *      Author: kunal71091
 */

#include "svo/pgo/pgo.h"

#include <fstream>
#include <string>
#include <iomanip>
#include <stdexcept>

using namespace svo;

bool OutputPoses(const std::string& filename, const ceres::MapOfPoses& poses)
{
  std::fstream outfile;
  outfile.open(filename.c_str(), std::istream::out);
  if (!outfile)
  {
    LOG(ERROR) << "Error opening the file: " << filename;
    return false;
  }
  outfile << std::setprecision(17);
  for (std::map<int, ceres::Pose3d, std::less<int>,
                Eigen::aligned_allocator<
                    std::pair<const int, ceres::Pose3d> > >::const_iterator
           poses_iter = poses.begin();
       poses_iter != poses.end(); ++poses_iter)
  {
    const std::map<int, ceres::Pose3d, std::less<int>,
                   Eigen::aligned_allocator<
                       std::pair<const int, ceres::Pose3d> > >::value_type&
        pair = *poses_iter;
    outfile << pair.first << " " << pair.second.p.transpose() << " "
            << pair.second.q.x() << " " << pair.second.q.y() << " "
            << pair.second.q.z() << " " << pair.second.q.w() << '\n';
  }
  return true;
}

int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);
  if (argc != 3)
  {
    std::cerr << "Usage: test_pgo input.g2o output.txt\n";
    return 2;
  }
  try
  {
  Pgo pgo_;
  std::ifstream infile(argv[1]);
  if (!infile) throw std::runtime_error("Cannot open input g2o file");
  size_t poses = 0, constraints = 0;

  std::string data_type;
  while (infile >> data_type)
  {
    if (data_type.front() == '#') { std::string comment; std::getline(infile, comment); continue; }
    if (data_type == ceres::Pose3d::name())
    {
      int id;
      infile >> id;
      kindr::minimal::Position p;
      Eigen::Quaternion<double> q_eigen;
      Transformation t;
      infile >> p.x() >> p.y() >> p.z() >> q_eigen.x() >> q_eigen.y() >>
          q_eigen.z() >> q_eigen.w();
      if (!infile || !p.allFinite() || !q_eigen.coeffs().allFinite() || q_eigen.norm() < 1e-12)
        throw std::runtime_error("Invalid g2o pose");
      q_eigen.normalize();
      Quaternion q(q_eigen);
      t = Transformation(q, p);
      pgo_.addPoseToPgoProblem(t, id); ++poses;
    }
    else if (data_type == ceres::Constraint3d::name())
    {
      int id_b, id_e;
      kindr::minimal::Position p;
      Eigen::Quaternion<double> q_eigen;
      Transformation t_be;
      Eigen::Matrix<double, 6, 6> info;
      infile >> id_b >> id_e;
      infile >> p.x() >> p.y() >> p.z() >> q_eigen.x() >> q_eigen.y() >>
          q_eigen.z() >> q_eigen.w();
      if (!infile || !p.allFinite() || !q_eigen.coeffs().allFinite() || q_eigen.norm() < 1e-12)
        throw std::runtime_error("Invalid g2o pose");
      q_eigen.normalize();
      Quaternion q(q_eigen);
      t_be = Transformation(q, p);
      for (int i = 0; i < 6 && infile.good(); ++i)
      {
        for (int j = i; j < 6 && infile.good(); ++j)
        {
          infile >> info(i, j);
          if (i != j)
          {
            info(j, i) = info(i, j);
          }
        }
      }
      if (!infile || !info.allFinite()) throw std::runtime_error("Invalid g2o information matrix");
      pgo_.addConstraint(t_be, id_b, id_e, info); ++constraints;
    }
    else throw std::runtime_error("Unsupported g2o record: " + data_type);
  }
  if (poses < 2 || constraints == 0) throw std::runtime_error("The graph needs poses and constraints");

  std::cout << "Beginning to Solve" << std::endl;
  pgo_.solve();

  if (!OutputPoses(argv[2], *pgo_.poses_)) return 1;
  return 0;
  }
  catch (const std::exception& error)
  {
    std::cerr << "Pose graph optimization failed: " << error.what() << '\n';
    return 1;
  }
}
