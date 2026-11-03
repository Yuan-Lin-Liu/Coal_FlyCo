/*⭐⭐⭐******************************************************************⭐⭐⭐*
 * Authors      :    Chen Feng <cfengag at connect dot ust dot hk>, UAV Group, ECE, HKUST.
 *                   Guiyong Zheng <shverses at gmail dot com>, SAI, SYSU.
 * Homepage     :    https://chen-albert-feng.github.io/AlbertFeng.github.io/
 *                   https://gy920.github.io/
 * Date         :    Apr. 2024
 * E-mail       :    cfengag at connect dot ust dot hk
 *                   shverses at gmail dot com
 * Description  :    This file is the main algorithm of classical Ray Casting and the proposed
 *                   Bidirectional Ray Casting (BiRC) in FlyCo.
 * Copyright    :    Copyright (c) 2026 Chen Feng and Guiyong Zheng.
 * License      :    PolyForm Noncommercial License 1.0.0
 *                   <https://polyformproject.org/licenses/noncommercial/1.0.0/>
 *
 *                   This software is released for noncommercial research and
 *                   educational use only. You may use, modify, and distribute
 *                   this software for noncommercial purposes, subject to the
 *                   terms of the PolyForm Noncommercial License 1.0.0.
 *
 *                   Commercial use, including use in commercial products,
 *                   commercial services, paid consulting, or internal business
 *                   operations, is prohibited without prior written permission
 *                   from the copyright holders.
 *
 *                   This software is provided "as is", without warranty of any
 *                   kind, express or implied.
 * Project      :    FlyCo: Foundation Model-Empowered Drones for Autonomous 3D Structure Scanning in Open-World Environments
 * Website      :    https://hkust-aerial-robotics.github.io/FC-Planner/
 *⭐⭐⭐*****************************************************************⭐⭐⭐*/

#include <Eigen/Eigen>
#include <cmath>
#include <limits>
#include <iostream>
#include <plan_env/raycast.h>

int signum(int x) {
  return x == 0 ? 0 : x < 0 ? -1 : 1;
}

double mod(double value, double modulus) {
  return fmod(fmod(value, modulus) + modulus, modulus);
}

double intbound(double s, double ds) {
  if (ds == 0.0) return std::numeric_limits<double>::infinity();
  // Negative rays on a cell face cross it immediately, not one voxel later.
  if (ds < 0.0) return (s - std::floor(s)) / -ds;
  return (std::floor(s) + 1.0 - s) / ds;
}

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, int& output_points_cnt, Eigen::Vector3d* output) {
  // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
  // by John Amanatides and Andrew Woo, 1987
  // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
  // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
  // Extensions to the described algorithm:
  //   • Imposed a distance limit.
  //   • The face passed through to reach the current cube is provided to
  //     the callback.

  // The foundation of this algorithm is a parameterized representation of
  // the provided ray,
  //                    origin + t * direction,
  // except that t is not actually stored; rather, at any given point in the
  // traversal, we keep track of the *greater* t values which we would have
  // if we took a step sufficient to cross a cube boundary along that axis
  // (i.e. change the integer part of the coordinate) in the variables
  // tMaxX, tMaxY, and tMaxZ.

  // Cube containing origin point.
  int x = (int)std::floor(start.x());
  int y = (int)std::floor(start.y());
  int z = (int)std::floor(start.z());
  int endX = (int)std::floor(end.x());
  int endY = (int)std::floor(end.y());
  int endZ = (int)std::floor(end.z());
  Eigen::Vector3d direction = (end - start);
  double maxDist = direction.squaredNorm();

  // Break out direction vector.
  double dx = endX - x;
  double dy = endY - y;
  double dz = endZ - z;

  // Direction to increment x,y,z when stepping.
  int stepX = (int)signum((int)dx);
  int stepY = (int)signum((int)dy);
  int stepZ = (int)signum((int)dz);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  double tMaxX = intbound(start.x(), dx);
  double tMaxY = intbound(start.y(), dy);
  double tMaxZ = intbound(start.z(), dz);

  // The change in t when taking a step (always positive).
  double tDeltaX = ((double)stepX) / dx;
  double tDeltaY = ((double)stepY) / dy;
  double tDeltaZ = ((double)stepZ) / dz;

  // Avoids an infinite loop.
  if (stepX == 0 && stepY == 0 && stepZ == 0) return;

  double dist = 0;
  while (true) {
    if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z()) {
      output[output_points_cnt](0) = x;
      output[output_points_cnt](1) = y;
      output[output_points_cnt](2) = z;

      output_points_cnt++;
      dist = sqrt((x - start(0)) * (x - start(0)) + (y - start(1)) * (y - start(1)) +
                  (z - start(2)) * (z - start(2)));

      if (dist > maxDist) return;

      /*            if (output_points_cnt > 1500) {
                      std::cerr << "Error, too many racyast voxels." <<
         std::endl;
                      throw std::out_of_range("Too many raycast voxels");
                  }*/
    }

    if (x == endX && y == endY && z == endZ) break;

    // tMaxX stores the t-value at which we cross a cube boundary along the
    // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
    // chooses the closest cube boundary. Only the first case of the four
    // has been commented in detail.
    if (tMaxX < tMaxY) {
      if (tMaxX < tMaxZ) {
        // Update which cube we are now in.
        x += stepX;
        // Adjust tMaxX to the next X-oriented boundary crossing.
        tMaxX += tDeltaX;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      if (tMaxY < tMaxZ) {
        y += stepY;
        tMaxY += tDeltaY;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
  }
}

void Raycast(const Eigen::Vector3d& start, const Eigen::Vector3d& end, const Eigen::Vector3d& min,
             const Eigen::Vector3d& max, std::vector<Eigen::Vector3d>* output) {
  // From "A Fast Voxel Traversal Algorithm for Ray Tracing"
  // by John Amanatides and Andrew Woo, 1987
  // <http://www.cse.yorku.ca/~amana/research/grid.pdf>
  // <http://citeseer.ist.psu.edu/viewdoc/summary?doi=10.1.1.42.3443>
  // Extensions to the described algorithm:
  //   • Imposed a distance limit.
  //   • The face passed through to reach the current cube is provided to
  //     the callback.

  // The foundation of this algorithm is a parameterized representation of
  // the provided ray,
  //                    origin + t * direction,
  // except that t is not actually stored; rather, at any given point in the
  // traversal, we keep track of the *greater* t values which we would have
  // if we took a step sufficient to cross a cube boundary along that axis
  // (i.e. change the integer part of the coordinate) in the variables
  // tMaxX, tMaxY, and tMaxZ.

  // Cube containing origin point.
  int x = (int)std::floor(start.x());
  int y = (int)std::floor(start.y());
  int z = (int)std::floor(start.z());
  int endX = (int)std::floor(end.x());
  int endY = (int)std::floor(end.y());
  int endZ = (int)std::floor(end.z());
  Eigen::Vector3d direction = (end - start);
  double maxDist = direction.squaredNorm();

  // Break out direction vector.
  double dx = endX - x;
  double dy = endY - y;
  double dz = endZ - z;

  // Direction to increment x,y,z when stepping.
  int stepX = (int)signum((int)dx);
  int stepY = (int)signum((int)dy);
  int stepZ = (int)signum((int)dz);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  double tMaxX = intbound(start.x(), dx);
  double tMaxY = intbound(start.y(), dy);
  double tMaxZ = intbound(start.z(), dz);

  // The change in t when taking a step (always positive).
  double tDeltaX = ((double)stepX) / dx;
  double tDeltaY = ((double)stepY) / dy;
  double tDeltaZ = ((double)stepZ) / dz;

  output->clear();

  // Avoids an infinite loop.
  if (stepX == 0 && stepY == 0 && stepZ == 0) return;

  double dist = 0;
  while (true) {
    if (x >= min.x() && x < max.x() && y >= min.y() && y < max.y() && z >= min.z() && z < max.z()) {
      output->push_back(Eigen::Vector3d(x, y, z));

      dist = (Eigen::Vector3d(x, y, z) - start).squaredNorm();

      if (dist > maxDist) return;

      if (output->size() > 1500) {
        std::cerr << "Error, too many racyast voxels." << std::endl;
        throw std::out_of_range("Too many raycast voxels");
      }
    }

    if (x == endX && y == endY && z == endZ) break;

    // tMaxX stores the t-value at which we cross a cube boundary along the
    // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
    // chooses the closest cube boundary. Only the first case of the four
    // has been commented in detail.
    if (tMaxX < tMaxY) {
      if (tMaxX < tMaxZ) {
        // Update which cube we are now in.
        x += stepX;
        // Adjust tMaxX to the next X-oriented boundary crossing.
        tMaxX += tDeltaX;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    } else {
      if (tMaxY < tMaxZ) {
        y += stepY;
        tMaxY += tDeltaY;
      } else {
        z += stepZ;
        tMaxZ += tDeltaZ;
      }
    }
  }
}

bool RayCaster::setInput(const Eigen::Vector3d& start,
                         const Eigen::Vector3d& end /* , const Eigen::Vector3d& min,
                         const Eigen::Vector3d& max */) {
  start_ = start;
  end_ = end;
  // max_ = max;
  // min_ = min;

  x_ = (int)std::floor(start_.x());
  y_ = (int)std::floor(start_.y());
  z_ = (int)std::floor(start_.z());
  endX_ = (int)std::floor(end_.x());
  endY_ = (int)std::floor(end_.y());
  endZ_ = (int)std::floor(end_.z());
  direction_ = (end_ - start_);
  maxDist_ = direction_.squaredNorm();

  // Break out direction vector.
  dx_ = endX_ - x_;
  dy_ = endY_ - y_;
  dz_ = endZ_ - z_;

  // Direction to increment x,y,z when stepping.
  stepX_ = (int)signum((int)dx_);
  stepY_ = (int)signum((int)dy_);
  stepZ_ = (int)signum((int)dz_);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  tMaxX_ = intbound(start_.x(), dx_);
  tMaxY_ = intbound(start_.y(), dy_);
  tMaxZ_ = intbound(start_.z(), dz_);

  // The change in t when taking a step (always positive).
  tDeltaX_ = ((double)stepX_) / dx_;
  tDeltaY_ = ((double)stepY_) / dy_;
  tDeltaZ_ = ((double)stepZ_) / dz_;

  dist_ = 0;

  step_num_ = 0;

  // Avoids an infinite loop.
  if (stepX_ == 0 && stepY_ == 0 && stepZ_ == 0)
    return false;
  else
    return true;
}

bool RayCaster::step(Eigen::Vector3d& ray_pt) {
  // if (x_ >= min_.x() && x_ < max_.x() && y_ >= min_.y() && y_ < max_.y() &&
  // z_ >= min_.z() && z_ <
  // max_.z())
  ray_pt = Eigen::Vector3d(x_, y_, z_);

  // step_num_++;

  // dist_ = (Eigen::Vector3d(x_, y_, z_) - start_).squaredNorm();

  if (x_ == endX_ && y_ == endY_ && z_ == endZ_) {
    return false;
  }

  // if (dist_ > maxDist_)
  // {
  //   return false;
  // }

  // tMaxX stores the t-value at which we cross a cube boundary along the
  // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
  // chooses the closest cube boundary. Only the first case of the four
  // has been commented in detail.
  if (tMaxX_ < tMaxY_) {
    if (tMaxX_ < tMaxZ_) {
      // Update which cube we are now in.
      x_ += stepX_;
      // Adjust tMaxX to the next X-oriented boundary crossing.
      tMaxX_ += tDeltaX_;
    } else {
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  } else {
    if (tMaxY_ < tMaxZ_) {
      y_ += stepY_;
      tMaxY_ += tDeltaY_;
    } else {
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  }

  return true;
}

void RayCaster::setParams(const double& res, const Eigen::Vector3d& origin) {
  resolution_ = res;
  half_ = Eigen::Vector3d(0.5, 0.5, 0.5);
  origin_ = origin;
}

bool RayCaster::input(const Eigen::Vector3d& start, const Eigen::Vector3d& end) {
  // Traverse the same grid as SDFMap::posToIndex. Applying a rounded origin
  // offset after world-grid traversal shifts cells for non-aligned origins.
  start_ = (start - origin_) / resolution_;
  end_ = (end - origin_) / resolution_;

  x_ = (int)std::floor(start_.x());
  y_ = (int)std::floor(start_.y());
  z_ = (int)std::floor(start_.z());
  endX_ = (int)std::floor(end_.x());
  endY_ = (int)std::floor(end_.y());
  endZ_ = (int)std::floor(end_.z());
  direction_ = (end_ - start_);
  maxDist_ = direction_.squaredNorm();

  // Break out direction vector.
  dx_ = direction_.x();
  dy_ = direction_.y();
  dz_ = direction_.z();

  // Direction to increment x,y,z when stepping.
  stepX_ = (dx_ > 0) - (dx_ < 0);
  stepY_ = (dy_ > 0) - (dy_ < 0);
  stepZ_ = (dz_ > 0) - (dz_ < 0);

  // See description above. The initial values depend on the fractional
  // part of the origin.
  tMaxX_ = intbound(start_.x(), dx_);
  tMaxY_ = intbound(start_.y(), dy_);
  tMaxZ_ = intbound(start_.z(), dz_);

  // The change in t when taking a step (always positive).
  tDeltaX_ = dx_ == 0.0 ? std::numeric_limits<double>::infinity() : std::abs(1.0 / dx_);
  tDeltaY_ = dy_ == 0.0 ? std::numeric_limits<double>::infinity() : std::abs(1.0 / dy_);
  tDeltaZ_ = dz_ == 0.0 ? std::numeric_limits<double>::infinity() : std::abs(1.0 / dz_);

  dist_ = 0;

  step_num_ = 0;

  // Avoids an infinite loop.
  if (x_ == endX_ && y_ == endY_ && z_ == endZ_)
    return false;
  else
    return true;
}

bool RayCaster::biInput(const Eigen::Vector3d& start, const Eigen::Vector3d& end)
{
  start_ = (start - origin_) / resolution_;
  inter_ = (0.5*(start+end) - origin_) / resolution_;
  end_ = (end - origin_) / resolution_;
  
  x_ = (int)std::floor(start_.x());
  y_ = (int)std::floor(start_.y());
  z_ = (int)std::floor(start_.z());
  interX_ = (int)std::floor(inter_.x());
  interY_ = (int)std::floor(inter_.y());
  interZ_ = (int)std::floor(inter_.z());
  endX_ = (int)std::floor(end_.x());
  endY_ = (int)std::floor(end_.y());
  endZ_ = (int)std::floor(end_.z());
  dir_p = (inter_ - start_);
  dir_n = (inter_ - end_);
  maxDp = dir_p.squaredNorm();
  maxDn = dir_n.squaredNorm();

  dx_p = dir_p.x(); dy_p = dir_p.y(); dz_p = dir_p.z();
  dx_n = dir_n.x(); dy_n = dir_n.y(); dz_n = dir_n.z();

  stepXp = (dx_p > 0) - (dx_p < 0);
  stepYp = (dy_p > 0) - (dy_p < 0);
  stepZp = (dz_p > 0) - (dz_p < 0);
  stepXn = (dx_n > 0) - (dx_n < 0);
  stepYn = (dy_n > 0) - (dy_n < 0);
  stepZn = (dz_n > 0) - (dz_n < 0);

  tMaxXp = intbound(start_.x(), dx_p);
  tMaxYp = intbound(start_.y(), dy_p);
  tMaxZp = intbound(start_.z(), dz_p);
  tMaxXn = intbound(end_.x(), dx_n);
  tMaxYn = intbound(end_.y(), dy_n);
  tMaxZn = intbound(end_.z(), dz_n);

  tDeltaXp = dx_p == 0.0 ? std::numeric_limits<double>::infinity() : std::abs(1.0 / dx_p);
  tDeltaYp = dy_p == 0.0 ? std::numeric_limits<double>::infinity() : std::abs(1.0 / dy_p);
  tDeltaZp = dz_p == 0.0 ? std::numeric_limits<double>::infinity() : std::abs(1.0 / dz_p);
  tDeltaXn = dx_n == 0.0 ? std::numeric_limits<double>::infinity() : std::abs(1.0 / dx_n);
  tDeltaYn = dy_n == 0.0 ? std::numeric_limits<double>::infinity() : std::abs(1.0 / dy_n);
  tDeltaZn = dz_n == 0.0 ? std::numeric_limits<double>::infinity() : std::abs(1.0 / dz_n);

  dist_ = 0;

  step_num_ = 0;

  if (stepXp == 0 && stepYp == 0 && stepZp == 0)
    return false;
  else
    return true;
}

bool RayCaster::nextId(Eigen::Vector3i& idx) {
  auto tmp = Eigen::Vector3d(x_, y_, z_);
  idx = tmp.cast<int>();

  if (x_ == endX_ && y_ == endY_ && z_ == endZ_) {
    return false;
  }

  // tMaxX stores the t-value at which we cross a cube boundary along the
  // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
  // chooses the closest cube boundary. Only the first case of the four
  // has been commented in detail.
  if (x_ == endX_) tMaxX_ = std::numeric_limits<double>::infinity();
  if (y_ == endY_) tMaxY_ = std::numeric_limits<double>::infinity();
  if (z_ == endZ_) tMaxZ_ = std::numeric_limits<double>::infinity();

  if (tMaxX_ < tMaxY_) {
    if (tMaxX_ < tMaxZ_) {
      // Update which cube we are now in.
      x_ += stepX_;
      // Adjust tMaxX to the next X-oriented boundary crossing.
      tMaxX_ += tDeltaX_;
    } else {
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  } else {
    if (tMaxY_ < tMaxZ_) {
      y_ += stepY_;
      tMaxY_ += tDeltaY_;
    } else {
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  }

  return true;
}

bool RayCaster::biNextId(Eigen::Vector3i& idx, Eigen::Vector3i& idxR)
{
  bool pflag = true, nflag = true;
  
  auto tmp = Eigen::Vector3d(x_, y_, z_);
  idx = tmp.cast<int>();
  
  if (x_ == interX_ && y_ == interY_ && z_ == interZ_)
    pflag = false;
  
  if (pflag == true)
  {
  if (x_ == interX_) tMaxXp = std::numeric_limits<double>::infinity();
  if (y_ == interY_) tMaxYp = std::numeric_limits<double>::infinity();
  if (z_ == interZ_) tMaxZp = std::numeric_limits<double>::infinity();
  if (tMaxXp < tMaxYp)
  {
    if (tMaxXp < tMaxZp)
    {
      x_ += stepXp;
      tMaxXp += tDeltaXp;
    }
    else
    {
      z_ += stepZp;
      tMaxZp += tDeltaZp;
    }
  }
  else
  {
    if (tMaxYp < tMaxZp) 
    {
      y_ += stepYp;
      tMaxYp += tDeltaYp;
    } else 
    {
      z_ += stepZp;
      tMaxZp += tDeltaZp;
    }
  }
  }

  auto tmpR = Eigen::Vector3d(endX_, endY_, endZ_);
  idxR = tmpR.cast<int>();

  if (endX_ == interX_ && endY_ == interY_ && endZ_ == interZ_)
    nflag = false;
  
  if (nflag == true)
  {
  if (endX_ == interX_) tMaxXn = std::numeric_limits<double>::infinity();
  if (endY_ == interY_) tMaxYn = std::numeric_limits<double>::infinity();
  if (endZ_ == interZ_) tMaxZn = std::numeric_limits<double>::infinity();
  if (tMaxXn < tMaxYn)
  {
    if (tMaxXn < tMaxZn)
    {
      endX_ += stepXn;
      tMaxXn += tDeltaXn;
    }
    else
    {
      endZ_ += stepZn;
      tMaxZn += tDeltaZn;
    }
  }
  else
  {
    if (tMaxYn < tMaxZn) 
    {
      endY_ += stepYn;
      tMaxYn += tDeltaYn;
    } else 
    {
      endZ_ += stepZn;
      tMaxZn += tDeltaZn;
    }
  }
  }

  if (pflag == false && nflag == false)
    return false;
  else
    return true;
}

bool RayCaster::nextPos(Eigen::Vector3d& pos) {
  auto tmp = Eigen::Vector3d(x_, y_, z_);
  pos = origin_ + (tmp + half_) * resolution_;

  if (x_ == endX_ && y_ == endY_ && z_ == endZ_) {
    return false;
  }

  // tMaxX stores the t-value at which we cross a cube boundary along the
  // X axis, and similarly for Y and Z. Therefore, choosing the least tMax
  // chooses the closest cube boundary. Only the first case of the four
  // has been commented in detail.
  if (x_ == endX_) tMaxX_ = std::numeric_limits<double>::infinity();
  if (y_ == endY_) tMaxY_ = std::numeric_limits<double>::infinity();
  if (z_ == endZ_) tMaxZ_ = std::numeric_limits<double>::infinity();

  if (tMaxX_ < tMaxY_) {
    if (tMaxX_ < tMaxZ_) {
      // Update which cube we are now in.
      x_ += stepX_;
      // Adjust tMaxX to the next X-oriented boundary crossing.
      tMaxX_ += tDeltaX_;
    } else {
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  } else {
    if (tMaxY_ < tMaxZ_) {
      y_ += stepY_;
      tMaxY_ += tDeltaY_;
    } else {
      z_ += stepZ_;
      tMaxZ_ += tDeltaZ_;
    }
  }

  return true;
}
