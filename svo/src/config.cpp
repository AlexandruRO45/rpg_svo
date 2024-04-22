// This file is part of SVO - Semi-direct Visual Odometry.
//
// Copyright (C) 2014 Christian Forster <forster at ifi dot uzh dot ch>
// (Robotics and Perception Group, University of Zurich, Switzerland).
//
// SVO is free software: you can redistribute it and/or modify it under the
// terms of the GNU General Public License as published by the Free Software
// Foundation, either version 3 of the License, or any later version.
//
// SVO is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

#include <svo/config.h>
#include <utility>

namespace svo {

Config::Config() :
    trace_name("svo"),
    trace_dir("/tmp"),
    n_pyr_levels(3),
    use_imu(false),
    core_n_kfs(3),
    map_scale(1.0),
    grid_size(25),
    init_min_disparity(50.0),
    init_min_tracked(50),
    init_min_inliers(40),
    klt_max_level(4),
    klt_min_level(2),
    reproj_thresh(2.0),
    poseoptim_thresh(2.0),
    poseoptim_num_iter(10),
    structureoptim_max_pts(20),
    structureoptim_num_iter(5),
    loba_thresh(2.0),
    loba_robust_huber_width(1.0),
    loba_num_iter(0),
    kfselect_mindist(0.12),
    triang_min_corner_score(20.0),
    triang_half_patch_size(4),
    subpix_n_iter(10),
    max_n_kfs(0),
    img_imu_delay(0.0),
    max_fts(120),
    quality_min_fts(50),
    quality_max_drop_fts(40)
{}

  Config::Config(
    string trace_name_,
    string trace_dir_,
    size_t n_pyr_levels_,
    bool use_imu_,
    size_t core_n_kfs_,
    double map_scale_,
    size_t grid_size_,
    double init_min_disparity_,
    size_t init_min_tracked_,
    size_t init_min_inliers_,
    size_t klt_max_level_,
    size_t klt_min_level_,
    double reproj_thresh_,
    double poseoptim_thresh_,
    size_t poseoptim_num_iter_,
    size_t structureoptim_max_pts_,
    size_t structureoptim_num_iter_,
    double loba_thresh_,
    double loba_robust_huber_width_,
    size_t loba_num_iter_,
    double kfselect_mindist_,
    double triang_min_corner_score_,
    size_t triang_half_patch_size_,
    size_t subpix_n_iter_,
    size_t max_n_kfs_,
    double img_imu_delay_,
    size_t max_fts_,
    size_t quality_min_fts_,
    int quality_max_drop_fts_
  ) :
    trace_name(std::move(trace_name_)),
    trace_dir(std::move(trace_dir_)),
    n_pyr_levels(n_pyr_levels_),
    use_imu(use_imu_),
    core_n_kfs(core_n_kfs_),
    map_scale(map_scale_),
    grid_size(grid_size_),
    init_min_disparity(init_min_disparity_),
    init_min_tracked(init_min_tracked_),
    init_min_inliers(init_min_inliers_),
    klt_max_level(klt_max_level_),
    klt_min_level(klt_min_level_),
    reproj_thresh(reproj_thresh_),
    poseoptim_thresh(poseoptim_thresh_),
    poseoptim_num_iter(poseoptim_num_iter_),
    structureoptim_max_pts(structureoptim_max_pts_),
    structureoptim_num_iter(structureoptim_num_iter_),
    loba_thresh(loba_thresh_),
    loba_robust_huber_width(loba_robust_huber_width_),
    loba_num_iter(loba_num_iter_),
    kfselect_mindist(kfselect_mindist_),
    triang_min_corner_score(triang_min_corner_score_),
    triang_half_patch_size(triang_half_patch_size_),
    subpix_n_iter(subpix_n_iter_),
    max_n_kfs(max_n_kfs_),
    img_imu_delay(img_imu_delay_),
    max_fts(max_fts_),
    quality_min_fts(quality_min_fts_),
    quality_max_drop_fts(quality_max_drop_fts_)
  {}

Config& Config::getInstance()
{
  return instance;
}

void Config::setInstance(const Config &config)
{
  instance = config;
}

Config instance;

} // namespace svo

