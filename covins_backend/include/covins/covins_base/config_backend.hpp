/**
* This file is part of COVINS.
*
* Copyright (C) 2018-2021 Patrik Schmuck / Vision for Robotics Lab
* (ETH Zurich) <collaborative (dot) slam (at) gmail (dot) com>
* For more information see <https://github.com/VIS4ROB-lab/covins>
*
* COVINS is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the license, or
* (at your option) any later version.
*
* COVINS is distributed to support research and development of
* multi-agent system, but WITHOUT ANY WARRANTY; without even the
* implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR
* PURPOSE. In no event will the authors be held liable for any damages
* arising from the use of this software. See the GNU General Public
* License for more details.
*
* You should have received a copy of the GNU General Public License
* along with COVINS. If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

// C++
#include <iostream>
#include <opencv2/opencv.hpp>

// COVINS
#include <covins/covins_base/typedefs_base.hpp>

namespace covins_params {

using precision_t = covins::TypeDefs::precision_t;

const std::string s0 (__FILE__);
const std::size_t p0 = s0.find("include/covins");
const std::string s1 (s0.substr(0,p0));
const std::string s2 ("config/config_backend.yaml");
const std::string s3 = s1 + s2;
const std::string conf (s3);

const std::string out0 (__FILE__);
const std::size_t iout0 = out0.find("include/covins");
const std::string out1 (out0.substr(0,iout0));
const std::string out2 ("output/");
const std::string out3 = out1 + out2;
const std::string outpath (out3);

struct VisColorRGB {
public:
    VisColorRGB(float fR,float fG, float fB)
        : mfR(fR),mfG(fG),mfB(fB),
          mu8R((u_int8_t)(fR*255)),mu8G((u_int8_t)(fG*255)),mu8B((u_int8_t)(fB*255))
        {}

    const float mfR,mfG,mfB;
    const u_int8_t mu8R,mu8G,mu8B;
};

inline std::vector<VisColorRGB> LoadColAsVec(std::string path)
{
    const VisColorRGB mc0                               = VisColorRGB(estd2::GetValFromYaml<float>(path,"vis.colorR0"),estd2::GetValFromYaml<float>(path,"vis.colorG0"),estd2::GetValFromYaml<float>(path,"vis.colorB0"));
    const VisColorRGB mc1                               = VisColorRGB(estd2::GetValFromYaml<float>(path,"vis.colorR1"),estd2::GetValFromYaml<float>(path,"vis.colorG1"),estd2::GetValFromYaml<float>(path,"vis.colorB1"));
    const VisColorRGB mc2                               = VisColorRGB(estd2::GetValFromYaml<float>(path,"vis.colorR2"),estd2::GetValFromYaml<float>(path,"vis.colorG2"),estd2::GetValFromYaml<float>(path,"vis.colorB2"));
    const VisColorRGB mc3                               = VisColorRGB(estd2::GetValFromYaml<float>(path,"vis.colorR3"),estd2::GetValFromYaml<float>(path,"vis.colorG3"),estd2::GetValFromYaml<float>(path,"vis.colorB3"));
    const VisColorRGB mc4                               = VisColorRGB(estd2::GetValFromYaml<float>(path,"vis.colorR4"),estd2::GetValFromYaml<float>(path,"vis.colorG4"),estd2::GetValFromYaml<float>(path,"vis.colorB4"));
    const VisColorRGB mc5                               = VisColorRGB(estd2::GetValFromYaml<float>(path,"vis.colorR5"),estd2::GetValFromYaml<float>(path,"vis.colorG5"),estd2::GetValFromYaml<float>(path,"vis.colorB5"));
    const VisColorRGB mc6                               = VisColorRGB(estd2::GetValFromYaml<float>(path,"vis.colorR6"),estd2::GetValFromYaml<float>(path,"vis.colorG6"),estd2::GetValFromYaml<float>(path,"vis.colorB6"));
    const VisColorRGB mc7                               = VisColorRGB(estd2::GetValFromYaml<float>(path,"vis.colorR7"),estd2::GetValFromYaml<float>(path,"vis.colorG7"),estd2::GetValFromYaml<float>(path,"vis.colorB7"));
    const VisColorRGB mc8                               = VisColorRGB(estd2::GetValFromYaml<float>(path,"vis.colorR8"),estd2::GetValFromYaml<float>(path,"vis.colorG8"),estd2::GetValFromYaml<float>(path,"vis.colorB8"));
    const VisColorRGB mc9                               = VisColorRGB(estd2::GetValFromYaml<float>(path,"vis.colorR9"),estd2::GetValFromYaml<float>(path,"vis.colorG9"),estd2::GetValFromYaml<float>(path,"vis.colorB9"));
    const VisColorRGB mc10                              = VisColorRGB(estd2::GetValFromYaml<float>(path,"vis.colorR10"),estd2::GetValFromYaml<float>(path,"vis.colorG10"),estd2::GetValFromYaml<float>(path,"vis.colorB10"));
    const VisColorRGB mc11                              = VisColorRGB(estd2::GetValFromYaml<float>(path,"vis.colorR11"),estd2::GetValFromYaml<float>(path,"vis.colorG11"),estd2::GetValFromYaml<float>(path,"vis.colorB11"));
    std::vector<VisColorRGB> colors;
    colors.push_back(mc0);
    colors.push_back(mc1);
    colors.push_back(mc2);
    colors.push_back(mc3);
    colors.push_back(mc4);
    colors.push_back(mc5);
    colors.push_back(mc6);
    colors.push_back(mc7);
    colors.push_back(mc8);
    colors.push_back(mc9);
    colors.push_back(mc10);
    colors.push_back(mc11);
    return colors;
}

namespace sys {
    const int threads_server                            = estd2::GetValFromYaml<int>(conf,"sys.threads_server");
    const int covis_thres                               = estd2::GetValFromYaml<int>(conf,"sys.covis_thres");
    const std::string map_path0                         = estd2::GetStringFromYaml(conf,"sys.map_path0");
    const std::string map_path1                         = estd2::GetStringFromYaml(conf,"sys.map_path1");
    const std::string map_path2                         = estd2::GetStringFromYaml(conf,"sys.map_path2");
    const std::string map_path3                         = estd2::GetStringFromYaml(conf,"sys.map_path3");
    const std::string map_path4                         = estd2::GetStringFromYaml(conf,"sys.map_path4");
    const std::string map_path5                         = estd2::GetStringFromYaml(conf,"sys.map_path5");
    const std::string map_path6                         = estd2::GetStringFromYaml(conf,"sys.map_path6");
    const std::string map_path7                         = estd2::GetStringFromYaml(conf,"sys.map_path7");
    const std::string map_path8                         = estd2::GetStringFromYaml(conf,"sys.map_path8");
    const std::string map_path9                         = estd2::GetStringFromYaml(conf,"sys.map_path9");
    const std::string map_path10                        = estd2::GetStringFromYaml(conf,"sys.map_path10");
    const std::string map_path11                        = estd2::GetStringFromYaml(conf,"sys.map_path11");
    //--------------------------
    const std::string voc_orb_dir                       = s1 + "config/ORBvoc.txt";
    //--------------------------
     const std::string data_path0                       = estd2::GetStringFromYaml(conf,"sys.data_path0");
    const std::string data_path1                        = estd2::GetStringFromYaml(conf,"sys.data_path1");
    const std::string data_path2                        = estd2::GetStringFromYaml(conf,"sys.data_path2");
    const std::string data_path3                        = estd2::GetStringFromYaml(conf,"sys.data_path3");
    const std::string data_path4                        = estd2::GetStringFromYaml(conf,"sys.data_path4");
    //--------------------------
    const std::string output_dir                        = outpath;
    const std::string trajectory_format                 = estd2::GetStringFromYaml(conf,"sys.trajectory_format");
}

namespace features {
    const std::string type                              = estd2::GetStringFromYaml(conf,"feat.type");
    const int desc_length                               = estd2::GetValFromYaml<int>(conf,"feat.desc_length");
    const int num_octaves                               = estd2::GetValFromYaml<int>(conf,"feat.num_octaves");
    const precision_t scale_factor                      = estd2::GetValFromYaml<precision_t>(conf,"feat.scale_factor");
    const float img_match_thres                         = estd2::GetValFromYaml<float>(conf,"extractor.img_match_thres");
}

namespace matcher {
    const int desc_matching_th_low                      = estd2::GetValFromYaml<int>(conf,"matcher.desc_matching_th_low");
    const int desc_matching_th_high                     = estd2::GetValFromYaml<int>(conf,"matcher.desc_matching_th_high");
    const precision_t search_radius_SE3                 = estd2::GetValFromYaml<precision_t>(conf,"matcher.search_radius_SE3");
    const precision_t search_radius_proj                = estd2::GetValFromYaml<precision_t>(conf,"matcher.search_radius_proj");
    const precision_t search_radius_fuse                = estd2::GetValFromYaml<precision_t>(conf,"matcher.search_radius_fuse");
}

namespace mapping {
    const bool activate_lm_culling                      = estd2::GetValFromYaml<bool>(conf,"mapping.activate_lm_culling");
    const precision_t kf_culling_th_red                 = estd2::GetValFromYaml<precision_t>(conf,"mapping.kf_culling_th_red");
    const precision_t kf_culling_max_time_dist          = estd2::GetValFromYaml<precision_t>(conf,"mapping.kf_culling_max_time_dist");
}

namespace placerec {
    const bool active                                   = estd2::GetValFromYaml<bool>(conf,"placerec.active");
    const bool use_gt                                   = estd2::GetValFromYaml<bool>(conf,"placerec.use_gt");
    const std::string type                              = estd2::GetStringFromYaml(conf,"placerec.type");
    const size_t start_after_kf                         = estd2::GetValFromYaml<int>(conf,"placerec.start_after_kf");
    const size_t consecutive_loop_dist                  = estd2::GetValFromYaml<int>(conf,"placerec.consecutive_loop_dist");
    const int min_loop_dist                             = estd2::GetValFromYaml<int>(conf,"placerec.min_loop_dist");
    const int cov_consistency_thres                     = estd2::GetValFromYaml<int>(conf,"placerec.cov_consistency_thres");
    const int solver_iterations                         = estd2::GetValFromYaml<int>(conf,"placerec.solver_iterations");
    const int matches_thres                             = estd2::GetValFromYaml<int>(conf,"placerec.matches_thres");
    const int matches_thres_merge                       = estd2::GetValFromYaml<int>(conf,"placerec.matches_thres_merge");
    const int inliers_thres                             = estd2::GetValFromYaml<int>(conf,"placerec.inliers_thres");
    const int total_matches_thres                       = estd2::GetValFromYaml<int>(conf,"placerec.total_matches_thres");

    const bool inter_map_matches_only                   = estd2::GetValFromYaml<bool>(conf,"placerec.inter_map_matches_only");
    const int exclude_kfs_with_id_less_than             = estd2::GetValFromYaml<size_t>(conf,"placerec.exclude_kfs_with_id_less_than");

    namespace ransac {
        const int min_inliers               = estd2::GetValFromYaml<int>(conf,"placerec.ransac.min_inliers");
        const precision_t probability       = estd2::GetValFromYaml<precision_t>(conf,"placerec.ransac.probability");
        const int max_iterations            = estd2::GetValFromYaml<int>(conf,"placerec.ransac.max_iterations");
        const int class_threshold           = estd2::GetValFromYaml<int>(conf,"placerec.ransac.class_threshold");
    } // namespace ransac

    namespace nc_rel_pose {
        const int cov_iters                 = estd2::GetValFromYaml<int>(conf,"placerec.nc_rel_pose.cov_iters");
        const int max_iters                 = estd2::GetValFromYaml<int>(conf,"placerec.nc_rel_pose.max_iters");
        const int min_inliers               = estd2::GetValFromYaml<int>(conf,"placerec.nc_rel_pose.min_inliers");
        const float cov_thres               = estd2::GetValFromYaml<float>(conf,"placerec.nc_rel_pose.cov_thres");
        const float rp_error                = estd2::GetValFromYaml<float>(conf,"placerec.nc_rel_pose.rp_error");
    } //namespace nc_rel_pose

    namespace rel_pose {
        const int max_iters                 = estd2::GetValFromYaml<int>(conf,"placerec.rel_pose.max_iters");
        const int min_inliers               = estd2::GetValFromYaml<int>(conf,"placerec.rel_pose.min_inliers");
        const float error_thres             = estd2::GetValFromYaml<float>(conf,"placerec.rel_pose.error_thres");
        const int min_img_matches           = estd2::GetValFromYaml<int>(conf,"placerec.rel_pose.min_img_matches");
    } //namespace nc_rel_pose
    
    const bool use_4dof                     = estd2::GetValFromYaml<bool>(conf,"placerec.use_4dof");
    const bool use_LBA                       = estd2::GetValFromYaml<bool>(conf,"placerec.enable_LBA");
}

namespace opt {
    const int gba_iteration_limit                       = estd2::GetValFromYaml<int>(conf,"opt.gba_iteration_limit");
    const precision_t th_outlier_align                  = estd2::GetValFromYaml<precision_t>(conf,"opt.th_outlier_align");
    const precision_t th_gba_outlier_global             = estd2::GetValFromYaml<precision_t>(conf,"opt.th_gba_outlier_global");
    const int pgo_iteration_limit                       = estd2::GetValFromYaml<int>(conf,"opt.pgo_iteration_limit");

    const bool perform_pgo                              = estd2::GetValFromYaml<bool>(conf,"opt.perform_pgo");
    const bool use_cov_f                                = estd2::GetValFromYaml<bool>(conf,"opt.use_cov_f");
    const bool use_robust_loss                          = estd2::GetValFromYaml<bool>(conf,"opt.use_robust_loss");
    const bool pgo_fix_kfs_after_gba                    = estd2::GetValFromYaml<bool>(conf,"opt.pgo_fix_kfs_after_gba");
    const bool pgo_fix_poses_loaded_maps                = estd2::GetValFromYaml<bool>(conf,"opt.pgo_fix_poses_loaded_maps");
    const bool gba_fix_poses_loaded_maps                = estd2::GetValFromYaml<bool>(conf,"opt.gba_fix_poses_loaded_maps");

    const bool pgo_use_cov_edges                        = estd2::GetValFromYaml<bool>(conf,"opt.pgo_use_cov_edges");
    const int pgo_min_edge_weight                       = estd2::GetValFromYaml<int>(conf,"opt.pgo_min_edge_weight");
    const bool pgo_use_map_loop_constraints             = estd2::GetValFromYaml<bool>(conf,"opt.pgo_use_map_loop_constraints");
    const bool pgo_use_loop_edges                       = estd2::GetValFromYaml<bool>(conf,"opt.pgo_use_loop_edges");

    const bool gba_use_map_loop_constraints             = estd2::GetValFromYaml<bool>(conf,"opt.gba_use_map_loop_constraints");

    // For Weighting Loops and KFs in PGO
    const float wt_kf_r             = estd2::GetValFromYaml<float>(conf,"opt.wt_kf_R");
    const float wt_kf_t             = estd2::GetValFromYaml<float>(conf,"opt.wt_kf_T");
    const float wt_lp_r1            = estd2::GetValFromYaml<float>(conf,"opt.wt_lp_R1");
    const float wt_lp_t1            = estd2::GetValFromYaml<float>(conf,"opt.wt_lp_T1");
    const float wt_lp_r2            = estd2::GetValFromYaml<float>(conf,"opt.wt_lp_R2");
    const float wt_lp_t2            = estd2::GetValFromYaml<float>(conf,"opt.wt_lp_T2");
    const float wt_lp_r3            = estd2::GetValFromYaml<float>(conf,"opt.wt_lp_R3");
    const float wt_lp_t3            = estd2::GetValFromYaml<float>(conf,"opt.wt_lp_T3");
    const float cov_switch          = estd2::GetValFromYaml<float>(conf,"opt.cov_switch");
    const float cov_switch2         = estd2::GetValFromYaml<float>(conf,"opt.cov_switch2");
}

namespace vis {
    const bool active                                   = estd2::GetValFromYaml<bool>(conf,"vis.active");
    const bool showcovgraph                             = estd2::GetValFromYaml<bool>(conf,"vis.showcovgraph");
    const bool showlandmarks                            = estd2::GetValFromYaml<bool>(conf,"vis.showlandmarks");
    const bool showtraj                                 = estd2::GetValFromYaml<bool>(conf,"vis.showtraj");
    const bool showkeyframes                            = estd2::GetValFromYaml<bool>(conf,"vis.showkeyframes"); //-1=no KFs;0=frusta;1=spheres
    const int covgraph_minweight                        = estd2::GetValFromYaml<int>(conf,"vis.covgraph_minweight");
    const bool covgraph_shared_edges_only               = estd2::GetValFromYaml<bool>(conf,"vis.covgraph_shared_edges_only"); // show only cov edges between trajectories from different agents

    const precision_t scalefactor                       = estd2::GetValFromYaml<precision_t>(conf,"vis.scalefactor");
    const precision_t trajmarkersize                    = estd2::GetValFromYaml<precision_t>(conf,"vis.trajmarkersize");
    const precision_t covmarkersize                     = estd2::GetValFromYaml<precision_t>(conf,"vis.covmarkersize");
    const precision_t loopmarkersize                    = estd2::GetValFromYaml<precision_t>(conf,"vis.loopmarkersize");
    const precision_t camsize                           = estd2::GetValFromYaml<precision_t>(conf,"vis.camsize");
    const precision_t camlinesize                       = estd2::GetValFromYaml<precision_t>(conf,"vis.camlinesize");
}

namespace colors {
    const std::vector<VisColorRGB> col_vec              = LoadColAsVec(conf);
    const VisColorRGB color_cov                         = VisColorRGB(estd2::GetValFromYaml<covins::TypeDefs::precision_t>(conf,"vis.colorRcov"),estd2::GetValFromYaml<covins::TypeDefs::precision_t>(conf,"vis.colorGcov"),estd2::GetValFromYaml<covins::TypeDefs::precision_t>(conf,"vis.colorBcov"));
}

void ShowParamsBackend();

} //end ns
