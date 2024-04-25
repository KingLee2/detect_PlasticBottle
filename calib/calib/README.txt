add_suppress.m 	重新选择哪些图做CALIB计算
affine.m  全库未使用
align_structures.m  全库未使用 作用：找到左右相机（？） 的R T 矩阵
analyse_error.m  calib_gui 主函数 误差分析
anisdiff.m  全图去光强？
apply_distortion.m
apply_distortion2.m
apply_fisheye_distortion.m
calib.m
calibration_pattern
calib_gui.m
calib_gui_fisheye.m
calib_gui_normal.m
calib_gui_no_read.m
calib_stereo.m  立体标定，计算立体R T
cam_proj_calib.m
cam_proj_calib_optim.m
cam_proj_extract_param.m
centercirclefinder.m
check_active_images.m
check_convergence.m
check_directory.m
check_extracted_images.m
clearwin.m
clear_windows.m
click_calib.m
click_calib_fisheye_no_read.m
click_calib_no_read.m
click_ima_calib.m
click_ima_calib3D.m
click_ima_calib_fisheye_no_read.m
click_ima_calib_no_read.m
click_stereo.m
combine_calib.m
compose_motion.m
Compute3D.m 从极水平图计算3D 由scanning_script调用 调用之前要有calib_cam_proj_optim.mat
ComputeStripes.m
compute_collineation.m
compute_epipole.m
compute_extrinsic.m
compute_extrinsic_init.m
compute_extrinsic_init_fisheye.m
compute_extrinsic_refine.m
compute_extrinsic_refine2.m
compute_extrinsic_refine_fisheye.m
compute_homography.m
comp_distortion.m
comp_distortion2.m
comp_distortion_oulu.m
comp_error_calib.m
comp_error_calib_fisheye.m
comp_ext_calib.m
comp_ext_calib_fisheye.m
comp_fisheye_distortion.m
cornerfinder.m  找角点
cornerfinder2.m
cornerfinder_saddle_point.m
count_squares.m
count_squares_distorted.m
count_squares_fisheye_distorted.m
dAB.m
data_calib.m
data_calib_no_read.m
Distor2Calib.m
downsample.m
edgefinder.m
eliminate_boundary.m
error_analysis.m
error_cam_proj.m
error_cam_proj2.m
error_cam_proj3.m
error_depth.m
error_depth_list.m
export_calib_data.m 
extract_distortion_data.m
extract_grid.m
extract_grid_manual.m
extract_parameters.m
extract_parameters3D.m
extract_parameters_fisheye.m
extrinsic_computation.m
ext_calib.m
ext_calib2.m
ext_calib_stereo.m  显示立体标定图
fixallvariables.m
fixvariable.m
fov.m  计算field of view
ginput2.m
ginput3.m
ginput4.m
go_calib_optim.m
go_calib_optim_fisheye_no_read.m
go_calib_optim_iter.m
go_calib_optim_iter_fisheye.m
go_calib_optim_iter_weak.m
go_calib_optim_no_read.m
go_calib_stereo.m
ima_read_calib.m
ima_read_calib_no_read.m
init_intrinsic_param.m
init_intrinsic_param_fisheye.m
inverse_motion.m
is3D.m
loading_calib.m
loading_stereo_calib.m
loadinr.m 读INR图片
loadpgm.m 读PGM图片
loadppm.m 读PPM图片
load_image.m  读图
load_stereo_calib_files.m
manual_corner_extraction.m
manual_corner_extraction_no_read.m
mean_std_robust.m
merge_calibration_sets.m
merge_two_datasets.m
Meshing.m
mosaic.m
mosaic_no_read.m
normalize.m
normalize2.m
normalize_pixel.m
normalize_pixel_fisheye.m
pattern.eps
pgmread.m  读PGM图片
point_distribution.m
project2_oulu.m
projectedGrid.m
projector_calib.m
projector_ima_corners.m
projector_marker.m
project_points.m
project_points2.m
project_points3.m
project_points_fisheye.m
project_points_weak.m
README.txt
readras.m
recomp_corner_calib.m
recomp_corner_calib_fisheye_no_read.m
recomp_corner_calib_no_read.m
recomp_corner_calib_saddle_points.m
rect.m
Rectangle2Square.m
rectify_stereo_pair.m
rect_index.m
reproject_calib.m
reproject_calib_no_read.m
rigid_motion.m 刚体旋转
rodrigues.m  R 矩阵从1*3 转为3*3
rotation.m
run_error_analysis.m
saveinr.m  保存图像为 INR格式
savepgm.m  保存图像为 PGM格式
saveppm.m  保存图像为 PPM格式
saving_calib.m
saving_calib_ascii.m
saving_calib_ascii_fisheye.m
saving_calib_fisheye.m
saving_calib_no_results.m
saving_stereo_calib.m
scanner_calibration_script.m
scanning_script.m
script_fit_distortion.m
script_fit_distortion_fisheye.m
show_calib_results.m
show_calib_results_fisheye.m
show_stereo_calib_results.m
show_window.m
skew3.m
small_test_script.m
smooth_images.m
startup.m
stereo_gui.m  立体标定的UI
stereo_triangulation.m 未使用
TestFunction.m
undistort_image.m
undistort_image_color.m
undistort_image_no_read.m
undistort_sequence.m
UnWarpPlane.m
visualize_distortions.m  显示畸变图。 需要导入Calib_Results 主要还是内参
willson_convert.m
willson_read.m
writeras.m
write_image.m