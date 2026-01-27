^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package sample_sensor_kit_description
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-changelog-0.49.0
* feat: remove sensor component dependency (`#1722 <https://github.com/autowarefoundation/autoware_launch/issues/1722>`_)
* Contributors: Ryohsuke Mitsudome, Takagi, Isamu

0.48.0 (2025-11-16)
-------------------
* Merge remote-tracking branch 'origin/main' into release-humble-0.48.0
* fix: add missing exec_depends for sensor description (`#1699 <https://github.com/autowarefoundation/autoware_launch/issues/1699>`_)
  * fix: add missing exec_depends for sensor description
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: Ryohsuke Mitsudome, mitsudome-r

0.47.0 (2025-08-11)
-------------------

0.46.0 (2025-06-20)
-------------------

0.45.0 (2025-05-22)
-------------------

0.44.3 (2025-06-10)
-------------------

0.44.2 (2025-05-30)
-------------------

0.44.1 (2025-05-12)
-------------------

0.44.0 (2025-05-01)
-------------------
* Merge commit 'bdbc8e8' into bump-up-version-to-0.44.0
* chore: bump version to 0.44.0 (`#1411 <https://github.com/autowarefoundation/autoware_launch/issues/1411>`_)
* feat: remove individual_params references (`#1403 <https://github.com/autowarefoundation/autoware_launch/issues/1403>`_)

0.43.1 (2025-04-01)
-------------------
* chore: set all versions to 0.43.0 (`#1384 <https://github.com/autowarefoundation/autoware_launch/issues/1384>`_)
* feat(*_launch): move here (`#1369 <https://github.com/autowarefoundation/autoware_launch/issues/1369>`_)
* Contributors: Mete Fatih Cırıt

0.43.0 (2025-02-12)
-------------------

0.41.0 (2025-02-12)
-------------------

0.40.0 (2025-01-17)
-------------------

0.39.0 (2024-12-09)
-------------------
* chore: update maintainer (`#60 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/60>`_)
* fix: suppress xacro warning (`#51 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/51>`_)
* ci(pre-commit): update pre-commit-hooks-ros (`#16 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/16>`_)
  * ci(pre-commit): update pre-commit-hooks-ros
  * ci(pre-commit): autofix
  * Update .pre-commit-config.yaml
  * Update .pre-commit-config.yaml
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: clean up files (`#6 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/6>`_)
* feat: add packages (`#3 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/3>`_)
  * release v0.4.0
  * Feature/phased timestamped velodyne (`#53 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/53>`_)
  * Replace with new velodyne driver, cutting scan based on azimuth
  * Fix launch/dependences
  * Fix version name for tier4/velodyne_vls
  * Add velodyne_driver dependency
  * Nodelet tlr (`#56 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/56>`_)
  * temporary commit tlr_nodelet
  * compressed to compressed
  * Update traffic_light.launch
  * fix bug
  * change image_transport to relay
  * fix bug
  * fix bug
  * decompress as rgb8
  * fix bug
  Co-authored-by: Yukihiro Saito <yukky.saito@gmail.com>
  * use env for livox id (`#58 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/58>`_)
  * Feature/optimize scan phase (`#59 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/59>`_)
  * Rename parameter name, sensor_phase -> scan_phase
  * Modify aip_xx1 scan_phase for better perception
  * Rename parameter name, sensor_phase -> scan_phase
  * Logging simulator (`#65 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/65>`_)
  * Add logging_simulator.launch
  * Don't load env when launch driver is false
  * removed ROS1 package
  * Revert "removed ROS1 package"
  This reverts commit 3122355145ddfc9cb7e7485e85d509d53f6836f0.
  * add COLCON_IGNORE file to all ROS1 packages
  * rename *.launch files to *.launch.xml
  * Port sensing_launch (`#14 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/14>`_)
  * [sensing_launch] Initial port without actually launching
  * [sensing_launch] default -> value, namespace, first nodelet porting
  * [sensing_launch] use usb_cam, eval -> var
  * [sensing_launch] Fix syntax errors in pointcloud_preprocessor.launch.py
  * [pointcloud-preprocessor] fix ground-filter component name
  * [pointcloud-preprocessor] Polish aip_s1/pointcloud_preprocessor.launch.py
  Only one error at runtime remains when testing on dev laptop due to pointclouds that need to be available for concatenation
  * [sensing_launch] ublox_gps refer to config file properly
  * (wip) velodyne_node_container before opaque
  * [sensing_launch] Port aip-s1 as far as possible
  * [sensing_launch] remove unused pointcloud_preprocessor_nodes.py
  * [sensing_launch] Manage to add ComposableNode conditionally
  * [sensing_launch] Update camera for s1, x1
  * [sensing_launch] Copy aip_s1/ content to aip_customized, aip_x1, aip_x2
  because they were identical before the porting
  * [sensing_launch] Port livox
  * [sensing_launch] Port aip_xx1
  * [sensing_launch] Port aip_xx2
  * [sensing_launch] Remove superfluous passthrough filter, min_z, max_z
  * [sensing_launch] Incorporate changes from vehicle testing
  * [sensing_launch] Declare launch argument for base_frame
  * [sensing_launch] Missing fixes to launch/velodyne*
  * [sensing_launch] Update copied configs
  * Added linters (`#32 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/32>`_)
  * Ros2 v0.8.0 sensing launch (`#57 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/57>`_)
  * restore file name
  * Update livox_horizon.launch (`#89 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/89>`_)
  * fix pass through filter launch (`#90 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/90>`_)
  * fix pass through filter launch
  * change if statement style
  * update aip_x1 sensing launch (`#100 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/100>`_)
  * fix livox launch arg (`#108 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/108>`_)
  * add usb_cam depend (`#118 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/118>`_)
  * update aip_x1 camera.launch (`#119 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/119>`_)
  * update imu.launch (`#120 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/120>`_)
  * fix veodyne setting in aip_x1/lidar.launch (`#125 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/125>`_)
  * Add velodyne_monitor to velodyne\_*.launch (`#101 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/101>`_)
  * Uupdate aip_x1 lidar.launch (`#143 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/143>`_)
  * Format gnss.launch (`#145 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/145>`_)
  * Add use_gnss arg to aip_x1 gnss.launch (`#146 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/146>`_)
  * support individual params (`#137 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/137>`_)
  * support individual params
  * remove kvaser_hardware_id.txt
  * Launch velodyne_monitor only when launch_driver is true (`#163 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/163>`_)
  * [sensing_launch] ros2 porting: use container for livox point preprocessor
  * [sensing_launch] ros2-porting: fix vehicle_info params
  * Revert "restore file name"
  This reverts commit 37d7ac4f6a2a617b003b4e2a5ac96c48b332ade0.
  * [sensing_launch] ros2-porting: fix vehicle_info for livox preprocessor launch
  * [sensing_launch] ros2-porting: fix vehicle_info for api\_** points_preprocessor.launch.py
  * fix launch
  * fix livox launch
  * added suffix ".xml" to "velodyne_monitor.launch" in the launch files
  * added use_sim_time with AW_ROS2_USE_SIM_TIME envvar for the parameters in  the *.launch.py (`#61 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/61>`_)
  * added use_sim_time with AW_ROS2_USE_SIM_TIME envvar for the parameters
  * changed to use EnvironmentVariable function for use_sim_time parameter
  * changed indent
  * removed an empty line
  Co-authored-by: hosokawa <hosokawa@sng-3f-ros2-eval.l.sng.tier4.jp>
  * fixed typo on the arg bd_code_param_path lines (`#63 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/63>`_)
  Co-authored-by: hosokawa <hosokawa@sng-3f-ros2-eval.l.sng.tier4.jp>
  * [sensing_launch]: Fix indentation in gnss launch
  * [sensing_launch]: Add missing dependency in package.xml
  * [sensing_launch]: Fix velodyne launch
  * [sensing_launch]: Fix livox launch
  * [sensing_launch]: Add arg for vehicle parameter file in lidar launch
  * [sensing_launch]: Cleanup
  * Add new line
  * [sensing_launch]: Add default config for xx1
  * [sensing_launch]: Fix indentation
  Co-authored-by: Yukihiro Saito <yukky.saito@gmail.com>
  Co-authored-by: Taichi Higashide <taichi.higashide@tier4.jp>
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  Co-authored-by: hiroyuki obinata <58019445+obi-t4@users.noreply.github.com>
  Co-authored-by: hosokawa <hosokawa@sng-3f-ros2-eval.l.sng.tier4.jp>
  Co-authored-by: HOSOKAWA Ikuto <hosokawa.ikuto@gmail.com>
  Co-authored-by: wep21 <border_goldenmarket@yahoo.co.jp>
  Co-authored-by: Autoware <autoware@tier4.jp>
  * Rename ROS-related .yaml to .param.yaml (`#65 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/65>`_)
  * Rename ROS-related .yaml to .param.yaml
  * Add missing '--'
  * Rename vehicle_info.yaml to vehicle_info.param.yaml
  * Fix livox param name
  * Sync with Ros2 v0.8.0 beta (`#71 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/71>`_)
  * update sensing launch to support aip_x1 (`#69 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/69>`_)
  * fix logging_simulator_bug (`#68 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/68>`_)
  * fix aip_x1 param (`#70 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/70>`_)
  Co-authored-by: Taichi Higashide <taichi.higashide@tier4.jp>
  * Fix aip_xx1's pointcloud_preprocessor.launch.py (`#72 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/72>`_)
  * fix velodyne launch (`#73 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/73>`_)
  * fix velodyne launch
  * fix bug
  * add scan_phase arg
  * fix bug (`#85 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/85>`_)
  * Use sensor data qos for pointcloud (`#82 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/82>`_)
  Co-authored-by: Autoware <autoware@tier4.jp>
  * Remove unused remappings (`#88 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/88>`_)
  * Livox composable node (`#87 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/87>`_)
  * Fix default value of use_concat_filter and use_radius_search (`#90 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/90>`_)
  * Fix default value of use_concat_filter and use_radius_search
  * Fix lint
  * [aip_x1]: Fix imu topic name (`#94 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/94>`_)
  * Fix various typos in launch files (`#97 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/97>`_)
  * Move individual params to a separate package (`#100 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/100>`_)
  * Remove individual params (`#101 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/101>`_)
  * add use_sim-time option (`#99 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/99>`_)
  * Format launch files (`#178 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/178>`_)
  * Fix bug of pointcloud_preprocessor.py (`#179 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/179>`_)
  Co-authored-by: autoware <autoware@example.com>
  * Replace doc by description (`#182 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/182>`_)
  * Ros2 lsim test (`#186 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/186>`_)
  * Add group to launch file for var scope
  * Remove pointcloud relay for localization
  * Add use_sim_time
  * Remove pointcloud relay for localization
  Co-authored-by: Takagi, Isamu <isamu.takagi@tier4.jp>
  * Add multithread and intra process option (`#187 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/187>`_)
  * Add multithread and intra process option
  * Fix velodyne node container executable
  * Add option into aip_xx2
  * Add option into aip_x2
  * Add option into aip_x1
  * Add option into aip_s1
  * Add option into aip_customized
  * Add option into lidar.launch.xml
  * Fix invalid attribute in gnss launch (`#191 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/191>`_)
  * Fix parameter for scan phase (`#193 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/193>`_)
  * add septentrio_gnss_driver launcher and switch(septentrio <-> ublox) (`#196 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/196>`_)
  * add septentrio_gnss_driver launcher and switch(septentrio <-> ublox)
  * rm arg(gnss_receiver) escalation and modify septentrio_gnss_driver_node option
  * change gnss_receiver default septentrio to ublox
  * remap all septentrio_gnss_driver topic names
  * replace septentrio gnss driver launch type 'node' to 'include'
  * Use set_parameter for use_sim_time (`#198 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/198>`_)
  * Use set_parameter for use_sim_time
  * Add default parameter for scenario simulator
  * Format launch files (`#228 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/228>`_)
  * Format launch files
  * Format launch.py
  * Fix lint
  * Fix aip_xx1 camera launch (`#242 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/242>`_)
  * Fix gnss topic name (`#243 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/243>`_)
  * Enable intra process and mt (`#204 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/204>`_)
  * add imu_corrector (`#296 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/296>`_)
  * add description for sensing_launch (`#336 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/336>`_)
  * add description
  * fix sentence
  * add README.md and svg files (`#328 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/328>`_)
  * add md and svg
  * fix typo
  * fix typo
  * fix word
  * fix typo
  * add lack of things
  * Update README
  * fix depending packages
  * fix word
  * Fix camera launch invalid type string (`#344 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/344>`_)
  * add view width direction to velodyne_node_container.launch.py etc... (`#366 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/366>`_)
  * add arg of view_width and view_direction
  * delete initial value
  * add args and params
  Co-authored-by: autoware-iv-sync-ci[bot] <87871706+autoware-iv-sync-ci[bot]@users.noreply.github.com>
  * Fix pre-commit (`#407 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/407>`_)
  * Fix pre-commit errors
  * Fix package.xml
  * Fix pre-commit target (`#436 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/436>`_)
  * Use scan ground filter for xx1 (`#313 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/313>`_)
  * Enable time series outlier filter (`#314 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/314>`_)
  * Fix param name in scan ground filter (`#357 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/357>`_)
  * Remove aip xx2 model from sensing launch (`#446 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/446>`_)
  * Add respawn for ublox (`#449 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/449>`_)
  * delete aip_customized
  * move to aip_launcher
  * delete namespace (`#5 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/5>`_)
  * fix revert dirname + delete unused arg (`#7 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/7>`_)
  * revert dirname
  * delete sensor_model
  * delete aip_s1 (`#8 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/8>`_)
  * Add pre-commit (`#10 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/10>`_)
  * Add pre-commit
  * Fix for pre-commit
  * Update version
  * Fix target
  * update README (`#9 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/9>`_)
  * update README temporary
  * Update
  * updateg
  * delete line number
  * re delete line number
  * fix for pre commit
  * fix for pre-commit
  * fix for pre commit
  * update README
  * update README
  * update README
  * Update README
  * update readme
  * use back quote
  * Sync with xx1 develop/ros2 (`#14 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/14>`_)
  * Fix velodyne launcher (`#15 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/15>`_)
  * Fix lidar launcher (`#16 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/16>`_)
  * Rollback XX1's pointcloud_preprocessor to main (`#18 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/18>`_)
  * Update aip_x1 launch files (`#25 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/25>`_)
  * Copy velodyne_node_container.launch.py to aip_x1_launch
  * Disable driving recorder (`#19 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/19>`_)
  * add use_driving_recorder param
  Co-authored-by: taichiH <azumade.30@gmail.com>
  * X1: Change scan_phase 0 to 180 deg
  * X1: Add topic state monitor
  * Add Livox tag filter
  * Add Livox min_range_filter
  * change livox_horizon.launch to support livox_tag_filter composable node (`#62 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/62>`_)
  * remove unnecessary crop filter for aip_x1 (`#63 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/63>`_)
  * remove sensing-lidar-pointcloud relay
  * add livox concatenate
  * disable use_intra_process for vector_map_filter
  * change use_intra_process to true
  * [sac ground filter] change height threshold 0.12 -> 0.18
  * Update launch for multi topic livox mode (`#68 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/68>`_)
  Co-authored-by: Hiroaki ISHIKAWA <hiroaki.ishikawa@tier4.jp>
  Co-authored-by: taichiH <azumade.30@gmail.com>
  * add westering sun extraction filter (`#76 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/76>`_)
  * fix bug (`#92 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/92>`_)
  * Fix concat timeout (`#91 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/91>`_)
  * add new livox driver launch file (`#94 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/94>`_)
  * fix frame_id (`#95 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/95>`_)
  * Feature/compare elevation map (`#100 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/100>`_)
  * Change livox crop range 14m->18m
  * Use executable for new_lidar_driver_launch (`#120 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/120>`_)
  * Change ransac height thresh (`#110 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/110>`_) (`#115 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/115>`_)
  * Add livox to diag name of topic_state_monitor (`#162 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/162>`_)
  * Change elevation value method and height thresh (`#163 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/163>`_)
  * change ground filter hight threshold (`#174 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/174>`_) (`#176 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/176>`_)
  Co-authored-by: Taichi Higashide <taichi.higashide@tier4.jp>
  * Copy velodyne_VLP16.launch.xml to aip_x1_launch
  * Change velodyne_node_container.launch.py reference in velodyne_VLP16.launch.xml
  Co-authored-by: Hiroki OTA <hiroki.ota@tier4.jp>
  Co-authored-by: taichiH <azumade.30@gmail.com>
  Co-authored-by: Taichi Higashide <taichi.higashide@tier4.jp>
  Co-authored-by: tomoya.kimura <tomoya.kimura@tier4.jp>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: Hiroaki ISHIKAWA <hiroaki.ishikawa@tier4.jp>
  Co-authored-by: Takeshi Miura <57553950+1222-takeshi@users.noreply.github.com>
  Co-authored-by: autoware-iv-sync-ci[bot] <87871706+autoware-iv-sync-ci[bot]@users.noreply.github.com>
  * Add parameter file for diagnostic_aggregator  to use in each product (`#13 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/13>`_)
  * add xx1 parameter
  * add x1 parameter
  * add x2 parameter
  * delete autoware_error_monitor
  * add sensor_kit.param for diagnostic_agg
  * update extra senser diag
  * Remove IMU from X2
  * Move to config directory
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  * Revert "Rollback XX1's pointcloud_preprocessor to main (`#18 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/18>`_)"
  This reverts commit 4f9d0e8384526d0638a18856c16500cf8933690b.
  * Change formatter to black (`#38 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/38>`_)
  * Update pre-commit settings
  * Apply Black
  * Replace ament_lint_common with autoware_lint_common
  * Update build_depends.repos
  * Fix build_depends
  * Remove lidar_ros_driver from package.xml (`#39 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/39>`_)
  * remove unused pointcloud preprocessor components (`#2 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/2>`_)
  * feature/use common pointcloud container (`#8 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/8>`_)
  * add arg
  * improve readability
  * fix/remove passthrough filter (`#9 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/9>`_)
  * release v0.4.0
  * Add vls description (`#3 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/3>`_)
  * remove ROS1 packages
  * Revert "remove ROS1 packages"
  This reverts commit 7c1e0d930473170ada063f45c961dc40abd0357b.
  * add colcon_ignore
  * port to ROS2
  * add xacro namespace for VLP-16/128 tags
  * fix xacro:color value
  * Ros2 v0.8.0 update (`#7 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/7>`_)
  * [ROS2] pandar_description (`#9 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/9>`_)
  * Feature/add pandar (`#7 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/7>`_)
  * add decription for Hesai LiDAR
  * fix direction
  * update for ros2
  * fix config_dir (`#11 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/11>`_)
  * delete descriptions except for current reference
  * fix suffix to name (`#2 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/2>`_)
  * delete aip_s1 (`#3 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/3>`_)
  * Modify sensor config (`#4 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/4>`_)
  * Update x1 sensor config files
  * Update xx1 sensor config files
  * Update x2 sensor config files
  * Run pre-commit
  * Add prettier-xacro to pre-commit (`#6 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/6>`_)
  * Run pre-commit
  * Update README.md
  * Fix for pre-commit
  * Cosmetic change
  * Add _link
  * Fix missing link
  Co-authored-by: Kenji Miyake <31987104+kenji-miyake@users.noreply.github.com>
  Co-authored-by: Kenji Miyake <kenji.miyake@tier4.jp>
  * Fix tlr camera link name for xx1 (`#9 <https://github.com/autowarefoundation/sample_sensor_kit_launch/issues/9>`_)
  * update README.md
  * fix build depends
  * fix files
  * apply pre-commit
  * fix package.xml
  * remove README for now
  Co-authored-by: mitsudome-r <ryohsuke.mitsudome@tier4.jp>
  Co-authored-by: Akihito Ohsato <aohsato@gmail.com>
  Co-authored-by: Daisuke Nishimatsu <42202095+wep21@users.noreply.github.com>
  Co-authored-by: Yukihiro Saito <yukky.saito@gmail.com>
  Co-authored-by: Frederik Beaujean <72439809+fred-apex-ai@users.noreply.github.com>
  Co-authored-by: Esteve Fernandez <esteve@apache.org>
  Co-authored-by: Takamasa Horibe <horibe.takamasa@gmail.com>
  Co-authored-by: Taichi Higashide <taichi.higashide@tier4.jp>
  Co-authored-by: hiroyuki obinata <58019445+obi-t4@users.noreply.github.com>
  Co-authored-by: hosokawa <hosokawa@sng-3f-ros2-eval.l.sng.tier4.jp>
  Co-authored-by: HOSOKAWA Ikuto <hosokawa.ikuto@gmail.com>
  Co-authored-by: wep21 <border_goldenmarket@yahoo.co.jp>
  Co-authored-by: Autoware <autoware@tier4.jp>
  Co-authored-by: Kazuki Miyahara <kmiya@outlook.com>
  Co-authored-by: tkimura4 <tomoya.kimura@tier4.jp>
  Co-authored-by: autoware <autoware@example.com>
  Co-authored-by: Takagi, Isamu <isamu.takagi@tier4.jp>
  Co-authored-by: hiro-ya-iv <30652835+hiro-ya-iv@users.noreply.github.com>
  Co-authored-by: YamatoAndo <yamato.ando@gmail.com>
  Co-authored-by: Hiroki OTA <hiroki.ota@tier4.jp>
  Co-authored-by: Kosuke Takeuchi <kosuke.tnp@gmail.com>
  Co-authored-by: autoware-iv-sync-ci[bot] <87871706+autoware-iv-sync-ci[bot]@users.noreply.github.com>
  Co-authored-by: taichiH <azumade.30@gmail.com>
  Co-authored-by: Hiroaki ISHIKAWA <hiroaki.ishikawa@tier4.jp>
  Co-authored-by: Takeshi Miura <57553950+1222-takeshi@users.noreply.github.com>
  Co-authored-by: Keisuke Shima <19993104+KeisukeShima@users.noreply.github.com>
  Co-authored-by: Satoshi OTA <44889564+satoshi-ota@users.noreply.github.com>
  Co-authored-by: Shinnosuke Hirakawa <8327162+0x126@users.noreply.github.com>
* Contributors: Daisuke Nishimatsu, Kenji Miyake
