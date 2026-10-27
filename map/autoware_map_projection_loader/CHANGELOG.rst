^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_map_projection_loader
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* feat(component_interface_specs): use template type in get_qos function (`#364 <https://github.com/autowarefoundation/autoware_core/issues/364>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* feat(map_projection_loader): add scale_factor and remove altitude (`#340 <https://github.com/autowarefoundation/autoware_core/issues/340>`_)
* Contributors: Takagi, Isamu, Yamato Ando

1.10.0 (2026-09-28)
-------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* feat(map): add node designs for the map nodes (`#1413 <https://github.com/autowarefoundation/autoware_core/issues/1413>`_)
  * feat(map): add node designs for the map nodes
  Describe the interfaces, parameters and processes of the pointcloud and
  lanelet2 map loaders, the map hash generator, the lanelet2 map
  visualizer and the map projection loader as system design node files.
  * fix(map): correct description formatting in Lanelet2MapLoader.node.yaml
  * refactor(map): rename node designs to match the node class names
  PointCloudMapLoader and Lanelet2MapVisualization follow the entity
  naming convention derived from the C++ class names, as in `#1335 <https://github.com/autowarefoundation/autoware_core/issues/1335>`_.
  * fix(map): declare the MapHashGenerator external API interfaces as remap targets
  The map hash publisher and the lanelet XML server carry the fixed API
  names as remap_target, so a system design connects them like any other
  port and the launcher remaps the node-side names.
  * fix(map): update description for map_projector_info to clarify its purpose
  ---------
* fix(map): declare the dependencies these packages use (`#1370 <https://github.com/autowarefoundation/autoware_core/issues/1370>`_)
  Each of these packages uses a package it never declares. Either it includes a
  header of that package, or it names a symbol of it while the header arrives
  through another dependency. Both build today only because some declared
  dependency re-exports the owner, so a change in an unrelated repository can
  break them without anything here changing.
  The tag follows where the dependency is used: a use in an installed header or
  in code compiled into the library takes <depend>, one reached only from test/
  takes <test_depend>. System libraries are named by the rosdep key this
  workspace already prefers.
* refactor(`autoware_map_projection_loader`): keep core logic free of logging (`#1310 <https://github.com/autowarefoundation/autoware_core/issues/1310>`_)
  * refactor: keep core logic free of logging
  * Emit the input paths from the node instead of std::cout
  * Drop the deprecated lowercase "local" projector type (now rejected).
  * fix: `README.md`
  * fix: separate source file into that of core/ROS-node logic
  ---------
  Co-authored-by: Tran Huu Nhat Huy <29034232+TranHuuNhatHuy@users.noreply.github.com>
* feat(map_projection_loader): apply `agnocast_wrapper::Node` to `map_projection_loader` (`#1202 <https://github.com/autowarefoundation/autoware_core/issues/1202>`_)
  * apply agnocast_wrapper::Node
  * delete unnecessary comments
  ---------
* Contributors: Junya Sasaki, Koichi Imai, Mete Fatih Cırıt, Taekjin LEE, github-actions

1.9.0 (2026-06-24)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* test(autoware_map_projection_loader): add gtest for load_info_from_yaml and load_map_projector_info (`#1140 <https://github.com/autowarefoundation/autoware_core/issues/1140>`_)
  Add a direct C++ gtest suite mirroring test_load_info_from_lanelet2_map.cpp
  that writes temporary YAML files and asserts the full MapProjectorInfo
  message contents per projector_type, closing the high-severity coverage
  gap previously exercised only indirectly by the launch_test files.
  Covered behaviors:
  - MGRS, LocalCartesianUTM, LocalCartesian, Local, TransverseMercator full
  message contents (vertical_datum, mgrs_grid, map_origin, scale_factor)
  - altitude always forced to 0.0
  - scale_factor defaulting matrix (TM default 0.9996 vs explicit override;
  MGRS/LocalCartesianUTM -> 0.9996; Local/LocalCartesian -> 1.0)
  - deprecated lowercase "local" -> Local remapping
  - invalid projector_type and scale_factor <= 0.0 throwing std::runtime_error
  - load_map_projector_info yaml-takes-precedence-over-lanelet2 selection and
  the no-files-found throw
  No public API changes; tests only.
  Refs: `autowarefoundation/autoware_core#1096 <https://github.com/autowarefoundation/autoware_core/issues/1096>`_
* Contributors: Yutaka Kondo, github-actions

1.8.0 (2026-05-01)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* refactor(autoware_core): add USE_SCOPED_HEADER_INSTALL_DIR to map packages (`#976 <https://github.com/mitsudome-r/autoware_core/issues/976>`_)
  Co-authored-by: github-actions <github-actions@github.com>
  Co-authored-by: Junya Sasaki <j2sasaki1990@gmail.com>
* chore(common, map): remove unused lanelet2_extension header (`#903 <https://github.com/mitsudome-r/autoware_core/issues/903>`_)
  * remove unused lanelet2_extension in map component
  * remove unused lanelet2_extension in common component
  ---------
* chore(planning, misc): remove unused header includes (`#840 <https://github.com/mitsudome-r/autoware_core/issues/840>`_)
* Contributors: Mamoru Sobue, Sarun MUKDAPITAK, Vishal Chauhan, github-actions

1.7.0 (2026-02-14)
------------------

1.6.0 (2025-12-30)
------------------
* Merge remote-tracking branch 'origin/main' into tmp/bot/bump_version_base
* chore: jazzy-porting: fix test depend launch-test missing (`#738 <https://github.com/autowarefoundation/autoware_core/issues/738>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* Contributors: github-actions, 心刚

1.5.0 (2025-11-16)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: replace `ament_auto_package` to `autoware_ament_auto_package` (`#700 <https://github.com/autowarefoundation/autoware_core/issues/700>`_)
  * replace ament_auto_package to autoware_ament_auto_package
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* chore: update maintainer (`#637 <https://github.com/autowarefoundation/autoware_core/issues/637>`_)
  * chore: update maintainer
  remove Takeshi Ishita
  * chore: update maintainer
  remove Kento Yabuuchi
  * chore: update maintainer
  remove Shintaro Sakoda
  * chore: update maintainer
  remove Ryu Yamamoto
  ---------
* chore: bump version (1.4.0) and update changelog (`#608 <https://github.com/autowarefoundation/autoware_core/issues/608>`_)
* Contributors: Mete Fatih Cırıt, Motz, Yutaka Kondo, mitsudome-r

1.4.0 (2025-08-11)
------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix(autoware_geography_utils): disable tests for egm2008-1 (`#593 <https://github.com/autowarefoundation/autoware_core/issues/593>`_)
* chore: bump version to 1.3.0 (`#554 <https://github.com/autowarefoundation/autoware_core/issues/554>`_)
* Contributors: Ryohsuke Mitsudome

1.3.0 (2025-06-23)
------------------
* fix: to be consistent version in all package.xml(s)
* chore: bump up version to 1.1.0 (`#462 <https://github.com/autowarefoundation/autoware_core/issues/462>`_) (`#464 <https://github.com/autowarefoundation/autoware_core/issues/464>`_)
* feat(component_interface_specs): use template type in get_qos function (`#364 <https://github.com/autowarefoundation/autoware_core/issues/364>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* feat(map_projection_loader): add scale_factor and remove altitude (`#340 <https://github.com/autowarefoundation/autoware_core/issues/340>`_)
* Contributors: Takagi, Isamu, Yamato Ando, Yutaka Kondo, github-actions

1.0.0 (2025-03-31)
------------------
* chore: update version in package.xml
* feat: move autoware_map_projection_loader package from Autoware Universe  (`#125 <https://github.com/autowarefoundation/autoware_core/issues/125>`_)
* Contributors: Ryohsuke Mitsudome
