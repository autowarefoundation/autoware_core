^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_motion_utils
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.1.0 (2025-05-01)
------------------
* fix(autoware_path_optimizer): incorrect application of input velocity due to badly mapping output trajectory to input trajectory (`#355 <https://github.com/youtalk/autoware_core/issues/355>`_)
  * changes to avoid improper mapping
  * Update common/autoware_motion_utils/include/autoware/motion_utils/trajectory/trajectory.hpp
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
  ---------
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* refactor(autoware_motion_utils): rewrite using modern C++ without API breakage (`#348 <https://github.com/youtalk/autoware_core/issues/348>`_)
* Contributors: Arjun Jagdish Ram, Yutaka Kondo

1.0.0 (2025-03-31)
------------------

0.3.0 (2025-03-21)
------------------
* chore: fix versions in package.xml
* test(autoware_motion_utils): add tests for missed lines (`#275 <https://github.com/autowarefoundation/autoware.core/issues/275>`_)
  Co-authored-by: Yutaka Kondo <yutaka.kondo@youtalk.jp>
* feat: porting `autoware_motion_utils` from universe to core (`#184 <https://github.com/autowarefoundation/autoware.core/issues/184>`_)
  * add(autoware_motion_utils): ported as follows (see below):
  * From `autoware.universe/common` to `autoware.core/common`
  * The history can be traced via:
  https://github.com/autowarefoundation/autoware.universe/tree/3274695847dfc76153bdc847e28b66821e16df60/common/autoware_motion_utils
  * fix(package.xml): set the version to `0.0.0` as the initial port
  ---------
* Contributors: Junya Sasaki, NorahXiong, mitsudome-r
