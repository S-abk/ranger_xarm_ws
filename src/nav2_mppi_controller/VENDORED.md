# Vendored nav2_mppi_controller

This is Nav2's `nav2_mppi_controller` at tag
[1.3.12](https://github.com/ros-navigation/navigation2/tree/1.3.12/nav2_mppi_controller)
(the version installed with ROS 2 Jazzy here), overlaid under the same
package name, with one addition: the `FourWIS` motion model for the Ranger
Mini V3's four-wheel independent steering
(`include/nav2_mppi_controller/four_wis_motion_model.hpp`, registered in
`src/optimizer.cpp`'s `setMotionModel`).

It has to be a copy rather than a plugin: Jazzy's MPPI chooses its motion
model from a fixed list in `Optimizer::setMotionModel`, not through
pluginlib. Everything else is upstream, unchanged, under upstream's licence
(`LICENSE.md`). Upstream's `test/`, `benchmark/` and `media/` are left out,
and the test block is removed from `CMakeLists.txt`.

Select it with `motion_model: "FourWIS"`; parameters live under
`FourWISConstraints` (see the header). Because the package name is the
same, whichever workspace is sourced last provides the controller: check
the controller_server log for "FourWIS motion model".

To move to a newer Nav2, copy the new upstream package over this one and
re-apply the header and the three-line registration.
