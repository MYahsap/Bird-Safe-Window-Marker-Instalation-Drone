#!/usr/bin/env bash
set -euo pipefail

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

python3 - "$repo_root" <<'PY'
import ast
import math
import pathlib
import sys
import xml.etree.ElementTree as ET

root = pathlib.Path(sys.argv[1])
xml_files = [
    root / "models/x650/model.config",
    root / "models/x650/model.sdf",
    root / "worlds/x650_world.sdf",
    root / "ros2_ws/src/x650_sim/package.xml",
]
for path in xml_files:
    ET.parse(path)
    print(f"XML OK: {path.relative_to(root)}")

python_files = list((root / "ros2_ws/src/x650_sim").rglob("*.py"))
for path in python_files:
    ast.parse(path.read_text(), filename=str(path))
    print(f"Python syntax OK: {path.relative_to(root)}")

setup_source = root / "ros2_ws/src/x650_sim/setup.py"
setup_globals = {"__file__": str(setup_source), "__name__": "x650_setup_validation"}
# Inspect the same source-relative calculation without invoking setuptools.
package_source = setup_source.resolve().parent
asset_root = package_source.parents[2]
assert (asset_root / "models/x650/model.sdf").is_file(), asset_root
assert (asset_root / "worlds/x650_world.sdf").is_file(), asset_root
print("Packaging paths OK: model and world assets resolve from setup.py")

model = ET.parse(root / "models/x650/model.sdf").getroot()
poses = {}
for link in model.findall(".//model/link"):
    if link.attrib.get("name", "").startswith("rotor_"):
        values = [float(v) for v in link.findtext("pose").split()]
        poses[link.attrib["name"]] = values[:3]
assert len(poses) == 4, poses
p0, p1 = poses["rotor_0"], poses["rotor_1"]
wheelbase = math.dist(p0[:2], p1[:2])
assert abs(wheelbase - 0.65) < 1e-4, wheelbase
plugins = model.findall(".//plugin[@name='gz::sim::systems::MulticopterMotorModel']")
assert len(plugins) == 4
assert {int(p.findtext("motorNumber")) for p in plugins} == {0, 1, 2, 3}
print(f"Geometry OK: wheelbase={wheelbase:.4f} m, four motor plugins")

servo_pose = [
    float(v)
    for v in model.find(".//model/link[@name='base_link']/visual[@name='manipulator_servo']/pose").text.split()
]
joint_pose = [
    float(v)
    for v in model.find(".//model/joint[@name='manipulator_tilt_joint']/pose").text.split()
]
manipulator_pose = [
    float(v)
    for v in model.find(".//model/link[@name='manipulator_link']/pose").text.split()
]
assert servo_pose[:3] == [0.15, 0.0, -0.10], servo_pose
assert joint_pose[:3] == servo_pose[:3], (joint_pose, servo_pose)
assert manipulator_pose[:3] == [0.15, 0.0, -0.35], manipulator_pose
joint_limit = model.find(
    ".//model/joint[@name='manipulator_tilt_joint']/axis/limit"
)
lower_limit = float(joint_limit.findtext("lower"))
upper_limit = float(joint_limit.findtext("upper"))
assert abs(lower_limit + math.pi) < 1e-5, lower_limit
assert abs(upper_limit - math.pi / 2.0) < 1e-5, upper_limit
force_plugins = model.findall(
    ".//plugin[@name='gz::sim::systems::ApplyJointForce']"
)
state_plugins = model.findall(
    ".//plugin[@name='gz::sim::systems::JointStatePublisher']"
)
assert len(force_plugins) == 1, len(force_plugins)
assert len(state_plugins) == 1, len(state_plugins)
assert not model.findall(
    ".//plugin[@name='gz::sim::systems::JointPositionController']"
)
controller_source = (
    root / "ros2_ws/src/x650_sim/x650_sim/manipulator_controller.py"
).read_text()
assert 'self.declare_parameter("initial_target_angle", -math.pi / 2.0)' in controller_source
assert 'self.declare_parameter("min_angle", -math.pi)' in controller_source
assert 'self.declare_parameter("stabilize_forward", True)' in controller_source
assert "self.forward_joint_angle + self.pitch_compensation_gain * pitch" in controller_source
assert '"/fmu/out/vehicle_attitude"' in controller_source
print("Manipulator geometry OK: pivot at x=0.15 m, z=-0.10 m")

launch_source = (root / "ros2_ws/src/x650_sim/launch/sim.launch.py").read_text()
expected_tuning = {
    '"PX4_PARAM_CA_ROTOR0_PX": "0.13"',
    '"PX4_PARAM_CA_ROTOR0_PY": "0.22"',
    '"PX4_PARAM_CA_ROTOR1_PY": "-0.20"',
    '"PX4_PARAM_MC_PITCHRATE_I": "0.20"',
    '"PX4_PARAM_MC_PITCHRATE_D": "0.003"',
    '"PX4_PARAM_MPC_XY_P": "0.95"',
    '"PX4_PARAM_MPC_Z_P": "1.00"',
}
missing_tuning = sorted(value for value in expected_tuning if value not in launch_source)
assert not missing_tuning, missing_tuning
print("PX4 baseline OK: FRD allocation signs and default gains restored")
PY

echo "All offline checks passed."
