#!/usr/bin/env python3
"""Regenerate the roll-selection / lead-in section of Sinter Raster Continuous and derive its mock test variant.

    python3 scripts/tracking/gen_continuous_objectives.py

Edits src/sinter_robot_hw/objectives/sinter_raster_continuous.xml in place (only the generated section, the
Parameters additions and the TrackRasterWithSun lift/hold ports) and writes
src/sinter_robot_mock/objectives/sinter_raster_continuous_mock.xml (RealSense plane fit replaced by a synthetic
3-line path in front of the lens; simulated sun; no ephemeris seed). Keep hand edits outside the generated section.
"""
import re
import xml.dom.minidom
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
HW = REPO / "src/sinter_robot_hw/objectives/sinter_raster_continuous.xml"
MOCK = REPO / "src/sinter_robot_mock/objectives/sinter_raster_continuous_mock.xml"
BEGIN = "<!--BEGIN GENERATED: roll selection + lead-in (scripts/tracking/gen_continuous_objectives.py)-->"
END = "<!--END GENERATED-->"
ROLLS = [("0 deg", "0;0;0;1"), ("45 deg", "0;0;0.382683;0.92388"), ("-45 deg", "0;0;-0.382683;0.92388"),
         ("90 deg", "0;0;0.707107;0.707107"), ("-90 deg", "0;0;-0.707107;0.707107"),
         ("135 deg", "0;0;0.92388;0.382683"), ("-135 deg", "0;0;-0.92388;0.382683"), ("180 deg", "0;0;1;0")]
JTAC = ('controller_names="joint_trajectory_admittance_controller" '
        'controller_action_server="/joint_trajectory_admittance_controller/follow_joint_trajectory" '
        'execution_pipeline="jtac" joint_group_name="manipulator"')


def roll_block(name, q, pad):
    return f'''        <Decorator ID="ForceSuccess" name="Roll {name}">
          <Control ID="Sequence">
            <SubTree ID="Raster Reachable With Roll" _collapsed="true" raster_path="{{raster_check_points}}" roll_xyzw="{q}" planning_scene_msg="{{planning_scene}}" rolled_orientation="{{cand_orientation}}" min_dexterity="{{cand_point_dexterity}}" />
            <Action ID="OffsetPosesAlongAxis" poses="{{raster_path}}" axis_orientation="{{cand_orientation}}" distance="{{lift_m}}" include_original="false" first_only="true" poses_out="{{cand_lifted_start}}" first_out="{{cand_lifted_start_pose}}" />
            <Action ID="ComputeInverseKinematics" target_poses="{{cand_lifted_start}}" tip_links="ee_focal_point_link" planning_scene_msg="{{planning_scene}}" planning_group_name="manipulator" ik_timeout_s="0.2" link_padding="{pad}" solution="{{cand_seed}}" />
            <Action ID="SimulateRasterScan" seed_joint_state="{{cand_seed}}" path="{{raster_path}}" orientation="{{cand_orientation}}" lift_m="{{lift_m}}" scan_speed_mps="{{scan_speed_mps}}" lift_speed_mps="{{lift_speed_mps}}" step_m="0.002" damping="0.005" max_residual_m="0.0005" max_joint_speed_rps="0.3" acquire_margin_s="30.0" sun_feedforward="{{sun_feedforward}}" latitude_deg="{{latitude_deg}}" north_heading_deg="{{north_heading_deg}}" sun_feedforward_scale="{{sun_feedforward_scale}}" planning_group="manipulator" tip_link="ee_focal_point_link" dexterity_length_m="0.4" min_dexterity="{{cand_dexterity}}" trajectory="{{cand_trajectory}}" />
            <Action ID="ValidateTrajectory" planning_scene_msg="{{planning_scene}}" planning_group_name="manipulator" joint_trajectory_msg="{{cand_trajectory}}" joint_space_step="0.05" cartesian_space_step="0.02" />
            <Action ID="KeepBestRoll" candidate_score="{{cand_dexterity}}" candidate_orientation="{{cand_orientation}}" candidate_joint_state="{{cand_seed}}" require_joint_state="true" candidate_name="roll {name}" best_score_in="{{best_dexterity}}" best_score_out="{{best_dexterity}}" best_orientation="{{sun_orientation}}" best_joint_state="{{best_scan_seed}}" best_name="{{best_roll_name}}" />
          </Control>
        </Decorator>'''


def generated(pad="0.01"):
    rolls = "\n".join(roll_block(n, q, pad) for n, q in ROLLS)
    return f'''{BEGIN}
      <!--Check points for every roll: the raster extremes AND the same points lifted lift_m along the lens axis (the tracker lifts there while acquiring, during clouds and before stopping). The lens axis is the aligned one, the same for every roll.-->
      <Action ID="UnpackPoseStampedMessage" pose_stamped="{{aligned_focal_pose}}" pose="{{aligned_focal_msg}}" />
      <Action ID="UnpackPoseMessage" pose="{{aligned_focal_msg}}" orientation="{{aligned_orientation}}" />
      <Action ID="OffsetPosesAlongAxis" poses="{{raster_extremes}}" axis_orientation="{{aligned_orientation}}" distance="{{lift_m}}" include_original="true" first_only="false" poses_out="{{raster_check_points}}" first_out="{{unused_pose}}" />
      <Action ID="GetCurrentPlanningScene" planning_scene_msg="{{planning_scene}}" />
      <!--The roll score below rehearses a scan with the roll HELD; gain_roll > 0 lets the tracker roll away from that configuration.-->
      <Decorator ID="Precondition" if="gain_roll &gt; 0.0" else="SUCCESS">
        <Action ID="LogMessage" message="gain_roll &gt; 0: the tracker will roll the lens during the scan, so the roll selection's simulated scan (roll held) is not what runs. Keep gain_roll 0 unless testing roll." log_level="warn" />
      </Decorator>
      <Control ID="Sequence" name="Pick the lens roll whose simulated scan stays farthest from a singularity">
        <!--Per roll (nearest the current roll first): IK + collisions at the check points (cheap prefilter); IK (with collisions) at the LIFTED start = the scan seed; SimulateRasterScan rehearses descend -> raster -> lift from that seed with the controller's damped least squares and the sun-turning orientation (fails on missed targets, joint limits or joint speed) and its trajectory is collision-checked (ValidateTrajectory); the score is the worst dexterity along that simulated scan, i.e. of the configuration that will actually run. A farther roll must be 25 % better (KeepBestRoll). The best seed is the lead-in target.-->
        <Action ID="Script" code="best_dexterity := -1.0" />
{rolls}
        <Decorator ID="Precondition" if="best_dexterity &lt; 0.0" else="SUCCESS">
          <Control ID="Sequence" name="No roll passes">
            <Action ID="LogMessage" message="Sinter raster rejected: at every lens roll either IK fails at a raster or lifted point, or the simulated scan misses the path, hits a joint limit or speed limit, or collides (see the SimulateRasterScan / ValidateTrajectory messages). Move the bed toward the base / away from the sun, shrink raster_size_m, or sinter nearer solar noon." log_level="error" />
            <Action ID="AlwaysFailure" />
          </Control>
        </Decorator>
        <Decorator ID="Precondition" if="best_dexterity &lt; min_roll_dexterity" else="SUCCESS">
          <Control ID="Sequence" name="Best roll too close to a singularity">
            <Action ID="LogMessage" message="Sinter raster too close to a singularity: the best roll's simulated scan has dexterity below min_roll_dexterity (see the KeepBestRoll scores)." log_level="error" />
            <Action ID="AlwaysFailure" />
          </Control>
        </Decorator>
      </Control>
      <!--Lead-in: planned, collision-checked move to the scored seed configuration = raster point 0 lifted lift_m along the lens axis (defocused). The tracker starts lifted (start_lifted), acquires there, then descends.-->
      <SubTree ID="Move to Joint State" _collapsed="true" target_joint_state="{{best_scan_seed}}" {JTAC} link_padding="{pad}" seed="0" keep_orientation="false" keep_orientation_tolerance="0.05" velocity_scale_factor="{{align_velocity_scale}}" acceleration_scale_factor="1.0" />
      {END}'''


def set_param(code, key, value, force=False):
    """Add `key := value` if missing; replace an existing value only with force (keeps hand tuning)."""
    if re.search(rf"\b{key} :=", code):
        return re.sub(rf"\b{key} := [^;\"]*", f"{key} := {value}", code) if force else code
    return code + f"; {key} := {value}"


def patch_hw(s):
    # Generated section: from the reachability comment (or a previous BEGIN marker) through the lead-in.
    if BEGIN in s:
        a = s.index(BEGIN)
        b = s.index(END) + len(END)
    else:
        a = s.index("<!--Reachability check before going near the surface")
        b = s.index("<!--Continuous scan:")
    s = s[:a] + generated() + "\n      " + s[b:].lstrip()
    # Parameters
    m = re.search(r'name="Parameters"\s+code="([^"]*)"', s)
    code = m.group(1)
    for k, v in (("lift_m", "0.08"), ("lift_speed_mps", "0.003"), ("scan_tolerance_rad", "0.004"),
                 ("sun_feedforward", "true"), ("sun_feedforward_scale", "1.0")):
        code = set_param(code, k, v)
    s = s[:m.start(1)] + code + s[m.end(1):]
    # Tracker ports
    t0 = s.index('ID="TrackRasterWithSun"')
    t1 = s.index("/>", t0)
    tr = s[t0:t1]
    tr = re.sub(r'lift_speed_mps="[^"]*"', 'lift_speed_mps="{lift_speed_mps}"', tr)
    # The simulator and the tracker must use the same feedforward.
    tr = re.sub(r'sun_feedforward="[^"]*"', 'sun_feedforward="{sun_feedforward}"', tr)
    tr = re.sub(r'sun_feedforward_scale="[^"]*"', 'sun_feedforward_scale="{sun_feedforward_scale}"', tr)
    for k, v in (("start_lifted", "true"), ("scan_tolerance_rad", "{scan_tolerance_rad}"), ("hold_lift_s", "0.1"),
                 ("resume_tolerance_m", "0.0005")):
        tr = re.sub(rf'\s{k}="[^"]*"', "", tr)
        tr = tr.rstrip() + f'\n          {k}="{v}"'
    tr = re.sub(r"\n[ \t]*(?=\n)", "", tr)  # drop whitespace-only lines left by earlier runs
    # The simulator scores the scan the tracker will run: both must read the same blackboard keys.
    for k in ("lift_m", "lift_speed_mps", "scan_speed_mps", "latitude_deg", "north_heading_deg", "sun_feedforward",
              "sun_feedforward_scale"):
        assert f'{k}="{{{k}}}"' in tr, f"TrackRasterWithSun {k} must be {{{k}}} (as in SimulateRasterScan)"
    return s[:t0] + tr.rstrip() + "\n        " + s[t1:]


def derive_mock(hw):
    s = hw.replace('main_tree_to_execute="Sinter Raster Continuous"', 'main_tree_to_execute="Sinter Raster Continuous Mock"')
    s = s.replace('ID="Sinter Raster Continuous"', 'ID="Sinter Raster Continuous Mock"')
    s = re.sub(r'_description="[^"]*"', '_description="Simulation test of Sinter Raster Continuous (generated by scripts/tracking/gen_continuous_objectives.py from the real objective; keep in sync by re-running it). Same roll selection (IK prefilter, lifted-start seed, SimulateRasterScan, ValidateTrajectory, KeepBestRoll), lifted lead-in to the scored seed, continuous tracking with the hold band, settle gate, defocus lift and projected sun feedforward, and the same failure branch. Differences: the RealSense scan and plane fit are replaced by a 3-line 30 x 8 mm path built in front of the lens at BeforeSinter; the simulated sun is re-centred before alignment; no ephemeris seed; link padding 0 (mock start pose)."', s, count=1)
    # Replace the RealSense block (Scan_Initial .. end of 'Generate scanning waypoints') with nothing; build the
    # synthetic path after BeforeSinter instead.
    a = s.index('waypoint_name="Scan_Initial"')
    a = s.rindex("<SubTree", 0, a)
    gen_end = s.index("</Control>", s.index('name="Generate scanning waypoints"')) + len("</Control>")
    s = s[:a] + s[gen_end:]
    pts = [(0, 0), (0.03, 0), (0.03, 0.004), (0, 0.004), (0, 0.008), (0.03, 0.008)]
    build = "\n".join(
        f'''        <Action ID="CreatePoseStamped" reference_frame="ee_focal_point_link" position_xyz="{x};{y};0" orientation_xyzw="0;0;0;1" pose_stamped="{{pt}}" />
        <Action ID="TransformPoseFrame" input_pose="{{pt}}" target_frame_id="world" output_pose="{{pt_world}}" />
        <Action ID="AddPoseStampedToVector" input="{{pt_world}}" vector="{{raster_path}}" />''' for x, y in pts)
    synth = f'''<SubTree ID="Reset Sun To Camera" _collapsed="true" />
      <Action ID="WaitForDuration" delay_duration="1.0" />
      <Control ID="Sequence" name="Synthetic raster (mock): 3 lines of 30 mm, 4 mm apart, in front of the lens, in world">
        <Action ID="ResetPoseStampedVector" vector="{{raster_path}}" />
{build}
        <Action ID="VisualizePath" path="{{raster_path}}" show_poses="false" />
      </Control>
      '''
    anchor = s.index("<!--Rotate lens to align to sun")
    s = s[:anchor] + synth + s[anchor:]
    m = re.search(r'name="Parameters"\s+code="([^"]*)"', s)
    code = set_param(set_param(m.group(1), "use_ephemeris_seed", "false", force=True), "scan_speed_mps", "0.002",
                     force=True)
    s = s[:m.start(1)] + code + s[m.end(1):]
    s = s.replace('link_padding="0.01"', 'link_padding="0.0"')
    s = s.replace("{raster_extremes}", "{raster_path}")  # the synthetic path is small: check every point
    s = s.replace('<Metadata subcategory="Sinter" />', '<Metadata subcategory="Continuous Tracking" />')
    return s


if __name__ == "__main__":
    hw = patch_hw(HW.read_text())
    mock = derive_mock(hw)
    xml.dom.minidom.parseString(hw)  # parse both before writing either: a bad edit leaves the files untouched
    xml.dom.minidom.parseString(mock)
    HW.write_text(hw)
    MOCK.write_text(mock)
    print("wrote", HW, "and", MOCK)
