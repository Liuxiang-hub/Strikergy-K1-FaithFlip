"""Strikergy K1 FaithFlip: standing backflip task for booster_deploy.

The policy plays its reference motion once. It observes the reference joint targets at the current frame, the
reference trunk orientation relative to the robot trunk, IMU angular velocity, joint state, last action and projected
gravity (122 values; layout in models/<model>/k1_backflip_meta.json, checked at start-up). Joint order = Booster SDK
order. High-risk motion: read the README safety notes before running it on a robot.
"""
from __future__ import annotations

import dataclasses
import json
import os
from dataclasses import MISSING

import numpy as np
import torch

from booster_deploy.controllers.base_controller import BaseController, Policy
from booster_deploy.controllers.controller_cfg import ControllerCfg, PolicyCfg
from booster_deploy.robots.k1 import K1_CFG
from booster_deploy.utils.isaaclab import math as lab_math
from booster_deploy.utils.isaaclab.configclass import configclass
from booster_deploy.utils.policy_runner import create_policy_runner
from booster_deploy.utils.registry import register_task

LAYOUT = [["command", [44]], ["motion_anchor_ori_b", [6]], ["base_ang_vel", [3]], ["joint_pos", [22]],
          ["joint_vel", [22]], ["actions", [22]], ["projected_gravity", [3]]]
# PD gains during the flip
STIFFNESS = [4.0, 4.0] + [20.0] * 8 + [80.0, 80.0, 80.0, 80.0, 50.0, 50.0] * 2
DAMPING = [0.25, 0.25] + [2.0] * 8 + [4.0, 4.0, 4.0, 4.0, 2.0, 2.0] * 2
EFFORT = [6.0, 6.0] + [14.0] * 8 + [68.0, 76.0, 38.3, 112.0, 38.3, 38.3] * 2  # torque limits (simulation only)


class K1BackflipPolicy(Policy):
    def __init__(self, cfg: "K1BackflipPolicyCfg", controller: BaseController):
        super().__init__(cfg, controller)
        self.cfg = cfg
        self.robot = controller.robot
        self.device = torch.device(cfg.device)
        here = lambda p: p if os.path.isabs(p) else os.path.join(self.task_path, p)  # noqa: E731
        meta = json.load(open(here(cfg.meta_path)))
        if meta["actor_obs_layout"] != LAYOUT or meta["actor_obs_dim"] != 122:
            raise RuntimeError(f"policy observation layout {meta['actor_obs_layout']} != {LAYOUT}")
        if len(meta["joint_names"]) != len(self.robot.cfg.joint_names):
            raise RuntimeError("joint count mismatch")
        self.model = create_policy_runner(here(cfg.checkpoint_path), self.device)
        self.default_joint_pos = torch.tensor(meta["default_joint_pos"], dtype=torch.float32, device=self.device)
        self.action_scale = torch.tensor(meta["action_scale"], dtype=torch.float32, device=self.device)
        m = np.load(here(os.path.join(os.path.dirname(cfg.meta_path), meta["motion_file"])))
        f = lambda a: torch.tensor(a, dtype=torch.float32, device=self.device)  # noqa: E731
        self.ref_joint_pos, self.ref_joint_vel = f(m["joint_pos"]), f(m["joint_vel"])
        self.ref_trunk_quat = f(m["body_quat_w"][:, 0])  # body 0 is the trunk (anchor)
        self.num_frames = self.ref_joint_pos.shape[0]
        self.robot.data.to(self.device)
        self.last_action = torch.zeros(22, dtype=torch.float32, device=self.device)
        self.frame = 0
        self.dt = controller.cfg.policy_dt
        self.t, self.crouch_from = 0.0, None
        self.sim_go, self.seen_go, self.waiting, self.wait_t = 0, 0, False, 0.0
        # the controllers read robot.joint_stiffness / joint_damping on every step, so the policy can switch them
        self.flip_kp, self.flip_kd = self.robot.joint_stiffness.clone(), self.robot.joint_damping.clone()
        self.crouch_kp = torch.tensor(K1_CFG.prepare_state.stiffness, dtype=torch.float32)
        self.crouch_kd = torch.tensor(K1_CFG.prepare_state.damping, dtype=torch.float32)

    def request_go(self) -> None:
        self.sim_go += 1

    def _go_count(self) -> int:
        counter = getattr(getattr(self.controller, "portal", None), "ball_toggle", None)
        return (counter.value if counter is not None else 0) + self.sim_go

    def reset(self) -> None:
        self.filt = None
        self.last_action.zero_()
        self.frame = self.cfg.start_frame
        self.waiting, self.wait_t = self.cfg.wait_frame >= 0, 0.0
        self.seen_go = self._go_count()
        if self.waiting:
            assert self.cfg.start_frame == self.cfg.wait_frame
            print("[k1_backflip] balancing; press D-pad up to flip", flush=True)
        self.t, self.crouch_from = 0.0, None
        self._align()
        if self.cfg.crouch_s > 0:  # a static PD crouch needs the stiff prepare gains: at the policy's 80 it fell in sim
            self._gains(self.crouch_kp, self.crouch_kd)

    def _gains(self, kp: torch.Tensor, kd: torch.Tensor) -> None:
        self.robot.joint_stiffness.copy_(kp)
        self.robot.joint_damping.copy_(kd)

    def _align(self) -> None:
        # express the robot's heading in the clip's frame: clip frame-0 yaw times the inverse of the robot's yaw now
        robot_yaw = lab_math.yaw_quat(self.robot.data.root_quat_w.reshape(1, 4))
        clip_yaw = lab_math.yaw_quat(self.ref_trunk_quat[0].reshape(1, 4))
        self.align = lab_math.quat_mul(clip_yaw, lab_math.quat_inv(robot_yaw)).reshape(4)

    def compute_observation(self) -> torch.Tensor:
        t = min(self.frame, self.num_frames - 1)
        q = self.robot.data.root_quat_w
        q_aligned = lab_math.quat_mul(self.align.reshape(1, 4), q.reshape(1, 4))
        rel = lab_math.quat_mul(lab_math.quat_inv(q_aligned), self.ref_trunk_quat[t].reshape(1, 4))
        ori = lab_math.matrix_from_quat(rel)[0, :, :2].reshape(-1)
        gravity = lab_math.quat_apply_inverse(q.reshape(1, 4), torch.tensor([[0.0, 0.0, -1.0]], device=self.device))[0]
        obs = torch.cat([self.ref_joint_pos[t], self.ref_joint_vel[t], ori, self.robot.data.root_ang_vel_b,
                         self.robot.data.joint_pos - self.default_joint_pos, self.robot.data.joint_vel,
                         self.last_action, gravity])
        if obs.numel() != 122 or not torch.isfinite(obs).all():
            raise RuntimeError("bad backflip observation")
        return obs.reshape(1, -1)

    def _crouch(self) -> torch.Tensor:
        """After A: from the X stand, ease into the clip's first pose (crouch_s), hold still (settle_s), with the stiff
        prepare gains; then switch to the flip gains. Sim (sim2sim_bf_xa.py) flips from this at the real latency."""
        if self.crouch_from is None:
            self.crouch_from = self.robot.data.joint_pos.clone()
        self.t += self.dt
        s = min(1.0, self.t / self.cfg.crouch_s)
        s = s * s * (3.0 - 2.0 * s)
        self.last_target = self.crouch_from + (self.default_joint_pos - self.crouch_from) * s
        if self.t >= self.cfg.crouch_s + self.cfg.settle_s:
            self._align()  # heading as the flip starts; gains ease to the flip's over ramp_frames in inference()
        return self.last_target

    def inference(self) -> torch.Tensor:
        if self.cfg.crouch_s > 0 and self.t < self.cfg.crouch_s + self.cfg.settle_s:
            return self._crouch()
        if self.frame >= self.num_frames and self.cfg.hold_at_motion_end:
            # clip done: keep the last targets (the landed stand), as sim2sim does for its 1 s check; B/X take over
            return self.last_target
        with torch.inference_mode():
            action = self.model(self.compute_observation()).reshape(-1)
        if action.numel() != 22 or not torch.isfinite(action).all():
            raise RuntimeError("bad backflip action")
        self.last_action.copy_(action)
        if self.waiting:
            # balance at wait_frame until the go signal (D-pad Up on the robot; request_go() or go_after_s)
            self.wait_t += self.dt
            if self._go_count() != self.seen_go or (self.cfg.go_after_s >= 0 and self.wait_t >= self.cfg.go_after_s):
                self.waiting = False
                print(f"[k1_backflip] go after {self.wait_t:.1f} s of balancing", flush=True)
        else:
            self.frame += 1
        self.last_target = self.default_joint_pos + action * self.action_scale
        if self.cfg.land_filter_alpha < 1.0 and self.frame >= self.cfg.land_filter_frame:
            # low-pass the joint targets from touchdown on (the policy still sees its raw action)
            if self.filt is None:
                self.filt = self.last_target.clone()
            a = self.cfg.land_filter_alpha
            self.filt = a * self.last_target + (1.0 - a) * self.filt
            self.last_target = self.filt.clone()
        if self.cfg.crouch_s > 0 and self.frame <= self.cfg.ramp_frames:
            # ease the gains from the stand to the flip gains
            w = self.frame / self.cfg.ramp_frames
            self._gains(self.crouch_kp + (self.flip_kp - self.crouch_kp) * w, self.crouch_kd + (self.flip_kd - self.crouch_kd) * w)
        if self.frame >= self.num_frames and self.cfg.stop_at_motion_end:
            self.controller.stop()
        return self.last_target


@configclass
class K1BackflipPolicyCfg(PolicyCfg):
    constructor = K1BackflipPolicy
    # FaithFlip: K1_FLIP_MODEL=hero (default, hero landing) | standard (stable two-foot landing)
    checkpoint_path: str = f"models/{os.environ.get('K1_FLIP_MODEL', 'hero')}/k1_backflip.onnx"
    meta_path: str = f"models/{os.environ.get('K1_FLIP_MODEL', 'hero')}/k1_backflip_meta.json"
    stop_at_motion_end: bool = False  # True would drop into exit_mode (damping) right after landing
    hold_at_motion_end: bool = True
    start_frame: int = 12  # the reference frame the robot stands in while waiting
    wait_frame: int = 12  # hold the reference here until the go signal; -1 = play straight through
    # go signal: D-pad Up when the remote service exposes portal.ball_toggle; otherwise (stock booster_deploy, MuJoCo)
    # K1_FLIP_GO_AFTER=<s> starts the flip after that many seconds of balancing (-1 = wait for D-pad Up)
    go_after_s: float = float(os.environ.get("K1_FLIP_GO_AFTER", "-1"))
    ramp_frames: int = 25  # policy steps (0.5 s) to go from the crouch gains to the flip gains
    crouch_s: float = 0.0  # seconds to ease from the X stand into the first frame (0 = the policy takes over at once)
    settle_s: float = 0.0  # seconds to hold still before the policy takes over
    enable_safety_fallback: bool = False  # the trunk turns upside down on purpose
    land_filter_alpha: float = 0.5  # <1: low-pass the joint targets from land_filter_frame on (1 = off)
    land_filter_frame: int = 100  # reference frame (50 fps) from which the landing filter is on


# X = the standing pose and gains the robot holds before the flip
X_STAND = [0.0, 0.0, 0.0, -1.4, 0.0, -0.4, 0.0, 1.4, 0.0, 0.4,
           -0.17, 0.0, 0.0, 0.41, -0.17, 0.0, -0.17, 0.0, 0.0, 0.41, -0.17, 0.0]
STAND_KP = [4.0, 4.0] + [10.0] * 8 + [80.0, 80.0, 80.0, 80.0, 50.0, 50.0] * 2
STAND_KD = [0.25, 0.25] + [1.0] * 8 + [4.0, 4.0, 4.0, 4.0, 2.0, 2.0] * 2
BF_DEFAULT = [0.0, 0.0, 0.0, -1.4, 0.0, -0.4, 0.0, 1.4, 0.0, 0.4,
              -0.4, 0.0, 0.0, 0.8, -0.4, 0.0, -0.4, 0.0, 0.0, 0.8, -0.4, 0.0]


# X takes 4 s to reach the stand where the robot config has the field (stock booster_deploy does not)
_PREPARE = {"prepare_duration_s": 4.0} if "prepare_duration_s" in {f.name for f in dataclasses.fields(K1_CFG)} else {}


@configclass
class K1BackflipControllerCfg(ControllerCfg):
    policy_dt: float = 0.02
    robot = K1_CFG.replace(  # type: ignore
        prepare_mode="standing",
        **_PREPARE,
        default_joint_pos=BF_DEFAULT,
        prepare_state=K1_CFG.prepare_state.replace(joint_pos=X_STAND, stiffness=STAND_KP, damping=STAND_KD),
        joint_stiffness=STIFFNESS,
        joint_damping=DAMPING,
        effort_limit=EFFORT,
    )
    policy: K1BackflipPolicyCfg = K1BackflipPolicyCfg()

    def __post_init__(self):
        super().__post_init__()
        self.booster.exit_mode = "damping"
        self.mujoco.init_pos = [0.0, 0.0, 0.52]


register_task("k1_backflip", K1BackflipControllerCfg())
