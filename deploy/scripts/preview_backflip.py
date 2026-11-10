"""Headless MuJoCo preview of the K1 FaithFlip task in booster_deploy.

The stock `deploy.py --mujoco` passes the policy's targets (robot/SDK joint order) straight to MuJoCo, whose K1 joint
order differs; this script maps the order, adds the robot's actuator command latency (the policy expects it), holds
the X stand, starts the policy, gives the go signal after `--wait` seconds and reports the rotation. It is a preview,
not a validation: no motor torque-speed model, no friction or terrain variation.

python scripts/preview_backflip.py --model hero|standard [--wait 2] [--video flip.mp4]
"""
from __future__ import annotations

import argparse
import importlib
import os
import subprocess
import sys
from pathlib import Path

import mujoco
import numpy as np
import torch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
ap = argparse.ArgumentParser()
ap.add_argument("--model", default="hero", choices=["hero", "standard"])
ap.add_argument("--wait", type=float, default=2.0, help="seconds of balancing before the go signal")
ap.add_argument("--video", type=Path)
a = ap.parse_args()
os.environ["K1_FLIP_MODEL"] = a.model
os.environ["K1_FLIP_GO_AFTER"] = str(a.wait)

from booster_deploy.controllers.mujoco_controller import MujocoController  # noqa: E402
from booster_deploy.robots.k1 import K1_CFG  # noqa: E402
from booster_deploy.utils.registry import get_task  # noqa: E402


class MappedMujocoController(MujocoController):
    """MuJoCo controller with the explicit K1 robot-order <-> sim-order joint mapping."""

    def __init__(self, cfg):
        super().__init__(cfg)
        m = self.mj_model
        jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in self.robot.cfg.sim_joint_names]
        if min(jid) < 0:
            raise RuntimeError("a K1 joint is missing from the MuJoCo model")
        self.qpos_ids, self.qvel_ids = m.jnt_qposadr[jid], m.jnt_dofadr[jid]
        self.act_ids = np.array([int(np.flatnonzero(m.actuator_trnid[:, 0] == j)[0]) for j in jid])
        self.r2s, self.s2r = self.robot.data.real2sim_joint_indexes, self.robot.data.sim2real_joint_indexes
        dt = cfg.policy_dt / cfg.mujoco.decimation
        ms = [22.0 if "hip" in n else 6.0 if ("knee" in n or "ankle" in n) else 15.0 for n in self.robot.cfg.sim_joint_names]
        self.delay = np.array([int(round(x / 1000 / dt)) for x in ms])  # actuator command latency, physics steps
        self.hist = None

    def update_state(self) -> None:
        d, dev = self.mj_data, self.robot.data.device
        f = lambda x: torch.from_numpy(x.astype(np.float32)).to(dev)  # noqa: E731
        self.robot.data.joint_pos = f(d.qpos[self.qpos_ids][self.s2r])
        self.robot.data.joint_vel = f(d.qvel[self.qvel_ids][self.s2r])
        self.robot.data.feedback_torque = f(d.qfrc_actuator[self.qvel_ids][self.s2r])
        self.robot.data.root_pos_w, self.robot.data.root_quat_w = f(d.qpos[:3]), f(d.qpos[3:7])
        self.robot.data.root_lin_vel_b, self.robot.data.root_ang_vel_b = f(d.qvel[:3]), f(d.qvel[3:6])

    def ctrl_step(self, targets: torch.Tensor) -> None:
        t = targets.detach().cpu().numpy()[self.r2s]
        kp, kd, eff = (x.numpy()[self.r2s] for x in (self.robot.joint_stiffness, self.robot.joint_damping, self.robot.effort_limit))
        if self.hist is None:
            self.hist = np.repeat(t[None], self.delay.max() + 1, 0)
        cols = np.arange(len(t))
        for _ in range(self.decimation):
            self.hist = np.roll(self.hist, 1, 0); self.hist[0] = t
            cur = self.hist[self.delay, cols]
            q, v = self.mj_data.qpos[self.qpos_ids], self.mj_data.qvel[self.qvel_ids]
            self.mj_data.ctrl[self.act_ids] = np.clip(kp * (cur - q) - kd * v, -eff, eff)
            mujoco.mj_step(self.mj_model, self.mj_data)


importlib.import_module("tasks.k1_backflip")
cfg = get_task("k1_backflip")
cfg.policy.device = "cpu"
c = MappedMujocoController(cfg)
m, d = c.mj_model, c.mj_data
renderer = mujoco.Renderer(m, 480, 640) if a.video else None
frames = []


def step(targets=None):
    c.update_state()
    c.ctrl_step(c.policy_step() if targets is None else targets)
    if renderer is not None:
        cam = mujoco.MjvCamera(); cam.lookat[:] = d.qpos[:3]; cam.distance, cam.azimuth, cam.elevation = 2.6, 90, -10
        renderer.update_scene(d, cam); frames.append(renderer.render())


# X: stand up in the X pose (stiff gains: MuJoCo has no joint friction), then A: the policy takes over
x_pose = torch.tensor(cfg.robot.prepare_state.joint_pos, dtype=torch.float32)
d.qpos[c.qpos_ids] = x_pose.numpy()[c.r2s]; d.qpos[2] = 0.58; mujoco.mj_forward(m, d)
c.robot.joint_stiffness = torch.tensor(K1_CFG.prepare_state.stiffness, dtype=torch.float32)
c.robot.joint_damping = torch.tensor(K1_CFG.prepare_state.damping, dtype=torch.float32)
for _ in range(int(2.0 / cfg.policy_dt)):
    step(x_pose)
c.robot.joint_stiffness = torch.tensor(cfg.robot.joint_stiffness, dtype=torch.float32)
c.robot.joint_damping = torch.tensor(cfg.robot.joint_damping, dtype=torch.float32)
c.update_state(); c.start()
turn = 0.0
for _ in range(int((a.wait + 7.0) / cfg.policy_dt)):
    step(); turn += float(d.qvel[4]) * cfg.policy_dt
R = np.empty(9); mujoco.mju_quat2Mat(R, d.qpos[3:7]); tilt = np.degrees(np.arccos(np.clip(R.reshape(3, 3)[2, 2], -1, 1)))
ok = abs(np.degrees(turn)) > 270 and tilt < 25 and d.qpos[2] > 0.4  # pitch-rate integral reads low when the flip rolls a little
print(f"{a.model}: rotation {abs(np.degrees(turn)):.0f} deg, trunk tilt at the end {tilt:.0f} deg, height {d.qpos[2]:.2f} m -> "
      f"{'flipped and standing' if ok else 'did not land standing'}")
if frames:
    p = subprocess.Popen(["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", "640x480", "-r", "50",
                          "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", str(a.video)], stdin=subprocess.PIPE)
    for fr in frames:
        p.stdin.write(fr.tobytes())
    p.stdin.close(); p.wait()
