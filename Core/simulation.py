# core/simulation.py
"""
Minimal MuJoCo simulation wrapper for the Intro to Robotics course.

Usage (from an assignment script):
    from core.simulation import Simulation

    with Simulation("core/models/ur5e_scene.xml", show_viewer=True) as sim:
        sim.run(duration=5.0)  # run 5 seconds of simulated time

Or with a custom per-step callback:
    def control_cb(sim):
        # Example: zero control each step
        sim.data.ctrl[:] = 0.0

    with Simulation("core/models/ur5e_scene.xml", show_viewer=True) as sim:
        sim.run(duration=5.0, step_cb=control_cb)

Students should *not* modify this file.
"""

from __future__ import annotations

import os
import time
from pathlib import Path
from typing import Callable, Optional

import numpy as np
import mujoco
import mujoco.viewer


class Simulation:
    """
    A small convenience wrapper around MuJoCo for consistent usage across assignments.

    Parameters
    ----------
    model_path : str | os.PathLike
        Path to the MJCF/XML model. If relative, we try it as-given,
        then relative to the directory of this file (e.g., core/models/...).
    show_viewer : bool
        If True, launches the interactive viewer. If False, runs headless.
    realtime : bool
        If True, attempts to keep wall-clock time close to simulation time in run().
    seed : Optional[int]
        Random seed for deterministic resets (sets np.random).
    """

    def __init__(
        self,
        model_path: str | os.PathLike,
        *,
        show_viewer: bool = True,
        realtime: bool = True,
        seed: Optional[int] = None,
    ) -> None:
        self.model_path = self._resolve_model_path(model_path)
        if not self.model_path.exists():
            raise FileNotFoundError(f"Model not found: {self.model_path}")

        # Optional deterministic behavior for any randomization performed by assignments.
        if seed is not None:
            np.random.seed(seed)

        # Load model & data
        self.model = mujoco.MjModel.from_xml_path(str(self.model_path))
        self.data = mujoco.MjData(self.model)

        # Viewer / UI
        self.show_viewer = show_viewer
        self._viewer_ctx = None  # type: Optional[mujoco.viewer.RendererContext]
        self._viewer = None      # type: Optional[mujoco.viewer.Viewer]

        # Timing
        self.realtime = realtime
        self._dt = self.model.opt.timestep

        # For clean context-manager handling
        self._opened = False

    # ---------- Public API ----------

    def reset(self) -> None:
        """Reset simulation state to model's keyframe 0 if present, else zeros."""
        mujoco.mj_resetData(self.model, self.data)
        # If there's a named keyframe "home" or index 0, use it as a nice default
        if self.model.nkey > 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, 0)

    def step(self, n: int = 1) -> None:
        """Advance the simulation by n internal steps."""
        for _ in range(n):
            mujoco.mj_step(self.model, self.data)

    def run(
        self,
        *,
        duration: Optional[float] = None,
        step_cb: Optional[Callable[["Simulation"], None]] = None,
    ) -> None:
        """
        Main loop helper.
        - If show_viewer is True, keeps running while the viewer is open.
        - If duration is given, runs for `duration` seconds of *simulated* time.
        - `step_cb` (if provided) is called every step before stepping the physics.

        This method returns when:
          * duration is reached (if specified), or
          * the viewer is closed (in viewer mode), or
          * the process receives KeyboardInterrupt.
        """
        target_steps = None
        if duration is not None:
            target_steps = int(np.ceil(duration / self._dt))

        steps_done = 0
        t_wall_last = time.perf_counter()

        try:
            if self.show_viewer:
                # Interactive viewer loop (passive control)
                with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
                    self._viewer = viewer
                    while viewer.is_running():
                        if step_cb is not None:
                            step_cb(self)

                        mujoco.mj_step(self.model, self.data)
                        steps_done += 1

                        # Attempt real-time pacing
                        if self.realtime:
                            t_now = time.perf_counter()
                            sim_advance = self._dt
                            wall_elapsed = t_now - t_wall_last
                            # Sleep a little if we're ahead of real time
                            if wall_elapsed < sim_advance:
                                time.sleep(sim_advance - wall_elapsed)
                            t_wall_last = time.perf_counter()

                        # Respect duration if provided
                        if target_steps is not None and steps_done >= target_steps:
                            break
            else:
                # Headless loop
                while True:
                    if step_cb is not None:
                        step_cb(self)

                    mujoco.mj_step(self.model, self.data)
                    steps_done += 1

                    if target_steps is not None and steps_done >= target_steps:
                        break

                    if self.realtime:
                        # Pace similar to viewer case
                        t_now = time.perf_counter()
                        sim_advance = self._dt
                        wall_elapsed = t_now - t_wall_last
                        if wall_elapsed < sim_advance:
                            time.sleep(sim_advance - wall_elapsed)
                        t_wall_last = time.perf_counter()

        except KeyboardInterrupt:
            # Graceful stop from user
            pass

    def set_qpos(self, qpos: np.ndarray) -> None:
        """Set joint positions (copies into data.qpos; length must match)."""
        if qpos.shape[0] != self.model.nq:
            raise ValueError(f"qpos length {qpos.shape[0]} != model.nq {self.model.nq}")
        self.data.qpos[:] = qpos
        mujoco.mj_forward(self.model, self.data)

    def set_qvel(self, qvel: np.ndarray) -> None:
        """Set joint velocities (copies into data.qvel; length must match)."""
        if qvel.shape[0] != self.model.nv:
            raise ValueError(f"qvel length {qvel.shape[0]} != model.nv {self.model.nv}")
        self.data.qvel[:] = qvel
        mujoco.mj_forward(self.model, self.data)

    def set_ctrl(self, ctrl: np.ndarray) -> None:
        """Set actuator control vector (copies into data.ctrl; length must match)."""
        if ctrl.shape[0] != self.model.nu:
            raise ValueError(f"ctrl length {ctrl.shape[0]} != model.nu {self.model.nu}")
        self.data.ctrl[:] = ctrl

    def get_qpos(self) -> np.ndarray:
        return np.copy(self.data.qpos)

    def get_qvel(self) -> np.ndarray:
        return np.copy(self.data.qvel)

    def get_ctrl(self) -> np.ndarray:
        return np.copy(self.data.ctrl)

    def close(self) -> None:
        """Cleanup (mainly for symmetry; viewer is context-managed in run())."""
        self._viewer = None

    # ---------- Context manager support ----------

    def __enter__(self) -> "Simulation":
        self.reset()
        self._opened = True
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.close()
        self._opened = False

    # ---------- Helpers ----------

    @staticmethod
    def _resolve_model_path(model_path: str | os.PathLike) -> Path:
        """
        Resolve the model path robustly:
          1) as given (absolute or relative to CWD)
          2) relative to this file's directory (e.g., core/models/...)
        """
        p = Path(model_path)
        if p.exists():
            return p.resolve()

        here = Path(__file__).resolve().parent
        candidate = (here / Path(model_path)).resolve()
        if candidate.exists():
            return candidate

        # Also try relative to a sibling 'models' directory if just a filename was provided
        models_dir = here / "models"
        candidate2 = (models_dir / p).resolve()
        if candidate2.exists():
            return candidate2

        # Nothing worked; return the original so caller sees the intended path in the error
        return p
