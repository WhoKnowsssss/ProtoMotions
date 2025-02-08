# Copyright (c) 2018-2022, NVIDIA Corporation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

from typing import TYPE_CHECKING
import torch

import isaaclab.sim as sim_utils
from isaaclab.scene import InteractiveScene
from isaaclab.sim import SimulationContext, PhysxCfg

from phys_anim.envs.base_interface.isaaclab_utils.robots import (
    SMPLSceneCfg,
    SMPLXSceneCfg,
    G1SceneCfg
)
from phys_anim.envs.base_interface.isaaclab_utils.domain_rand import (
    randomize_rigid_body_material,
    randomize_rigid_body_mass,
    randomize_joint_default_pos,
    push_by_setting_velocity,
)
from phys_anim.envs.base_interface.common import BaseInterface

if TYPE_CHECKING:
    # Just used for autocomplete.
    from phys_anim.envs.humanoid.isaaclab import Humanoid
else:
    Humanoid = object


class SimBaseInterface(BaseInterface, Humanoid):
    def __init__(self, config, device: torch.device, simulation_app):
        """
        This class provides a unified interface with IsaacGym environments.
        """

        # IsaacSim does not support substeps.
        # Instead we run at a higher FPS but perform more decimation steps (control_freq_inv).
        # This enables sharing the config between IsaacSim and IsaacGym.
        config.simulator.sim.fps *= config.simulator.sim.substeps
        config.simulator.sim.control_freq_inv *= config.simulator.sim.substeps
        config.simulator.sim.substeps = 1

        super().__init__(config, device, simulation_app)

        sim_cfg = sim_utils.SimulationCfg(
            device=str(device),
            dt=1.0 / config.simulator.sim.fps,
            render_interval=1,
            physx=PhysxCfg(
                solver_type=config.simulator.sim.physx.solver_type,
                max_position_iteration_count=config.simulator.sim.physx.num_position_iterations,
                max_velocity_iteration_count=config.simulator.sim.physx.num_velocity_iterations,
                bounce_threshold_velocity=config.simulator.sim.physx.bounce_threshold_velocity,
                gpu_max_rigid_contact_count=config.simulator.sim.physx.gpu_max_rigid_contact_count,
                gpu_found_lost_pairs_capacity=config.simulator.sim.physx.gpu_found_lost_pairs_capacity,
                gpu_found_lost_aggregate_pairs_capacity=config.simulator.sim.physx.gpu_found_lost_aggregate_pairs_capacity,
            ),
        )
        self.simulation_app = simulation_app
        self.sim = SimulationContext(sim_cfg)
        # Set main camera
        self.sim.set_camera_view([2.5, 0.0, 4.0], [0.0, 0.0, 2.0])

        num_objects = 0
        if self.scene_lib is not None:
            num_objects = self.scene_lib.total_num_objects
        if self.config.robot.asset.robot_type == "smpl_humanoid":
            scene_cfg = SMPLSceneCfg(
                num_envs=config.num_envs,
                env_spacing=2.0,
                num_objects=num_objects,
            )
        elif self.config.robot.asset.robot_type == "smplx_humanoid":
            scene_cfg = SMPLXSceneCfg(
                num_envs=config.num_envs,
                env_spacing=2.0,
                num_objects=num_objects,
            )
        elif self.config.robot.asset.robot_type == "g1_humanoid":
            scene_cfg = G1SceneCfg(
            num_envs=config.num_envs,
            env_spacing=2.0,
            num_objects=num_objects,
            )
        else:
            raise ValueError(
                f"Unsupported robot type: {self.config.robot.asset.robot_type}"
            )

        self.scene = InteractiveScene(scene_cfg)
        self.set_up_scene()
        # Now we are ready!
        print("[INFO]: Setup complete...")

        self.robot = self.scene["robot"]
        self.contact_sensor = self.scene["contact_sensor"]

        self.sim.reset()

        # ---------------- domain randomization ----------------------
        randomize_rigid_body_material(  
            robot=self.robot, 
            static_friction_range=(0.1, 0.6),
            dynamic_friction_range=(0.1, 0.6),
            restitution_range=(0.0, 0.1),
            num_buckets=64,
            num_envs=self.num_envs, 
            body_names=[self.config.robot.right_foot_name, self.config.robot.left_foot_name], 
        )
        randomize_rigid_body_mass(
            robot=self.robot,
            mass_distribution_params=[-5,5],
            operation='add',
            num_envs=self.num_envs,
            body_names=['torso', 'pelvis']
        )
        randomize_joint_default_pos(
            robot=self.robot,
            num_envs=self.num_envs,
            pos_distribution_params=(-0.05, 0.05),
        )
        self._interval_term_time_left = torch.zeros(1, device=self.device)

        # Get material properties
        if self.objects_view is not None:
            self.objects_view.initialize()

    def set_up_scene(self) -> None:
        if not self.headless:
            self.setup_keyboard()

    def setup_keyboard(self):
        from isaaclab.devices.keyboard.se2_keyboard import Se2Keyboard

        self.keyboard_interface = Se2Keyboard()
        self.keyboard_interface.add_callback("R", self.force_reset)
        self.keyboard_interface.add_callback("U", self.update_inference_parameters)
        self.keyboard_interface.add_callback("L", self.toggle_video_record)
        self.keyboard_interface.add_callback(";", self.cancel_video_record)
        self.keyboard_interface.add_callback("Q", self.close_simulation)

    def on_environment_ready(self):
        self.init_done = True

    @property
    def default_base_env_path(self):
        """Retrieves default path to the parent of all env prims.

        Returns:
            default_base_env_path(str): Defaults to "/World/envs".
        """
        return "/World/envs"

    @property
    def default_zero_env_path(self):
        """Retrieves default path to the first env prim (index 0).

        Returns:
            default_zero_env_path(str): Defaults to "/World/envs/env_0".
        """
        return f"{self.default_base_env_path}/env_0"

    @property
    def default_object_path(self):
        return "/World/objects"

    def step(self, actions):
        self.pre_physics_step(actions)

        self.physics_step()

        self.post_physics_step()

        self.render()

        return (
            self.rew_buf,
            self.reset_buf,
            self.extras,
        )
    
    def post_physics_step(self):

        time_left = self._interval_term_time_left[0]
        # update the time left for each environment
        time_left -= self.dt
        # check if the interval has passed and sample a new interval
        # note: we compare with a small value to handle floating point errors
        if time_left < 1e-6:
            lower, upper = [4, 8]
            sampled_interval = torch.rand(1) * (upper - lower) + lower
            self._interval_term_time_left[0] = sampled_interval
            push_by_setting_velocity(
                robot=self.robot,
                velocity_range={"x": (-0.5, 0.5), "y": (-0.5, 0.5)},
                num_envs=self.num_envs,
            )

    def write_viewport_to_file(self, file_name):
        from omni.kit.viewport.utility import (
            get_active_viewport,
            capture_viewport_to_file,
        )

        vp_api = get_active_viewport()
        capture_viewport_to_file(vp_api, file_name)

    def simulate(self):
        self.scene.write_data_to_sim()
        self.sim.step()
        # Update buffers
        self.scene.update(self.sim.get_physics_dt())

    def close_simulation(self):
        self.simulation_app.close()
