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

import os
import sys
from pathlib import Path

import hydra
from hydra.utils import instantiate
from omegaconf import OmegaConf
import torch, copy

class _OnnxPolicyExporter(torch.nn.Module):
    """Exporter of actor-critic into ONNX file."""

    def __init__(self, actor, normalizer=None, verbose=False):
        super().__init__()
        self.verbose = verbose
        self.actor = copy.deepcopy(actor._original_module)
        # copy normalizer if exists
        if normalizer:
            self.normalizer = copy.deepcopy(normalizer)
        else:
            self.normalizer = torch.nn.Identity()

    def forward(self, x):
        return self.actor.eval_forward(x)

    def export(self, path, filename):
        self.to("cpu")
        obs = {'obs': torch.zeros(1, 448),  'mimic_phase': torch.zeros(1,2)}
        torch.onnx.export(
            self,
            {'x': obs},
            os.path.join(path, filename),
            export_params=True,
            opset_version=11,
            verbose=self.verbose,
            input_names=["obs"],
            output_names=["actions"],
            dynamic_axes={},
        )


has_robot_arg = False
backbone = None
for arg in sys.argv:
    # This hack ensures that isaacgym is imported before any torch modules.
    # The reason it is here (and not in the main func) is due to pytorch lightning multi-gpu behavior.
    if "robot" in arg:
        has_robot_arg = True
    if "backbone" in arg:
        if not has_robot_arg:
            raise ValueError("+robot argument should be provided before +backbone")
        if "isaacgym" in arg.split("=")[-1]:
            import isaacgym  # noqa: F401

            backbone = "isaacgym"
        elif "isaaclab" in arg.split("=")[-1]:
            from isaaclab.app import AppLauncher

            backbone = "isaaclab"

from lightning.fabric import Fabric  # noqa: E402
from utils.config_utils import *  # noqa: E402, F403

from phys_anim.agents.ppo import PPO  # noqa: E402


@hydra.main(config_path="config")
def main(override_config: OmegaConf):
    os.chdir(hydra.utils.get_original_cwd())

    if override_config.checkpoint is not None:
        has_config = True

        checkpoint = Path(override_config.checkpoint)
        config_path = checkpoint.parent / "config.yaml"
        if not config_path.exists():
            config_path = checkpoint.parent.parent / "config.yaml"
            if not config_path.exists():
                has_config = False
                print(f"Could not find config path: {config_path}")

        if has_config:
            print(f"Loading training config file from {config_path}")
            with open(config_path) as file:
                train_config = OmegaConf.load(file)

            if train_config.eval_overrides is not None:
                train_config = OmegaConf.merge(
                    train_config, train_config.eval_overrides
                )

            config = OmegaConf.merge(train_config, override_config)
        else:
            config = override_config
    else:
        if override_config.eval_overrides is not None:
            config = override_config.copy()
            eval_overrides = OmegaConf.to_container(config.eval_overrides, resolve=True)
            for arg in sys.argv[1:]:
                if not arg.startswith("+"):
                    key = arg.split("=")[0]
                    if key in eval_overrides:
                        del eval_overrides[key]
            config.eval_overrides = OmegaConf.create(eval_overrides)
            config = OmegaConf.merge(config, eval_overrides)
        else:
            config = override_config

    fabric: Fabric = instantiate(config.fabric)
    fabric.launch()

    if backbone == "isaaclab":
        app_launcher = AppLauncher({"headless": config.headless})
        simulation_app = app_launcher.app
        env = instantiate(
            config.env, device=fabric.device, simulation_app=simulation_app
        )
    else:
        env = instantiate(config.env, device=fabric.device)

    algo: PPO = instantiate(config.algo, env=env, fabric=fabric)
    algo.setup()
    algo.load(config.checkpoint)

    # _OnnxPolicyExporter(algo.actor).export(
    #     './policy_output', "policy.onnx"
    # )

    # algo.evaluate_policy()
    # algo.collect_dataset()
    algo.evaluate_sequential_policy()


if __name__ == "__main__":
    main()
