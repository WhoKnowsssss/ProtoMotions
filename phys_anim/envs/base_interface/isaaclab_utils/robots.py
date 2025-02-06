from isaaclab.actuators import ImplicitActuatorCfg
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg
from isaaclab.utils import configclass
from isaaclab.scene import InteractiveSceneCfg
from isaaclab.sensors import ContactSensorCfg


SMPLX_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"phys_anim/data/assets/usd/smplx_humanoid.usda",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=10.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=8,
            solver_velocity_iteration_count=1,
        ),
        visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(0.65, 0.1, 1.0), metallic=0.5
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(0.0, 0.0, 0.95),
        joint_pos={".*": 0.0},
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                "L_Hip_.",
                "R_Hip_.",
                "L_Knee_.",
                "R_Knee_.",
                "L_Ankle_.",
                "R_Ankle_.",
                "L_Toe_.",
                "R_Toe_.",
            ],
            effort_limit=500,
            velocity_limit=100.0,
            stiffness={
                "L_Hip_.": 800,
                "R_Hip_.": 800,
                "L_Knee_.": 800,
                "R_Knee_.": 800,
                "L_Ankle_.": 800,
                "R_Ankle_.": 800,
                "L_Toe_.": 500,
                "R_Toe_.": 500,
            },
            damping={
                "L_Hip_.": 80,
                "R_Hip_.": 80,
                "L_Knee_.": 80,
                "R_Knee_.": 80,
                "L_Ankle_.": 80,
                "R_Ankle_.": 80,
                "L_Toe_.": 50,
                "R_Toe_.": 50,
            },
        ),
        "torso": ImplicitActuatorCfg(
            joint_names_expr=[
                "Torso_.",
                "Spine_.",
                "Chest_.",
                "Neck_.",
                "Head_.",
                "L_Thorax_.",
                "R_Thorax_.",
            ],
            effort_limit=500,
            velocity_limit=100.0,
            stiffness={
                "Torso_.": 1000,
                "Spine_.": 1000,
                "Chest_.": 1000,
                "Neck_.": 500,
                "Head_.": 500,
                "L_Thorax_.": 500,
                "R_Thorax_.": 500,
            },
            damping={
                "Torso_.": 100,
                "Spine_.": 100,
                "Chest_.": 100,
                "Neck_.": 50,
                "Head_.": 50,
                "L_Thorax_.": 50,
                "R_Thorax_.": 50,
            },
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                "L_Shoulder_.",
                "R_Shoulder_.",
                "L_Elbow_.",
                "R_Elbow_.",
                "L_Wrist_.",
                "R_Wrist_.",
            ],
            effort_limit=300,
            velocity_limit=100.0,
            stiffness={
                "L_Shoulder_.": 500,
                "R_Shoulder_.": 500,
                "L_Elbow_.": 300,
                "R_Elbow_.": 300,
                "L_Wrist_.": 300,
                "R_Wrist_.": 300,
            },
            damping={
                "L_Shoulder_.": 50,
                "R_Shoulder_.": 50,
                "L_Elbow_.": 30,
                "R_Elbow_.": 30,
                "L_Wrist_.": 30,
                "R_Wrist_.": 30,
            },
        ),
        "hands": ImplicitActuatorCfg(
            joint_names_expr=[
                "L_Index1_.",
                "L_Index2_.",
                "L_Index3_.",
                "L_Middle1_.",
                "L_Middle2_.",
                "L_Middle3_.",
                "L_Pinky1_.",
                "L_Pinky2_.",
                "L_Pinky3_.",
                "L_Ring1_.",
                "L_Ring2_.",
                "L_Ring3_.",
                "L_Thumb1_.",
                "L_Thumb2_.",
                "L_Thumb3_.",
                "R_Index1_.",
                "R_Index2_.",
                "R_Index3_.",
                "R_Middle1_.",
                "R_Middle2_.",
                "R_Middle3_.",
                "R_Pinky1_.",
                "R_Pinky2_.",
                "R_Pinky3_.",
                "R_Ring1_.",
                "R_Ring2_.",
                "R_Ring3_.",
                "R_Thumb1_.",
                "R_Thumb2_.",
                "R_Thumb3_.",
            ],
            effort_limit=10,
            velocity_limit=10.0,
            stiffness={
                "L_Index1_.": 20,
                "L_Index2_.": 20,
                "L_Index3_.": 20,
                "L_Middle1_.": 20,
                "L_Middle2_.": 20,
                "L_Middle3_.": 20,
                "L_Pinky1_.": 20,
                "L_Pinky2_.": 20,
                "L_Pinky3_.": 20,
                "L_Ring1_.": 20,
                "L_Ring2_.": 20,
                "L_Ring3_.": 20,
                "L_Thumb1_.": 20,
                "L_Thumb2_.": 20,
                "L_Thumb3_.": 20,
                "R_Index1_.": 20,
                "R_Index2_.": 20,
                "R_Index3_.": 20,
                "R_Middle1_.": 20,
                "R_Middle2_.": 20,
                "R_Middle3_.": 20,
                "R_Pinky1_.": 20,
                "R_Pinky2_.": 20,
                "R_Pinky3_.": 20,
                "R_Ring1_.": 20,
                "R_Ring2_.": 20,
                "R_Ring3_.": 20,
                "R_Thumb1_.": 20,
                "R_Thumb2_.": 20,
                "R_Thumb3_.": 20,
            },
            damping={
                "L_Index1_.": 2,
                "L_Index2_.": 2,
                "L_Index3_.": 2,
                "L_Middle1_.": 2,
                "L_Middle2_.": 2,
                "L_Middle3_.": 2,
                "L_Pinky1_.": 2,
                "L_Pinky2_.": 2,
                "L_Pinky3_.": 2,
                "L_Ring1_.": 2,
                "L_Ring2_.": 2,
                "L_Ring3_.": 2,
                "L_Thumb1_.": 2,
                "L_Thumb2_.": 2,
                "L_Thumb3_.": 2,
                "R_Index1_.": 2,
                "R_Index2_.": 2,
                "R_Index3_.": 2,
                "R_Middle1_.": 2,
                "R_Middle2_.": 2,
                "R_Middle3_.": 2,
                "R_Pinky1_.": 2,
                "R_Pinky2_.": 2,
                "R_Pinky3_.": 2,
                "R_Ring1_.": 2,
                "R_Ring2_.": 2,
                "R_Ring3_.": 2,
                "R_Thumb1_.": 2,
                "R_Thumb2_.": 2,
                "R_Thumb3_.": 2,
            },
        ),
    },
)


SMPL_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"phys_anim/data/assets/usd/smpl_humanoid.usda",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=10.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
        visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(0.65, 0.1, 1.0), metallic=0.5
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(
            0.0,
            0.0,
            0.95,
        ),  # Default initial state of SMPL with the upright configuration
        joint_pos={".*": 0.0},
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                "L_Hip_.",
                "R_Hip_.",
                "L_Knee_.",
                "R_Knee_.",
                "L_Ankle_.",
                "R_Ankle_.",
                "L_Toe_.",
                "R_Toe_.",
            ],
            effort_limit=500,
            velocity_limit=100.0,
            stiffness={
                "L_Hip_.": 800,
                "R_Hip_.": 800,
                "L_Knee_.": 800,
                "R_Knee_.": 800,
                "L_Ankle_.": 800,
                "R_Ankle_.": 800,
                "L_Toe_.": 500,
                "R_Toe_.": 500,
            },
            damping={
                "L_Hip_.": 80,
                "R_Hip_.": 80,
                "L_Knee_.": 80,
                "R_Knee_.": 80,
                "L_Ankle_.": 80,
                "R_Ankle_.": 80,
                "L_Toe_.": 50,
                "R_Toe_.": 50,
            },
        ),
        "torso": ImplicitActuatorCfg(
            joint_names_expr=[
                "Torso_.",
                "Spine_.",
                "Chest_.",
                "Neck_.",
                "Head_.",
                "L_Thorax_.",
                "R_Thorax_.",
            ],
            effort_limit=500,
            velocity_limit=100.0,
            stiffness={
                "Torso_.": 1000,
                "Spine_.": 1000,
                "Chest_.": 1000,
                "Neck_.": 500,
                "Head_.": 500,
                "L_Thorax_.": 500,
                "R_Thorax_.": 500,
            },
            damping={
                "Torso_.": 100,
                "Spine_.": 100,
                "Chest_.": 100,
                "Neck_.": 50,
                "Head_.": 50,
                "L_Thorax_.": 50,
                "R_Thorax_.": 50,
            },
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                "L_Shoulder_.",
                "R_Shoulder_.",
                "L_Elbow_.",
                "R_Elbow_.",
                "L_Wrist_.",
                "R_Wrist_.",
                "L_Hand_.",
                "R_Hand_.",
            ],
            effort_limit=300,
            velocity_limit=100.0,
            stiffness={
                "L_Shoulder_.": 500,
                "R_Shoulder_.": 500,
                "L_Elbow_.": 300,
                "R_Elbow_.": 300,
                "L_Wrist_.": 300,
                "R_Wrist_.": 300,
                "L_Hand_.": 300,
                "R_Hand_.": 300,
            },
            damping={
                "L_Shoulder_.": 50,
                "R_Shoulder_.": 50,
                "L_Elbow_.": 30,
                "R_Elbow_.": 30,
                "L_Wrist_.": 30,
                "R_Wrist_.": 30,
            },
        ),
    },
)

G1_CFG = ArticulationCfg(
    spawn=sim_utils.UsdFileCfg(
        usd_path=f"phys_anim/data/assets/usd/g1.usd",
        activate_contact_sensors=True,
        rigid_props=sim_utils.RigidBodyPropertiesCfg(
            disable_gravity=False,
            retain_accelerations=False,
            linear_damping=0.0,
            angular_damping=0.0,
            max_linear_velocity=1000.0,
            max_angular_velocity=1000.0,
            max_depenetration_velocity=10.0,
        ),
        articulation_props=sim_utils.ArticulationRootPropertiesCfg(
            enabled_self_collisions=True,
            solver_position_iteration_count=4,
            solver_velocity_iteration_count=0,
        ),
        visual_material=sim_utils.PreviewSurfaceCfg(
            diffuse_color=(0.65, 0.1, 1.0), metallic=0.5
        ),
    ),
    init_state=ArticulationCfg.InitialStateCfg(
        pos=(
            0.0,
            0.0,
            0.8,
        ),  # Default initial state of SMPL with the upright configuration
        joint_pos={".*": 0.0},
        joint_vel={".*": 0.0},
    ),
    soft_joint_pos_limit_factor=0.9,
    actuators={
        "legs": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_hip_yaw_joint",
                ".*_hip_roll_joint",
                ".*_hip_pitch_joint",
                ".*_knee_joint",
            ],
            effort_limit={
                ".*_hip_yaw_joint": 88.0,
                ".*_hip_roll_joint": 139.0,
                ".*_hip_pitch_joint": 88.0,
                ".*_knee_joint": 139.0,
            },
            velocity_limit={
                ".*_hip_yaw_joint": 32.0,
                ".*_hip_roll_joint": 20.0,
                ".*_hip_pitch_joint": 32.0,
                ".*_knee_joint": 20.0,
            },
            stiffness={
                ".*_hip_yaw_joint": 75.0,
                ".*_hip_roll_joint": 75.0,
                ".*_hip_pitch_joint": 75.0,
                ".*_knee_joint": 75.0,
            },
            damping={
                ".*_hip_yaw_joint": 2.0,
                ".*_hip_roll_joint": 2.0,
                ".*_hip_pitch_joint": 2.0,
                ".*_knee_joint": 2.0,
            },
            armature={
                ".*_hip_pitch_joint": 0.01017752004,
                ".*_hip_roll_joint": 0.025101925,
                ".*_hip_yaw_joint": 0.01017752004,
                ".*_knee_joint": 0.025101925,
            },
        ),
        "feet": ImplicitActuatorCfg(
            effort_limit={
                ".*_ankle_pitch_joint": 50.0,
                ".*_ankle_roll_joint": 50.0,
            },
            velocity_limit={
                ".*_ankle_pitch_joint": 37.0,
                ".*_ankle_roll_joint": 37.0,
            },
            joint_names_expr=[".*_ankle_pitch_joint", ".*_ankle_roll_joint"],
            stiffness={".*_ankle_pitch_joint": 20.0, ".*_ankle_roll_joint": 2.0},
            damping={".*_ankle_pitch_joint": 1.0, ".*_ankle_roll_joint": 0.2},
            armature={
                ".*_ankle_pitch_joint": 0.00721945,
                ".*_ankle_roll_joint": 0.00721945
            }
        ),
        "waist": ImplicitActuatorCfg(
            effort_limit=88.0,
            velocity_limit=32.0,
            joint_names_expr=["waist_yaw_joint", "waist_pitch_joint", "waist_roll_joint"],
            stiffness=75.0,
            damping=2.0,
            armature=0.01017752004,
        ),
        "arms": ImplicitActuatorCfg(
            joint_names_expr=[
                ".*_shoulder_pitch_joint",
                ".*_shoulder_roll_joint",
                ".*_shoulder_yaw_joint",
                ".*_elbow_joint",
                ".*_wrist_roll_joint",
                ".*_wrist_pitch_joint",
                ".*_wrist_yaw_joint",
            ],
            effort_limit={
                ".*_shoulder_pitch_joint": 25.0,
                ".*_shoulder_roll_joint": 25.0,
                ".*_shoulder_yaw_joint": 25.0,
                ".*_elbow_joint": 25.0,
                ".*_wrist_roll_joint": 25.0,
                ".*_wrist_pitch_joint": 5.0,
                ".*_wrist_yaw_joint": 5.0,
            },
            velocity_limit={
                ".*_shoulder_pitch_joint": 37.0,
                ".*_shoulder_roll_joint": 37.0,
                ".*_shoulder_yaw_joint": 37.0,
                ".*_elbow_joint": 37.0,
                ".*_wrist_roll_joint": 37.0,
                ".*_wrist_pitch_joint": 22.0,
                ".*_wrist_yaw_joint": 22.0,
            },
            stiffness={
                ".*_shoulder_pitch_joint": 15.0,
                ".*_shoulder_roll_joint": 15.0,
                ".*_shoulder_yaw_joint": 15.0,
                ".*_elbow_joint": 15.0,
                ".*_wrist_roll_joint": 2.0,
                ".*_wrist_pitch_joint": 2.0,
                ".*_wrist_yaw_joint": 2.0,
            },
            damping={
                ".*_shoulder_pitch_joint": 1.0,
                ".*_shoulder_roll_joint": 1.0,
                ".*_shoulder_yaw_joint": 1.0,
                ".*_elbow_joint": 1.0,
                ".*_wrist_roll_joint": 0.2,
                ".*_wrist_pitch_joint": 0.2,
                ".*_wrist_yaw_joint": 0.2,
            },
            armature={
                ".*_shoulder_pitch_joint": 0.003609725,
                ".*_shoulder_roll_joint": 0.003609725,
                ".*_shoulder_yaw_joint": 0.003609725,
                ".*_elbow_joint": 0.003609725,
                ".*_wrist_roll_joint": 0.003609725,
                ".*_wrist_pitch_joint": 0.00425,
                ".*_wrist_yaw_joint": 0.00425,
            }
        ),
        
    },
)


@configclass
class SMPLSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    def __init__(self, num_objects: int, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # lights
        self.dome_light = AssetBaseCfg(
            prim_path="/World/Light",
            spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
        )

        # articulation
        self.robot: ArticulationCfg = SMPL_CFG.replace(
            prim_path="/World/envs/env_.*/Robot"
        )
        self.contact_sensor: ContactSensorCfg = ContactSensorCfg(
            prim_path="/World/envs/env_.*/Robot/bodies/.*",
            filter_prim_paths_expr=[
                f"/World/objects/object_{i}" for i in range(num_objects)
            ]
            + ["/World/terrain"],
        )

@configclass
class G1SceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    def __init__(self, num_objects: int, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # lights
        self.dome_light = AssetBaseCfg(
            prim_path="/World/Light",
            spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
        )

        # articulation
        self.robot: ArticulationCfg = G1_CFG.replace(
            prim_path="/World/envs/env_.*/Robot"
        )
        self.contact_sensor: ContactSensorCfg = ContactSensorCfg(
            prim_path="/World/envs/env_.*/Robot/.*",
            filter_prim_paths_expr=[
                f"/World/objects/object_{i}" for i in range(num_objects)
            ]
            + ["/World/terrain"],
        )

@configclass
class SMPLXSceneCfg(InteractiveSceneCfg):
    """Configuration for a cart-pole scene."""

    def __init__(self, num_objects: int, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # lights
        self.dome_light = AssetBaseCfg(
            prim_path="/World/Light",
            spawn=sim_utils.DomeLightCfg(intensity=3000.0, color=(0.75, 0.75, 0.75)),
        )

        # articulation
        self.robot: ArticulationCfg = SMPLX_CFG.replace(
            prim_path="/World/envs/env_.*/Robot"
        )
        self.contact_sensor: ContactSensorCfg = ContactSensorCfg(
            prim_path="/World/envs/env_.*/Robot/bodies/.*",
            filter_prim_paths_expr=[
                f"/World/objects/object_{i}" for i in range(num_objects)
            ]
            + ["/World/terrain"],
        )
