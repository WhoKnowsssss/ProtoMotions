import numpy as np
import torch
from typing import TYPE_CHECKING, Literal

from isaaclab.assets import Articulation, RigidObject
import isaaclab.utils.math as math_utils

def randomize_rigid_body_material(
    robot: RigidObject | Articulation,
    static_friction_range: tuple[float, float],
    dynamic_friction_range: tuple[float, float],
    restitution_range: tuple[float, float],
    num_buckets: int,
    num_envs: int,
    body_names: list[str],
):
    """Randomize the physics materials on all geometries of the asset.

    This function creates a set of physics materials with random static friction, dynamic friction, and restitution
    values. The number of materials is specified by ``num_buckets``. The materials are generated by sampling
    uniform random values from the given ranges.

    The material properties are then assigned to the geometries of the asset. The assignment is done by
    creating a random integer tensor of shape  (num_instances, max_num_shapes) where ``num_instances``
    is the number of assets spawned and ``max_num_shapes`` is the maximum number of shapes in the asset (over
    all bodies). The integer values are used as indices to select the material properties from the
    material buckets.

    .. attention::
        This function uses CPU tensors to assign the material properties. It is recommended to use this function
        only during the initialization of the environment. Otherwise, it may lead to a significant performance
        overhead.

    .. note::
        PhysX only allows 64000 unique physics materials in the scene. If the number of materials exceeds this
        limit, the simulation will crash.
    """
    # extract the used quantities (to enable type-hinting)
    env_ids = torch.arange(num_envs, device="cpu")

    # retrieve material buffer
    materials = robot.root_physx_view.get_material_properties()

    # sample material properties from the given ranges
    material_samples = np.zeros(materials.shape)
    material_samples[..., 0] = np.random.uniform(*static_friction_range)
    material_samples[..., 1] = np.random.uniform(*dynamic_friction_range)
    material_samples[..., 2] = np.random.uniform(*restitution_range)

    # create uniform range tensor for bucketing
    lo = np.array([static_friction_range[0], dynamic_friction_range[0], restitution_range[0]])
    hi = np.array([static_friction_range[1], dynamic_friction_range[1], restitution_range[1]])

    # to avoid 64k material limit in physx, we bucket materials by binning randomized material properties
    # into buckets based on the number of buckets specified
    for d in range(3):
        buckets = np.array([(hi[d] - lo[d]) * i / num_buckets + lo[d] for i in range(num_buckets)])
        material_samples[..., d] = buckets[np.searchsorted(buckets, material_samples[..., d]) - 1]

    # update material buffer with new samples
    if isinstance(robot, Articulation):
        # obtain number of shapes per body (needed for indexing the material properties correctly)
        # note: this is a workaround since the Articulation does not provide a direct way to obtain the number of shapes
        #  per body. We use the physics simulation view to obtain the number of shapes per body.
        num_shapes_per_body = []
        link_names = robot.root_physx_view.link_paths[0]
        body_ids = []
        for idx, link_path in enumerate(link_names):
            link_physx_view = robot._physics_sim_view.create_rigid_body_view(link_path)  # type: ignore
            num_shapes_per_body.append(link_physx_view.max_shapes)
            if len([body_name for body_name in body_names if body_name in link_path]) > 0:
                body_ids.append(idx)

        # sample material properties from the given ranges
        for body_id in body_ids:
            # start index of shape
            start_idx = sum(num_shapes_per_body[:body_id])
            # end index of shape
            end_idx = start_idx + num_shapes_per_body[body_id]
            # assign the new materials
            # material ids are of shape: num_env_ids x num_shapes
            # material_buckets are of shape: num_buckets x 3
            materials[:, start_idx:end_idx] = torch.from_numpy(material_samples[:, start_idx:end_idx]).to(
                dtype=torch.float
            )
    else:
        materials = torch.from_numpy(material_samples).to(dtype=torch.float)

    # apply to simulation
    robot.root_physx_view.set_material_properties(materials, env_ids)

def _randomize_prop_by_op(
    data: torch.Tensor,
    distribution_parameters: tuple[float | torch.Tensor, float | torch.Tensor],
    dim_0_ids: torch.Tensor | None,
    dim_1_ids: torch.Tensor | slice,
    operation: Literal["add", "scale", "abs"],
    distribution: Literal["uniform", "log_uniform", "gaussian"],
) -> torch.Tensor:
    """Perform data randomization based on the given operation and distribution.

    Args:
        data: The data tensor to be randomized. Shape is (dim_0, dim_1).
        distribution_parameters: The parameters for the distribution to sample values from.
        dim_0_ids: The indices of the first dimension to randomize.
        dim_1_ids: The indices of the second dimension to randomize.
        operation: The operation to perform on the data. Options: 'add', 'scale', 'abs'.
        distribution: The distribution to sample the random values from. Options: 'uniform', 'log_uniform'.

    Returns:
        The data tensor after randomization. Shape is (dim_0, dim_1).

    Raises:
        NotImplementedError: If the operation or distribution is not supported.
    """
    # resolve shape
    # -- dim 0
    if dim_0_ids is None:
        n_dim_0 = data.shape[0]
        dim_0_ids = slice(None)
    else:
        n_dim_0 = len(dim_0_ids)
        dim_0_ids = dim_0_ids[:, None]
    # -- dim 1
    if isinstance(dim_1_ids, slice):
        n_dim_1 = data.shape[1]
    else:
        n_dim_1 = len(dim_1_ids)

    # resolve the distribution
    if distribution == "uniform":
        dist_fn = math_utils.sample_uniform
    elif distribution == "log_uniform":
        dist_fn = math_utils.sample_log_uniform
    elif distribution == "gaussian":
        dist_fn = math_utils.sample_gaussian
    else:
        raise NotImplementedError(
            f"Unknown distribution: '{distribution}' for joint properties randomization."
            " Please use 'uniform', 'log_uniform', 'gaussian'."
        )
    # perform the operation
    if operation == "add":
        data[dim_0_ids, dim_1_ids] += dist_fn(*distribution_parameters, (n_dim_0, n_dim_1), device=data.device)
    elif operation == "scale":
        data[dim_0_ids, dim_1_ids] *= dist_fn(*distribution_parameters, (n_dim_0, n_dim_1), device=data.device)
    elif operation == "abs":
        data[dim_0_ids, dim_1_ids] = dist_fn(*distribution_parameters, (n_dim_0, n_dim_1), device=data.device)
    else:
        raise NotImplementedError(
            f"Unknown operation: '{operation}' for property randomization. Please use 'add', 'scale', or 'abs'."
        )
    return data

def randomize_rigid_body_mass(
    robot: RigidObject | Articulation,
    mass_distribution_params: tuple[float, float],
    operation: Literal["add", "scale", "abs"],
    num_envs: int,
    body_names: list[str],
    distribution: Literal["uniform", "log_uniform", "gaussian"] = "uniform",
    recompute_inertia: bool = True,
):
    """Randomize the mass of the bodies by adding, scaling, or setting random values.

    This function allows randomizing the mass of the bodies of the asset. The function samples random values from the
    given distribution parameters and adds, scales, or sets the values into the physics simulation based on the operation.

    If the ``recompute_inertia`` flag is set to ``True``, the function recomputes the inertia tensor of the bodies
    after setting the mass. This is useful when the mass is changed significantly, as the inertia tensor depends
    on the mass. It assumes the body is a uniform density object. If the body is not a uniform density object,
    the inertia tensor may not be accurate.

    .. tip::
        This function uses CPU tensors to assign the body masses. It is recommended to use this function
        only during the initialization of the environment.
    """
    # extract the used quantities (to enable type-hinting)
    env_ids = torch.arange(num_envs, device="cpu")

    # get the current masses of the bodies (num_assets, num_bodies)
    masses = robot.root_physx_view.get_masses()

    link_names = robot.root_physx_view.link_paths[0]
    body_ids = []
    for idx, link_path in enumerate(link_names):
        if len([body_name for body_name in body_names if body_name in link_path]) > 0:
            body_ids.append(idx)

    # apply randomization on default values
    # this is to make sure when calling the function multiple times, the randomization is applied on the
    # default values and not the previously randomized values
    masses[env_ids[:, None], body_ids] = robot.data.default_mass[env_ids[:, None], body_ids].clone()
    # sample from the given range
    # note: we modify the masses in-place for all environments
    #   however, the setter takes care that only the masses of the specified environments are modified
    masses = _randomize_prop_by_op(
        masses, mass_distribution_params, env_ids, body_ids, operation=operation, distribution=distribution
    )

    # set the mass into the physics simulation
    robot.root_physx_view.set_masses(masses, env_ids)

    # recompute inertia tensors if needed
    if recompute_inertia:
        # compute the ratios of the new masses to the initial masses
        ratios = masses[env_ids[:, None], body_ids] / robot.data.default_mass[env_ids[:, None], body_ids]
        # scale the inertia tensors by the the ratios
        # since mass randomization is done on default values, we can use the default inertia tensors
        inertias = robot.root_physx_view.get_inertias()
        if isinstance(robot, Articulation):
            # inertia has shape: (num_envs, num_bodies, 9) for articulation
            inertias[env_ids[:, None], body_ids] = (
                robot.data.default_inertia[env_ids[:, None], body_ids] * ratios[..., None]
            )
        else:
            # inertia has shape: (num_envs, 9) for rigid object
            inertias[env_ids] = robot.data.default_inertia[env_ids] * ratios
        # set the inertia tensors into the physics simulation
        robot.root_physx_view.set_inertias(inertias, env_ids)

def randomize_joint_default_pos(
    robot: RigidObject | Articulation,
    num_envs: int,
    pos_distribution_params: tuple[float, float] | None = None,
    operation: Literal["add", "scale", "abs"] = "add",
    distribution: Literal["uniform", "log_uniform", "gaussian"] = "uniform",
):
    """
    Randomize the joint default positions which may be different from URDF due to calibration errors.
    """
    env_ids = torch.arange(num_envs, device="cpu")
    joint_ids = torch.arange(robot.num_joints, device="cpu")
    
    if pos_distribution_params is not None:
        pos = robot.data.default_joint_pos.to(robot.device).clone()
        pos = _randomize_prop_by_op(
            pos, pos_distribution_params, env_ids, joint_ids, operation=operation, distribution=distribution
        )[env_ids][:, joint_ids]

        if env_ids != slice(None) and joint_ids != slice(None):
            env_ids = env_ids[:, None]
        robot.data.default_joint_pos[env_ids, joint_ids] = pos

def push_by_setting_velocity(
    robot: RigidObject | Articulation,
    velocity_range: dict[str, tuple[float, float]],
    num_envs: int,
):
    """Push the asset by setting the root velocity to a random value within the given ranges.

    This creates an effect similar to pushing the asset with a random impulse that changes the asset's velocity.
    It samples the root velocity from the given ranges and sets the velocity into the physics simulation.

    The function takes a dictionary of velocity ranges for each axis and rotation. The keys of the dictionary
    are ``x``, ``y``, ``z``, ``roll``, ``pitch``, and ``yaw``. The values are tuples of the form ``(min, max)``.
    If the dictionary does not contain a key, the velocity is set to zero for that axis.
    """
    # extract the used quantities (to enable type-hinting)
    env_ids = torch.arange(num_envs, device=robot.device)

    # velocities
    vel_w = robot.data.root_vel_w[env_ids]
    # sample random velocities
    range_list = [velocity_range.get(key, (0.0, 0.0)) for key in ["x", "y", "z", "roll", "pitch", "yaw"]]
    ranges = torch.tensor(range_list, device=robot.device)
    vel_w[:] = math_utils.sample_uniform(ranges[:, 0], ranges[:, 1], vel_w.shape, device=robot.device)
    # set the velocities into the physics simulation
    robot.write_root_velocity_to_sim(vel_w, env_ids=env_ids)