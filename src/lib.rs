use bepuvy_sys::bepu::{
    bodies::*, collisions::*, continuity::*, functions::*, handles::*, interop_math::*,
    pose_integration::*, shapes::*, utilities::*, *,
};
use bevy::prelude::*;
use constraints::SpringSettings;
use statics::StaticDescription;

pub struct BepuvyPlugin;

impl Plugin for BepuvyPlugin {
    fn build(&self, app: &mut App) {
        unsafe {
            Initialize();
        }

        let buffer_pool = unsafe { CreateBufferPool(131072, 16) };

        let thread_dispatcher = unsafe { CreateThreadDispatcher(GetPlatformThreadCount(), 16384) };

        // Use the same callbacks from the basic example
        let simulation = unsafe {
            CreateSimulation(
                buffer_pool,
                create_default_narrow_phase_callbacks(),
                create_default_pose_integrator_callbacks(),
                SolveDescription {
                    velocity_iteration_count: 8,
                    substep_count: 1,
                    fallback_batch_threshold: 128,
                    velocity_iteration_scheduler: None,
                },
                SimulationAllocationSizes {
                    bodies: 4096,
                    statics: 4096,
                    islands: 4096,
                    shapes_per_type: 128,
                    constraints: 4096,
                    constraints_per_type_batch: 128,
                    constraint_count_per_body_estimate: 8,
                },
            )
        };

        app.insert_resource(PhysicsResources {
            buffer_pool,
            thread_dispatcher,
            simulation,
        })
        .add_systems(Update, physics_step);
    }
}

// Add these callback functions:
unsafe extern "C" fn integrate_velocity(
    _simulation: SimulationHandle,
    _body_index: i32,
    _position: Vector3,
    _orientation: Quaternion,
    _local_inertia: BodyInertia,
    _worker_index: i32,
    dt: f32,
    velocity: *mut BodyVelocity,
) {
    let gravity = Vector3::new(0.0, -10.0, 0.0);
    (*velocity).linear.x += gravity.x * dt;
    (*velocity).linear.y += gravity.y * dt;
    (*velocity).linear.z += gravity.z * dt;
}

fn create_default_narrow_phase_callbacks() -> NarrowPhaseCallbacks {
    unsafe extern "C" fn allow_contact_generation(
        _simulation: SimulationHandle,
        _worker_index: i32,
        a: CollidableReference,
        b: CollidableReference,
        _speculative_margin: *mut f32,
    ) -> bool {
        a.mobility() == CollidableMobility::Dynamic || b.mobility() == CollidableMobility::Dynamic
    }

    unsafe extern "C" fn allow_contact_generation_between_children(
        _simulation: SimulationHandle,
        _worker_index: i32,
        _pair: CollidablePair,
        _child_index_a: i32,
        _child_index_b: i32,
    ) -> bool {
        true
    }

    unsafe extern "C" fn configure_contact_manifold(
        _simulation: SimulationHandle,
        _worker_index: i32,
        _pair: CollidablePair,
        _manifold: *mut ConvexContactManifold,
        material: *mut PairMaterialProperties,
    ) -> bool {
        (*material).friction_coefficient = 1.0;
        (*material).maximum_recovery_velocity = 2.0;
        (*material).contact_spring_settings = SpringSettings::new(30.0, 1.0);
        true
    }

    unsafe extern "C" fn configure_nonconvex_contact_manifold(
        _simulation: SimulationHandle,
        _worker_index: i32,
        _pair: CollidablePair,
        _manifold: *mut NonconvexContactManifold,
        material: *mut PairMaterialProperties,
    ) -> bool {
        (*material).friction_coefficient = 1.0;
        (*material).maximum_recovery_velocity = 2.0;
        (*material).contact_spring_settings = SpringSettings::new(30.0, 1.0);
        true
    }

    unsafe extern "C" fn configure_child_contact_manifold(
        _simulation: SimulationHandle,
        _worker_index: i32,
        _pair: CollidablePair,
        _child_index_a: i32,
        _child_index_b: i32,
        _manifold: *mut ConvexContactManifold,
    ) -> bool {
        true
    }

    NarrowPhaseCallbacks {
        initialize_function: None,
        dispose_function: None,
        allow_contact_generation_function: Some(allow_contact_generation),
        allow_contact_generation_between_children_function: Some(
            allow_contact_generation_between_children,
        ),
        configure_convex_contact_manifold_function: Some(configure_contact_manifold),
        configure_nonconvex_contact_manifold_function: Some(configure_nonconvex_contact_manifold),
        configure_child_contact_manifold_function: Some(configure_child_contact_manifold),
    }
}

fn create_default_pose_integrator_callbacks() -> PoseIntegratorCallbacks {
    PoseIntegratorCallbacks {
        angular_integration_mode: AngularIntegrationMode::Nonconserving,
        allow_substeps_for_unconstrained_bodies: false,
        integrate_velocity_for_kinematics: false,
        use_scalar_callback: true,
        initialize: None,
        prepare_for_integration: None,
        integrate_velocity_scalar: Some(integrate_velocity),
        integrate_velocity_simd128: None,
        integrate_velocity_simd256: None,
    }
}

impl Drop for BepuvyPlugin {
    fn drop(&mut self) {
        unsafe {
            Destroy();
        }
    }
}

#[derive(Resource)]
pub struct PhysicsResources {
    pub buffer_pool: BufferPoolHandle,
    pub thread_dispatcher: ThreadDispatcherHandle,
    pub simulation: SimulationHandle,
}

// Components
#[derive(Component)]
pub struct RigidBody {
    pub handle: BodyHandle,
}

#[derive(Component)]
pub struct StaticBody {
    pub handle: StaticHandle,
}

// Main physics step system
fn physics_step(
    time: Res<Time>,
    physics_resources: Res<PhysicsResources>,
    mut transform_query: Query<(&RigidBody, &mut Transform)>,
) {
    let dt = time.delta_secs();

    if (dt <= 0.) {
        return;
    }

    // Step the physics simulation
    unsafe {
        Timestep(
            physics_resources.simulation,
            dt,
            physics_resources.thread_dispatcher,
        );
    }

    // Update transforms from physics bodies
    for (rigidbody, mut transform) in transform_query.iter_mut() {
        let dynamics = unsafe { GetBodyDynamics(physics_resources.simulation, rigidbody.handle) };

        let pos = unsafe {
            Vec3::new(
                (*dynamics).motion.pose.position.x,
                (*dynamics).motion.pose.position.y,
                (*dynamics).motion.pose.position.z,
            )
        };

        transform.translation = pos;
    }
}

// Helper functions for creating bodies
impl PhysicsResources {
    pub fn create_box_shape(&self, width: f32, height: f32, length: f32) -> TypedIndex {
        let box_shape = Box::new(width, height, length);
        unsafe { AddBox(self.simulation, box_shape) }
    }

    pub fn create_sphere_shape(&self, radius: f32) -> TypedIndex {
        let sphere = bepuvy_sys::bepu::shapes::Sphere { radius };
        unsafe { AddSphere(self.simulation, sphere) }
    }

    pub fn create_dynamic_body(&self, pose: RigidPose, shape: TypedIndex, mass: f32) -> BodyHandle {
        let box_shape = Box::new(1.0, 1.0, 1.0);
        let box_inertia = unsafe { ComputeBoxInertia(box_shape, mass) };
        let collidable = CollidableDescription::passive(shape);
        let activity = BodyActivityDescription::new(0.01, 32);

        let body_desc = BodyDescription::create_dynamic(
            pose,
            BodyVelocity::zero(),
            box_inertia,
            collidable,
            activity,
        );

        unsafe { AddBody(self.simulation, body_desc) }
    }

    pub fn create_static_body(&self, pose: RigidPose, shape: TypedIndex) -> StaticHandle {
        let static_desc = StaticDescription::create_with_position_orientation_discrete(
            pose.position,
            pose.orientation,
            shape,
        );

        unsafe { AddStatic(self.simulation, static_desc) }
    }
}