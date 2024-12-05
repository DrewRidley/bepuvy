use bepuvy::{BepuvyPlugin, PhysicsResources, RigidBody, StaticBody};
use bepuvy_sys::bepu::interop_math::{Quaternion, RigidPose, Vector3};
use bevy::core::TaskPoolThreadAssignmentPolicy;
use bevy::diagnostic::FrameTimeDiagnosticsPlugin;
use bevy::tasks::available_parallelism;
use bevy::{
    color::palettes::css::{BLUE, RED},
    prelude::*,
};
use iyes_perf_ui::prelude::PerfUiDefaultEntries;
use iyes_perf_ui::PerfUiPlugin;
use rand::Rng;

fn main() {
    App::new()
        .add_plugins(DefaultPlugins.set(TaskPoolPlugin {
            task_pool_options: TaskPoolOptions {
                compute: TaskPoolThreadAssignmentPolicy {
                    percent: 0.5, // this value is irrelevant in this case
                    min_threads: 1,
                    max_threads: available_parallelism(),
                },
                // keep the defaults for everything else
                ..default()
            },
        }))
        .add_plugins(BepuvyPlugin)
        .add_plugins(bevy::diagnostic::FrameTimeDiagnosticsPlugin)
        .add_plugins(PerfUiPlugin)
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    physics: Res<PhysicsResources>,
) {
    commands.spawn(PerfUiDefaultEntries::default());

    // Ground setup (same as before)
    let ground_shape = physics.create_box_shape(100.0, 1.0, 100.0);
    let ground_pose = RigidPose::new(Vector3::new(0.0, -0.5, 0.0), Quaternion::identity());
    let ground_handle = physics.create_static_body(ground_pose, ground_shape);

    commands.spawn((
        Mesh3d(meshes.add(Mesh::from(Cuboid::new(100.0, 0.01, 100.0)))),
        MeshMaterial3d(materials.add(StandardMaterial::from_color(RED))),
        StaticBody {
            handle: ground_handle,
        },
    ));

    // Create shared resources for cubes
    let cube_mesh = meshes.add(Mesh::from(Cuboid::new(1.0, 1.0, 1.0)));
    let cube_material = materials.add(StandardMaterial::from_color(BLUE));
    let cube_shape = physics.create_box_shape(1.0, 1.0, 1.0);

    let mut rng = rand::thread_rng();

    // Spawn cubes in a tower formation
    let cube_count = 32_000;
    let base_size = 20; // Width/depth of the tower base
    let spacing = 4.0; // Space between cubes

    for i in 0..cube_count {
        // Calculate position in a tower formation
        let layer = i / (base_size * base_size); // Current vertical layer
        let remaining = i % (base_size * base_size);
        let row = remaining / base_size;
        let col = remaining % base_size;

        // Calculate position with slight random offset for more interesting physics
        let x = ((col as f32) - base_size as f32 / 2.0) * spacing + rng.gen_range(-0.1..0.1);
        let z = ((row as f32) - base_size as f32 / 2.0) * spacing + rng.gen_range(-0.1..0.1);
        let y = layer as f32 * spacing + 1.0 + rng.gen_range(-0.05..0.05);

        // Create cube physics body
        let cube_pose = RigidPose::new(Vector3::new(x, y, z), Quaternion::identity());
        let cube_handle = physics.create_dynamic_body(cube_pose, cube_shape.clone(), 1.0);

        // Spawn cube entity
        commands.spawn((
            Mesh3d(cube_mesh.clone()),
            MeshMaterial3d(cube_material.clone()),
            Transform::from_xyz(x, y, z),
            RigidBody {
                handle: cube_handle,
            },
        ));
    }

    // Adjust camera position to better view the tower
    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-50.0, 50.0, 50.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));

    // Light
    commands.spawn((
        PointLight {
            intensity: 3000.0,
            shadows_enabled: true,
            ..default()
        },
        Transform::from_xyz(4.0, 8.0, 4.0),
    ));
}
