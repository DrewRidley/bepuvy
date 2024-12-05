use bepuvy::{BepuvyPlugin, PhysicsResources, RigidBody, StaticBody};
use bepuvy_sys::bepu::interop_math::{Quaternion, RigidPose, Vector3};
use bevy::{
    color::palettes::css::{BLUE, GREEN, RED, YELLOW},
    prelude::*,
};

fn main() {
    App::new()
        .add_plugins(DefaultPlugins)
        .add_plugins(BepuvyPlugin) // Add the physics plugin
        .add_systems(Startup, setup)
        .run();
}

fn setup(
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<StandardMaterial>>,
    physics: Res<PhysicsResources>,
) {
    // Create physics shapes
    let cube_shape = physics.create_box_shape(1.0, 1.0, 1.0);
    let ground_shape = physics.create_box_shape(100.0, 1.0, 100.0);

    // Create ground physics body
    let ground_pose = RigidPose::new(Vector3::new(0.0, -0.5, 0.0), Quaternion::identity());
    let ground_handle = physics.create_static_body(ground_pose, ground_shape);

    // Spawn ground
    commands.spawn((
        Mesh3d(meshes.add(Mesh::from(Cuboid::new(100.0, 0.01, 100.0)))),
        MeshMaterial3d(materials.add(StandardMaterial::from_color(RED))),
        StaticBody {
            handle: ground_handle,
        },
    ));

    // Create cube physics body
    let cube_pose = RigidPose::new(Vector3::new(0.0, 5.0, 0.0), Quaternion::identity());
    let cube_handle = physics.create_dynamic_body(cube_pose, cube_shape, 1.0);

    commands.spawn((
        Mesh3d(meshes.add(Mesh::from(Cuboid::new(1.0, 1.0, 1.0)))),
        MeshMaterial3d(materials.add(StandardMaterial::from_color(BLUE))),
        Transform::from_xyz(0.0, 5.0, 0.0),
        RigidBody {
            handle: cube_handle,
        },
    ));

    commands.spawn((PointLight::default(), Transform::from_xyz(4.0, 8.0, 4.0)));

    commands.spawn((
        Camera3d::default(),
        Transform::from_xyz(-8.0, 8.0, 8.0).looking_at(Vec3::ZERO, Vec3::Y),
    ));
}
