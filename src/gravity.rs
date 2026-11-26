//! Gravity torque from a URDF: what jt's t_ff is, `ocs2::PinocchioInterfaceTpl::getTorque_each_joint_with_gravity`
//! (ocs2pi@0x1ab4a0), which is `pinocchio::computeGeneralizedGravity(model, data, q)` (its
//! ComputeGeneralizedGravityForward/BackwardStep visitors) copied out, with Pinocchio's default gravity (0, 0, -9.81)
//! in the root link's frame.
//!
//! For a revolute joint i, Pinocchio's g(q)_i is the torque that holds every body below it against gravity:
//! sum over those bodies k of (axis_i x (com_k - origin_i)) . (m_k * 9.81 z). Prismatic joints get axis_i . (m g).
//! Joints not listed as movable are frozen at 0 (fixed, and prismatic or revolute alike).

use std::collections::HashMap;

/// Pinocchio's `Model::gravity981` magnitude.
pub const PINOCCHIO_GRAVITY_M_S2: f64 = 9.81;

type Vec3 = [f64; 3];
type Mat3 = [[f64; 3]; 3];

fn add(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn sub(a: Vec3, b: Vec3) -> Vec3 {
    [a[0] - b[0], a[1] - b[1], a[2] - b[2]]
}

fn scale(a: Vec3, s: f64) -> Vec3 {
    [a[0] * s, a[1] * s, a[2] * s]
}

fn dot(a: Vec3, b: Vec3) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: Vec3, b: Vec3) -> Vec3 {
    [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]]
}

fn mat_vec(m: &Mat3, v: Vec3) -> Vec3 {
    [dot(m[0], v), dot(m[1], v), dot(m[2], v)]
}

fn mat_mul(a: &Mat3, b: &Mat3) -> Mat3 {
    std::array::from_fn(|r| std::array::from_fn(|c| a[r][0] * b[0][c] + a[r][1] * b[1][c] + a[r][2] * b[2][c]))
}

const IDENTITY: Mat3 = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]];

/// URDF rpy: R = Rz(yaw) Ry(pitch) Rx(roll).
fn rpy(roll: f64, pitch: f64, yaw: f64) -> Mat3 {
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

/// Rotation by `angle` about the unit `axis` (Rodrigues).
fn axis_angle(axis: Vec3, angle: f64) -> Mat3 {
    let (s, c) = angle.sin_cos();
    let t = 1.0 - c;
    let [x, y, z] = axis;
    [
        [t * x * x + c, t * x * y - s * z, t * x * z + s * y],
        [t * x * y + s * z, t * y * y + c, t * y * z - s * x],
        [t * x * z - s * y, t * y * z + s * x, t * z * z + c],
    ]
}

#[derive(Clone, Debug, PartialEq)]
enum JointKind {
    Fixed,
    Revolute,
    Prismatic,
}

#[derive(Clone, Debug)]
struct Joint {
    name: String,
    kind: JointKind,
    parent: String,
    child: String,
    origin_xyz: Vec3,
    origin_rotation: Mat3,
    axis: Vec3,
}

#[derive(Clone, Debug, Default)]
struct Inertial {
    mass_kg: f64,
    com: Vec3,
}

/// A URDF's kinematic tree with its masses, and which joints move.
#[derive(Clone, Debug)]
pub struct GravityModel {
    root: String,
    joints: Vec<Joint>,
    inertials: HashMap<String, Inertial>,
    /// Movable joint names, in the order of `q` and of the result.
    movable: Vec<String>,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct UrdfError(pub String);

impl std::fmt::Display for UrdfError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(formatter, "urdf: {}", self.0)
    }
}

impl std::error::Error for UrdfError {}

fn numbers<const N: usize>(text: Option<&str>, default: [f64; N]) -> Result<[f64; N], UrdfError> {
    let Some(text) = text else { return Ok(default) };
    let values: Vec<f64> = text
        .split_whitespace()
        .map(|word| word.parse::<f64>().map_err(|_| UrdfError(format!("bad number {word:?}"))))
        .collect::<Result<_, _>>()?;
    values.try_into().map_err(|_| UrdfError(format!("expected {N} numbers in {text:?}")))
}

impl GravityModel {
    /// Parse `urdf`; `movable` names the joints of `q`, every other joint is frozen at 0.
    pub fn from_urdf(urdf: &str, movable: &[&str]) -> Result<GravityModel, UrdfError> {
        let document = roxmltree::Document::parse(urdf).map_err(|error| UrdfError(error.to_string()))?;
        let robot = document.root_element();
        let mut inertials = HashMap::new();
        let mut links = Vec::new();
        for link in robot.children().filter(|node| node.has_tag_name("link")) {
            let name = link.attribute("name").ok_or(UrdfError("link without a name".into()))?.to_string();
            links.push(name.clone());
            let Some(inertial) = link.children().find(|node| node.has_tag_name("inertial")) else { continue };
            let child = |tag: &str| inertial.children().find(|node| node.has_tag_name(tag));
            let mass_kg = child("mass")
                .and_then(|mass| mass.attribute("value"))
                .ok_or(UrdfError(format!("{name}: inertial without a mass")))?
                .parse::<f64>()
                .map_err(|_| UrdfError(format!("{name}: bad mass")))?;
            let com = numbers(child("origin").and_then(|origin| origin.attribute("xyz")), [0.0; 3])?;
            inertials.insert(name, Inertial { mass_kg, com });
        }
        let mut joints = Vec::new();
        for joint in robot.children().filter(|node| node.has_tag_name("joint")) {
            let name = joint.attribute("name").ok_or(UrdfError("joint without a name".into()))?.to_string();
            let child = |tag: &str| joint.children().find(|node| node.has_tag_name(tag));
            let link = |tag: &str| {
                child(tag)
                    .and_then(|node| node.attribute("link"))
                    .map(str::to_string)
                    .ok_or(UrdfError(format!("{name}: no {tag}")))
            };
            let kind = match joint.attribute("type") {
                Some("revolute") | Some("continuous") => JointKind::Revolute,
                Some("prismatic") => JointKind::Prismatic,
                _ => JointKind::Fixed,
            };
            let origin = child("origin");
            let [roll, pitch, yaw] = numbers(origin.and_then(|node| node.attribute("rpy")), [0.0; 3])?;
            let axis = numbers(child("axis").and_then(|node| node.attribute("xyz")), [1.0, 0.0, 0.0])?;
            let norm = dot(axis, axis).sqrt();
            joints.push(Joint {
                kind,
                parent: link("parent")?,
                child: link("child")?,
                origin_xyz: numbers(origin.and_then(|node| node.attribute("xyz")), [0.0; 3])?,
                origin_rotation: rpy(roll, pitch, yaw),
                axis: if norm > 0.0 { scale(axis, 1.0 / norm) } else { axis },
                name,
            });
        }
        let root = links
            .iter()
            .find(|link| !joints.iter().any(|joint| &joint.child == *link))
            .ok_or(UrdfError("no root link".into()))?
            .clone();
        for name in movable {
            if !joints.iter().any(|joint| joint.name == *name && joint.kind != JointKind::Fixed) {
                return Err(UrdfError(format!("no movable joint {name}")));
            }
        }
        Ok(GravityModel { root, joints, inertials, movable: movable.iter().map(|name| name.to_string()).collect() })
    }

    pub fn movable(&self) -> &[String] {
        &self.movable
    }

    /// World pose of every link and world origin, axis and kind of every joint at `q` (movable order).
    #[allow(clippy::type_complexity)]
    fn forward(&self, q: &[f64]) -> (HashMap<&str, (Vec3, Mat3)>, HashMap<&str, (Vec3, Vec3, &JointKind)>) {
        let angle: HashMap<&str, f64> =
            self.movable.iter().map(String::as_str).zip(q.iter().copied().chain(std::iter::repeat(0.0))).collect();
        let mut link_pose: HashMap<&str, (Vec3, Mat3)> = HashMap::from([(self.root.as_str(), ([0.0; 3], IDENTITY))]);
        let mut joint_frame: HashMap<&str, (Vec3, Vec3, &JointKind)> = HashMap::new();
        let mut frontier = vec![self.root.as_str()];
        while let Some(parent) = frontier.pop() {
            let (parent_position, parent_rotation) = link_pose[parent];
            for joint in self.joints.iter().filter(|joint| joint.parent == parent) {
                let position = add(parent_position, mat_vec(&parent_rotation, joint.origin_xyz));
                let rotation = mat_mul(&parent_rotation, &joint.origin_rotation);
                let world_axis = mat_vec(&rotation, joint.axis);
                let value = angle.get(joint.name.as_str()).copied().unwrap_or(0.0);
                let (position, rotation) = match joint.kind {
                    JointKind::Fixed => (position, rotation),
                    JointKind::Revolute => (position, mat_mul(&rotation, &axis_angle(joint.axis, value))),
                    JointKind::Prismatic => (add(position, scale(world_axis, value)), rotation),
                };
                joint_frame.insert(joint.name.as_str(), (position, world_axis, &joint.kind));
                link_pose.insert(joint.child.as_str(), (position, rotation));
                frontier.push(joint.child.as_str());
            }
        }
        (link_pose, joint_frame)
    }

    /// `link`'s position and KDL `GetRPY` (roll, pitch, yaw) in the root frame at `q`.
    pub fn link_pose(&self, q: &[f64], link: &str) -> Option<(Vec3, Vec3)> {
        let (poses, _) = self.forward(q);
        let (position, r) = *poses.get(link)?;
        let roll = r[2][1].atan2(r[2][2]);
        let pitch = (-r[2][0]).atan2((r[0][0] * r[0][0] + r[1][0] * r[1][0]).sqrt());
        let yaw = r[1][0].atan2(r[0][0]);
        Some((position, [roll, pitch, yaw]))
    }

    /// g(q): the torque (N m, or N for a prismatic joint) each movable joint needs to hold the robot still.
    pub fn torques(&self, q: &[f64]) -> Vec<f64> {
        let (link_pose, joint_frame) = self.forward(q);
        let below = |joint: &str| -> Vec<&str> {
            let mut links = Vec::new();
            let mut stack: Vec<&str> =
                self.joints.iter().filter(|j| j.name == joint).map(|j| j.child.as_str()).collect();
            while let Some(link) = stack.pop() {
                links.push(link);
                stack.extend(self.joints.iter().filter(|j| j.parent == link).map(|j| j.child.as_str()));
            }
            links
        };
        let up = [0.0, 0.0, PINOCCHIO_GRAVITY_M_S2];
        self.movable
            .iter()
            .map(|name| {
                let Some(&(origin, axis, kind)) = joint_frame.get(name.as_str()) else { return 0.0 };
                below(name)
                    .into_iter()
                    .filter_map(|link| Some((self.inertials.get(link)?, link_pose.get(link)?)))
                    .map(|(inertial, (position, rotation))| {
                        let weight = scale(up, inertial.mass_kg);
                        match kind {
                            JointKind::Prismatic => dot(axis, weight),
                            _ => {
                                let com = add(*position, mat_vec(rotation, inertial.com));
                                dot(cross(axis, sub(com, origin)), weight)
                            }
                        }
                    })
                    .sum()
            })
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const PENDULUM: &str = r#"<robot name="p">
        <link name="base"/>
        <link name="arm"><inertial><origin xyz="0.5 0 0"/><mass value="2"/></inertial></link>
        <link name="tip"><inertial><origin xyz="0 0 0"/><mass value="1"/></inertial></link>
        <joint name="shoulder" type="revolute"><parent link="base"/><child link="arm"/><axis xyz="0 1 0"/></joint>
        <joint name="wrist" type="fixed"><parent link="arm"/><child link="tip"/><origin xyz="1 0 0"/></joint>
    </robot>"#;

    #[test]
    fn horizontal_pendulum_needs_its_moment() {
        let model = GravityModel::from_urdf(PENDULUM, &["shoulder"]).unwrap();
        // arm along +x, gravity -z, y axis: holding torque about +y is -(2 * 0.5 + 1 * 1) * 9.81
        let horizontal = model.torques(&[0.0]);
        assert!((horizontal[0] + 2.0 * 9.81).abs() < 1e-12, "{horizontal:?}");
        // pointing straight down needs nothing
        let down = model.torques(&[std::f64::consts::FRAC_PI_2]);
        assert!(down[0].abs() < 1e-12, "{down:?}");
    }

    #[test]
    fn unknown_joints_are_rejected() {
        assert!(GravityModel::from_urdf(PENDULUM, &["elbow"]).is_err());
        assert!(GravityModel::from_urdf(PENDULUM, &["wrist"]).is_err());
    }
}
