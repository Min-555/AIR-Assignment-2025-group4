import numpy as np
import pinocchio as pin
# from pinocchio.visualize import MeshcatVisualizer

# Set this to True if you want to visualize the robot using Meshcat
use_visualization = False

# Set the path to your URDF and mesh directory
urdf_path = "./Go2_leg_urdf/urdf/go2_leg_description.urdf"
mesh_dir = "./Go2_leg_urdf/meshes/Go2_leg_urdf/dae"

# Load the robot model from the URDF
model = pin.buildModelFromUrdf(urdf_path)
data = model.createData()

# Visualization setup
if use_visualization:
    viz = MeshcatVisualizer(model, model.createData(), model.createData())
    viz.initViewer(open_browser=True)
    viz.loadViewerModel()

# Basic model info
print("Model name:", model.name)
print("Number of joints (nq):", model.nq)
print("Number of DoF (nv):", model.nv)

# Get the foot frame ID
foot_frame_name = "FR_foot"
try:
    foot_frame_id = model.getFrameId(foot_frame_name)
    print("Foot frame ID =", foot_frame_id)
except:
    raise ValueError(f"No frame named {foot_frame_name} found in URDF! Please check the name.")

# Define joint configurations
q_config1 = np.array([ np.pi/12,  np.pi/2, -2*np.pi/3 ])
q_config2 = np.array([-np.pi/6,   np.pi/8, -np.pi/3 ])

# Function to compute 6xN foot Jacobian in WORLD frame
def get_foot_jacobian(model, data, q, foot_frame_id):
    pin.computeJointJacobians(model, data, q)
    pin.updateFramePlacements(model, data)
    J6 = pin.getFrameJacobian(model, data, foot_frame_id, pin.ReferenceFrame.WORLD)
    return J6

# (a) Forward kinematics to get foot pose
pin.forwardKinematics(model, data, q_config1)
pin.updateFramePlacements(model, data)
foot_pose_config1 = data.oMf[foot_frame_id]
print("[Config1] Foot pose:\n", foot_pose_config1)
print("[Config1] Foot translation (x,y,z) =", foot_pose_config1.translation)

pin.forwardKinematics(model, data, q_config2)
pin.updateFramePlacements(model, data)
foot_pose_config2 = data.oMf[foot_frame_id]
print("[Config2] Foot pose:\n", foot_pose_config2)
print("[Config2] Foot translation (x,y,z) =", foot_pose_config2.translation)

# Visualize the joint configurations
if use_visualization:
    viz.display(q_config1)
    viz.display(q_config2)

# (b) Compute Jacobian and perform velocity mapping
J6_config1 = get_foot_jacobian(model, data, q_config1, foot_frame_id)
J6_config2 = get_foot_jacobian(model, data, q_config2, foot_frame_id)

print("\n[Config1] 6×3 Jacobian:\n", J6_config1)
print("[Config2] 6×3 Jacobian:\n", J6_config2)

# Extract linear velocity part (top 3 rows)
Jv_config1 = J6_config1[:3, :]
Jv_config2 = J6_config2[:3, :]

# (1) Given joint velocity => foot velocity (Config1)
qdot_config1 = np.array([-5*np.pi/12, np.pi/2, np.pi/4])  # rad/s
foot_vel_config1 = Jv_config1.dot(qdot_config1)
print("\n[Config1] Given qdot =", qdot_config1, " => foot linear velocity =", foot_vel_config1, " [m/s]")

# (2) Given foot velocity => joint velocity (inverse mapping) (Config2)
xdot_config2 = np.array([0.5, -1.0, -0.3])  # m/s
qdot_config2 = np.linalg.inv(Jv_config2).dot(xdot_config2)
print("[Config2] Given foot xdot =", xdot_config2, " => qdot =", qdot_config2, " [rad/s]")

# Question 3: Dynamics (torque computation using RNEA)
model.gravity.linear = np.array([0, 0, -9.81])
q_config1_dyn   = np.array([ np.pi/12, np.pi/2, -2*np.pi/3 ])
qd_config1_dyn  = np.array([-5*np.pi/12, np.pi/2, np.pi/4])
qdd_config1_dyn = np.array([1.5, -2.0, 0.5])

tau_config1 = pin.rnea(model, data, q_config1_dyn, qd_config1_dyn, qdd_config1_dyn)
print("\n[Config1] Tau =", tau_config1, " [Nm], with qdd =", qdd_config1_dyn, " and gravity = -9.81 m/s²")

# Question 4: Forward dynamics (get qdd from torque input = 0)
q_config2_fd   = np.array([-np.pi/6, np.pi/8, -np.pi/3])
qd_config2_fd  = np.array([-np.pi/6, np.pi/2, -2*np.pi/3])
tau_zero       = np.zeros(model.nv)

qdd_config2_fd = pin.forwardDynamics(model, data, q_config2_fd, qd_config2_fd, tau_zero)
print("\n[Config2] With zero torque => qdd =", qdd_config2_fd, " [rad/s²]")

if use_visualization:
    viz.display(q_config2_fd)
