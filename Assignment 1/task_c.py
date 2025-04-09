

import pinocchio as pin
from pinocchio.visualize import MeshcatVisualizer
import numpy as np
import os

def main():
    # ========== [Step 1] Load URDF and visualize the robot ==========
    print("=== Step 1: URDF Loading & Forward Kinematics ===")

    # 1) Set path to model/URDF
    model_path = os.path.abspath(os.path.dirname(__file__))  # path to /src
    urdf_filename = "go2_leg_description.urdf"
    urdf_path = os.path.join(model_path, urdf_filename)
    mesh_dir = model_path  # "dae" folder is assumed to be inside `model_path`

    # 2) Load model from URDF
    model, collision_model, visual_model = pin.buildModelsFromUrdf(
        urdf_path, package_dirs=[mesh_dir]
    )
    data = model.createData()
    viz = MeshcatVisualizer(model, collision_model, visual_model)

    # 3) Launch Meshcat and display robot at default config (q=0)
    viz.initViewer(open=True)
    viz.loadViewerModel()
    viz.display(np.zeros(model.nq))
    print("URDF model loaded and visualized in Meshcat!")
    print(f"Number of DoFs: nq = {model.nq}, nv = {model.nv}")

    # 4) Forward Kinematics for two configurations (q1, q2)
    foot_id = model.getFrameId("FR_foot")

    # -- Configuration 1
    print("\n== Computing foot position for Configuration 1 ==")
    q1 = np.array([np.pi/12, np.pi/2, -2*np.pi/3])  # 3 dof
    q1_full = np.zeros(model.nq)
    q1_full[:3] = q1  # assume front leg is in the first 3 dofs

    pin.forwardKinematics(model, data, q1_full)
    pin.updateFramePlacements(model, data)
    foot_pos_1 = data.oMf[foot_id].translation

    print("q1 =", q1)
    print("FR_foot position in world frame:", foot_pos_1)
    viz.display(q1_full)
    input("Press Enter to continue to Configuration 2...")

    # -- Configuration 2
    print("\n== Computing foot position for Configuration 2 ==")
    q2 = np.array([-np.pi/6, np.pi/8, -np.pi/3])
    q2_full = np.zeros(model.nq)
    q2_full[:3] = q2

    pin.forwardKinematics(model, data, q2_full)
    pin.updateFramePlacements(model, data)
    foot_pos_2 = data.oMf[foot_id].translation

    print("q2 =", q2)
    print("FR_foot position in world frame:", foot_pos_2)
    viz.display(q2_full)
    input("Press Enter to proceed to Step 2...")

    # ========== [Step 2] Jacobian and velocity mapping ==========
    print("\n=== Step 2: Jacobian-based velocity mapping ===")

    # ========== 2a) Velocity mapping in Configuration 1 ==========
    print("\n== Velocity mapping in Configuration 1 ==")
    dq1 = np.array([-5*np.pi/12, np.pi/2, np.pi/4])  # joint velocity
    dq1_full = np.zeros(model.nv)
    dq1_full[:3] = dq1

    # Recompute kinematics at q1_full
    pin.forwardKinematics(model, data, q1_full)
    pin.updateFramePlacements(model, data)
    # Compute the Jacobian (LOCAL_WORLD_ALIGNED => expressed in world frame)
    J1 = pin.getFrameJacobian(model, data, foot_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)
    
    # The linear velocity is the top 3 rows of J1 times dq
    v_foot_1 = J1[:3, :] @ dq1_full
    print("Joint velocity dq1 =", dq1)
    print("Foot linear velocity from J*q̇ =", v_foot_1)

    input("✅ Press Enter to compute inverse Jacobian mapping in Configuration 2...")

    # ========== 2b) Inverse velocity mapping in Configuration 2 ==========
    print("\n== Inverse velocity mapping in Configuration 2 ==")

    # We want the foot to have a target linear velocity
    v_foot_2_desired = np.array([0.5, -1.0, -0.3])  # m/s

    # Recompute kinematics & Jacobian at q2_full
    pin.forwardKinematics(model, data, q2_full)
    pin.updateFramePlacements(model, data)
    J2 = pin.getFrameJacobian(model, data, foot_id, pin.ReferenceFrame.LOCAL_WORLD_ALIGNED)

    # For a 3-dof leg, we can just extract the 3x3 linear part
    J2_linear = J2[:3, :3]

    # Then solve dq2 = pinv(J2_linear) * v_foot_2_desired
    dq2_est = np.linalg.pinv(J2_linear) @ v_foot_2_desired

    print("Desired foot velocity:", v_foot_2_desired)
    print("Estimated joint velocity dq2 =", dq2_est)
    input("Press Enter to proceed to Step 3...")

    # ========== [Step 3] Torque required at joints in configuration 1 ==========
    print("\n=== Step 3: RNEA - Compute joint torques in Configuration 1 ===")

    # Still in config1 => we have q1_full, dq1_full, now define ddq1
    ddq1 = np.array([1.5, -2.0, 0.5])  # rad/s² 
    ddq1_full = np.zeros(model.nv)
    ddq1_full[:3] = ddq1

    # Use RNEA to compute joint torques τ
    tau = pin.rnea(model, data, q1_full, dq1_full, ddq1_full)

    # Output torques for the first 3 joints
    print("Joint accelerations ddq1 =", ddq1)
    print("Computed torques τ =", tau[:3])
    input("Press Enter to proceed to Step 4...")

    # ========== [Step 4] Forward dynamics (ABA) to compute joint accelerations in config 2 ==========
    print("\n=== Step 4: ABA - Compute joint accelerations in Configuration 2 ===")

    # dq2_est was found above from the inverse velocity mapping
    # Here we consider zero external torques => tau2 = 0
    tau2 = np.zeros(model.nv)

    # Use ABA to compute ddq2
    ddq2 = pin.aba(model, data, q2_full, dq2_est, tau2)

    print("dq2_est (joint velocities for config2) =", dq2_est)
    print("Applied joint torque tau2 =", tau2[:3])
    print("Resulting joint accelerations ddq2 =", ddq2[:3])

    input("Press Enter to exit the program.")

if __name__ == "__main__":
    main()
