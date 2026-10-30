import csv
import time
import pybullet as p
import pybullet_data
import robotics_sim


def main():
    # 1. Initialize PyBullet Physics Engine
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, 0)

    # 2. Control Buttons & Target Sliders
    start_btn = p.addUserDebugParameter("Start / Reset Simulation", 1, 0)
    target_x_slider = p.addUserDebugParameter("Target X", -10.0, 10.0, 5.0)
    target_y_slider = p.addUserDebugParameter("Target Y", -10.0, 10.0, 5.0)
    target_z_slider = p.addUserDebugParameter("Target Z", -10.0, 10.0, 5.0)

    # 3. Dynamic PID Controller Parameter Sliders
    kp_slider = p.addUserDebugParameter("Kp (Proportional)", 0.0, 100.0, 35.0)
    ki_slider = p.addUserDebugParameter("Ki (Integral)", 0.0, 10.0, 0.5)
    kd_slider = p.addUserDebugParameter("Kd (Derivative)", 0.0, 50.0, 12.0)
    max_i_slider = p.addUserDebugParameter("Max Integral Clamp", 0.0, 100.0, 20.0)
    max_f_slider = p.addUserDebugParameter("Max Force Saturation", 10.0, 500.0, 150.0)

    # 4. Spawn Quadrotor Drone & Target Marker
    start_pos = [0.0, 0.0, 0.0]
    robot_id = p.loadURDF("quadrotor.urdf", start_pos)

    visual_shape_id = p.createVisualShape(
        shapeType=p.GEOM_SPHERE, radius=0.15, rgbaColor=[1, 0, 0, 0.6]
    )
    target_id = p.createMultiBody(
        baseMass=0,
        baseVisualShapeIndex=visual_shape_id,
        basePosition=[5.0, 5.0, 5.0],
    )

    # 5. Initialize C++ Sensors and Kalman Filter
    sensor_stddev = 0.08
    sensor = robotics_sim.PositionSensor(sensor_stddev)
    kf = robotics_sim.KalmanFilter3D(1e-4, 1e-4, sensor_stddev**2)

    cur_kp = p.readUserDebugParameter(kp_slider)
    cur_ki = p.readUserDebugParameter(ki_slider)
    cur_kd = p.readUserDebugParameter(kd_slider)
    cur_max_i = p.readUserDebugParameter(max_i_slider)
    cur_max_f = p.readUserDebugParameter(max_f_slider)

    pid = robotics_sim.PIDController(cur_kp, cur_ki, cur_kd, cur_max_i, cur_max_f)

    # 6. CSV Telemetry Setup
    csv_file = open("telemetry_data.csv", mode="w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "timestamp",
        "true_x", "true_y", "true_z",
        "noisy_x", "noisy_y", "noisy_z",
        "kf_x", "kf_y", "kf_z",
        "target_x", "target_y", "target_z",
        "force_x", "force_y", "force_z"
    ])

    last_click_count = p.readUserDebugParameter(start_btn)
    is_running = False
    dt = 1.0 / 240.0
    sim_time = 0.0
    last_drone_pos = start_pos

    print("--- Telemetry Enabled. Logging to telemetry_data.csv ---")

    try:
        while p.isConnected():
            current_click_count = p.readUserDebugParameter(start_btn)

            if current_click_count > last_click_count:
                last_click_count = current_click_count
                p.resetBasePositionAndOrientation(robot_id, start_pos, [0, 0, 0, 1])
                p.resetBaseVelocity(robot_id, [0, 0, 0], [0, 0, 0])
                p.removeAllUserDebugItems()

                kf.reset()
                pid.reset()
                sim_time = 0.0
                last_drone_pos = start_pos
                is_running = True
                print("[Telemetry] Reset sequence initiated. Logging fresh run...")

            tx = p.readUserDebugParameter(target_x_slider)
            ty = p.readUserDebugParameter(target_y_slider)
            tz = p.readUserDebugParameter(target_z_slider)
            p.resetBasePositionAndOrientation(target_id, [tx, ty, tz], [0, 0, 0, 1])
            target_pos = robotics_sim.Vector3(tx, ty, tz)

            new_kp = p.readUserDebugParameter(kp_slider)
            new_ki = p.readUserDebugParameter(ki_slider)
            new_kd = p.readUserDebugParameter(kd_slider)
            new_max_i = p.readUserDebugParameter(max_i_slider)
            new_max_f = p.readUserDebugParameter(max_f_slider)

            if new_kp != cur_kp or new_ki != cur_ki or new_kd != cur_kd:
                cur_kp, cur_ki, cur_kd = new_kp, new_ki, new_kd
                pid.set_gains(cur_kp, cur_ki, cur_kd)

            if new_max_i != cur_max_i or new_max_f != cur_max_f:
                cur_max_i, cur_max_f = new_max_i, new_max_f
                pid.set_limits(cur_max_i, cur_max_f)

            if is_running:
                pos, _ = p.getBasePositionAndOrientation(robot_id)
                true_pos = robotics_sim.Vector3(pos[0], pos[1], pos[2])

                noisy_pos = sensor.read_position(true_pos)
                filtered_pos = kf.filter(noisy_pos, dt)

                force = pid.calculate(target_pos, filtered_pos, dt)

                p.applyExternalForce(
                    objectUniqueId=robot_id,
                    linkIndex=-1,
                    forceObj=[force.x, force.y, force.z],
                    posObj=pos,
                    flags=p.WORLD_FRAME,
                )

                # Log tick data to CSV
                csv_writer.writerow([
                    f"{sim_time:.4f}",
                    true_pos.x, true_pos.y, true_pos.z,
                    noisy_pos.x, noisy_pos.y, noisy_pos.z,
                    filtered_pos.x, filtered_pos.y, filtered_pos.z,
                    target_pos.x, target_pos.y, target_pos.z,
                    force.x, force.y, force.z
                ])
                csv_file.flush()

                p.addUserDebugLine(last_drone_pos, pos, lineColorRGB=[0, 1, 0.5], lineWidth=2, lifeTime=2.0)
                last_drone_pos = pos

                p.stepSimulation()
                sim_time += dt

            time.sleep(dt)

    finally:
        csv_file.close()
        print("[Telemetry] File handle flushed and closed.")


if __name__ == "__main__":
    main()