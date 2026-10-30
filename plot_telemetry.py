import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def main():
    # 1. Load CSV Telemetry Data
    try:
        df = pd.read_csv("telemetry_data.csv")
    except FileNotFoundError:
        print("Error: telemetry_data.csv not found. Run sim_runner.py first.")
        return

    if df.empty:
        print("Error: telemetry_data.csv is empty.")
        return

    time = df["timestamp"].values

    # Extract positions
    true_pos = df[["true_x", "true_y", "true_z"]].values
    noisy_pos = df[["noisy_x", "noisy_y", "noisy_z"]].values
    kf_pos = df[["kf_x", "kf_y", "kf_z"]].values
    target_pos = df[["target_x", "target_y", "target_z"]].values
    forces = df[["force_x", "force_y", "force_z"]].values

    # 2. Compute Quantitative Metrics
    # Root Mean Square Error (RMSE) comparing True Position vs Noisy and KF
    rmse_noisy = np.sqrt(np.mean((noisy_pos - true_pos) ** 2))
    rmse_kf = np.sqrt(np.mean((kf_pos - true_pos) ** 2))
    noise_reduction_pct = ((rmse_noisy - rmse_kf) / rmse_noisy) * 100.0

    # Euclidean distance to setpoint
    error_to_target = np.linalg.norm(true_pos - target_pos, axis=1)
    initial_dist = error_to_target[0] if len(error_to_target) > 0 else 1.0

    # 2% Settling Time threshold
    settling_threshold = 0.02 * initial_dist
    within_bounds = error_to_target <= settling_threshold

    settling_time = None
    for i in range(len(within_bounds)):
        if np.all(within_bounds[i:]):
            settling_time = time[i]
            break

    print("==================================================")
    print("      DIGITAL TWIN CONTROL & FILTER METRICS       ")
    print("==================================================")
    print(f" Raw Sensor Noise RMSE     : {rmse_noisy:.4f} m")
    print(f" Kalman Filtered RMSE      : {rmse_kf:.4f} m")
    print(f" Noise Variance Reduction  : {noise_reduction_pct:.2f}%")
    if settling_time is not None:
        print(f" 2% Settling Time          : {settling_time:.2f} seconds")
    else:
        print(" 2% Settling Time          : Not settled within run window")
    print("==================================================")

    # 3. Render Engineering Figures
    plt.style.use("seaborn-v0_8-darkgrid" if "seaborn-v0_8-darkgrid" in plt.style.available else "default")
    fig, axs = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
    fig.suptitle("C++ Kalman Filter & PID Control Performance Benchmarks", fontsize=14, fontweight="bold")

    # Plot 1: Z-Axis Tracking (True vs Noisy vs Kalman Filter vs Target)
    axs[0].plot(time, df["target_z"], "r--", label="Target Z Setpoint", linewidth=1.5)
    axs[0].plot(time, df["noisy_z"], color="gray", alpha=0.4, label="Noisy Sensor Reading")
    axs[0].plot(time, df["kf_z"], "b-", label="C++ Kalman Filter Estimate", linewidth=1.8)
    axs[0].plot(time, df["true_z"], "g-", label="True Physical State", linewidth=1.8)
    axs[0].set_ylabel("Z Position (m)", fontweight="bold")
    axs[0].set_title(f"State Estimation Accuracy (Kalman RMSE: {rmse_kf:.4f}m vs Noisy: {rmse_noisy:.4f}m)")
    axs[0].legend(loc="lower right")

    # Plot 2: Euclidean Position Error
    axs[1].plot(time, error_to_target, "m-", label="Euclidean Distance Error")
    axs[1].axhline(y=settling_threshold, color="k", linestyle=":", label="2% Settling Threshold")
    if settling_time:
        axs[1].axvline(x=settling_time, color="g", linestyle="--", label=f"Settled at {settling_time:.2f}s")
    axs[1].set_ylabel("Error Vector Norm (m)", fontweight="bold")
    axs[1].set_title("Setpoint Error & Settling Dynamics")
    axs[1].legend(loc="upper right")

    # Plot 3: Actuator Control Forces
    axs[2].plot(time, df["force_x"], label="Fx (N)")
    axs[2].plot(time, df["force_y"], label="Fy (N)")
    axs[2].plot(time, df["force_z"], label="Fz (N)")
    axs[2].set_xlabel("Time (seconds)", fontweight="bold")
    axs[2].set_ylabel("Control Force (N)", fontweight="bold")
    axs[2].set_title("C++ PID Control Effort & Actuator Saturation")
    axs[2].legend(loc="upper right")

    plt.tight_layout()
    plot_filename = "telemetry_performance.png"
    plt.savefig(plot_filename, dpi=300)
    print(f"[Plotter] Performance figure saved to '{plot_filename}'")
    plt.show()


if __name__ == "__main__":
    main()
    