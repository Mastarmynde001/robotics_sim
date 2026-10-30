import os
import subprocess
import sys


def run_pipeline():
    print("==================================================")
    print("   LAUNCHING DIGITAL TWIN SIMULATION & PIPELINE   ")
    print("==================================================")

    # 1. Launch PyBullet
    print("\n[1/2] Launching PyBullet Simulation (sim_runner.py)...")
    print(" -> Interact with GUI or click 'Start / Reset Simulation'. Close window to exit.")
    
    sim_result = subprocess.run([sys.executable, "sim_runner.py"])

    # Ignore PyBullet/X11 window termination crashes on Ubuntu
    if sim_result.returncode != 0:
        print(f"\n[Note] PyBullet window closed (Exit Code {sim_result.returncode}). Proceeding to analysis...")

    # 2. Verify Telemetry CSV Output Existence
    telemetry_file = "telemetry_data.csv"
    if not os.path.exists(telemetry_file) or os.path.getsize(telemetry_file) == 0:
        print(f"\n[Error] '{telemetry_file}' was not found or is empty. Did you click 'Start' in the GUI?")
        sys.exit(1)

    # 3. Launch Telemetry Analysis & Plotting Script
    print("\n[2/2] Running Telemetry Analysis & Generating Graphs (plot_telemetry.py)...")
    plot_result = subprocess.run([sys.executable, "plot_telemetry.py"])

    if plot_result.returncode == 0:
        print("\n[Success] Pipeline execution complete. Telemetry graphics saved to disk.")
    else:
        print(f"\n[Error] Telemetry plotting failed with exit code: {plot_result.returncode}")


if __name__ == "__main__":
    run_pipeline()