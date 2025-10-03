import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_compare():
    pressure_data = {}
    temp_data = {}

    # loop หาไฟล์ floor1.csv - floor10.csv
    for i in range(1, 11):
        filename = f"floor{i}.csv"
        if not os.path.exists(filename):
            continue

        # อ่านไฟล์ CSV
        df = pd.read_csv(filename)
        if df.empty:
            print(f"{filename} ไม่มีข้อมูล")
            continue

        # เก็บข้อมูลเพื่อเอาไป plot รวม
        pressure_data[f"Floor {i}"] = (df["time(ms)"], df["pressure(Pa)"])
        temp_data[f"Floor {i}"] = (df["time(ms)"], df["temperature(C)"])

    # ----------------- Plot Pressure -----------------
    plt.figure(figsize=(10, 5))
    for floor, (time, pressure) in pressure_data.items():
        plt.plot(time, pressure, label=floor)
    plt.title("Pressure Comparison by Floor")
    plt.xlabel("Time (ms)")
    plt.ylabel("Pressure (Pa)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # ----------------- Plot Temperature -----------------
    plt.figure(figsize=(10, 5))
    for floor, (time, temp) in temp_data.items():
        plt.plot(time, temp, label=floor)
    plt.title("Temperature Comparison by Floor")
    plt.xlabel("Time (ms)")
    plt.ylabel("Temperature (°C)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    plot_compare()
