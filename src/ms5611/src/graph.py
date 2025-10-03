import serial
import csv
import time
import os

# --------- CONFIG ---------
PORT = "/dev/ttyUSB0"   # แก้ตามพอร์ต Arduino/ESP32
BAUD = 115200
# --------------------------

def main():
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)  # รอให้อุปกรณ์ reset เสร็จ
    print(f"Connected to {PORT} at {BAUD} baud")

    try:
        while True:
            # รับ input ชั้นจากผู้ใช้
            floor = input("กรอกชั้น (1-9) หรือ q ออก: ")
            if floor.lower() == "q":
                break
            if floor not in [str(i) for i in range(1, 10)]:
                print("กรุณากรอก 1-9 เท่านั้น")
                continue

            # ตั้งชื่อไฟล์ CSV ตามชั้น
            outfile = f"floor{floor}.csv"
            new_file = not os.path.exists(outfile)

            with open(outfile, "a", newline="") as f:
                writer = csv.writer(f)
                if new_file:
                    writer.writerow(["floor", "time(ms)", "pressure(Pa)", "temperature(C)"])  # header

                # ส่งไป Arduino
                ser.write(floor.encode())
                print(f"เริ่มบันทึกชั้น {floor} ...")

                start = time.time()
                while time.time() - start < 32:   # เผื่อ buffer 2 วิ
                    line = ser.readline().decode(errors="ignore").strip()
                    if not line:
                        continue
                    if line.startswith("#"):
                        print(line)  # log/debug
                        continue

                    parts = line.split(",")
                    if len(parts) == 4:  # floor,time,pressure,temperature
                        writer.writerow(parts)
                        print(parts)

            print(f"✔ บันทึกลง {outfile} เรียบร้อยแล้ว")

    except KeyboardInterrupt:
        print("หยุดโปรแกรมด้วย Ctrl+C")
    finally:
        ser.close()
        print("Serial ปิดแล้ว")

if __name__ == "__main__":
    main()
