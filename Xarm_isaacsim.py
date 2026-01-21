from xarm import Controller
import socket
import math
import time

def connect_socket():
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.connect(('localhost', 12345))
    return sock

arm = Controller('USB')

prev_angles = [0.0] * 6
sock = connect_socket()

try:
    while True:
        curr_angles = []
        for i in range(1, 7):
            try:
                pos = arm.getPosition(i, False)
                angle_deg = (pos - 500) * 0.25
                curr_angles.append(angle_deg)
            except Exception:
                curr_angles.append(prev_angles[i-1])
        
        changed = any(abs(p - c) > 0.3 for p, c in zip(prev_angles, curr_angles))
        
        if changed:
            ur_angles = curr_angles[::-1]
            rad_angles = [math.radians(a) for a in ur_angles]
            data = ','.join(f"{a:.6f}" for a in rad_angles)
            while True:
                try:
                    sock.sendall(data.encode('utf-8'))
                    print(f"Sent: {rad_angles}")
                    break
                except (ConnectionAbortedError, ConnectionResetError, OSError):
                    print("Reconnecting...")
                    sock.close()
                    sock = connect_socket()
                    time.sleep(0.01)
        
        prev_angles = curr_angles[:]
        time.sleep(0.03)  # 33Hz, less aggressive

except KeyboardInterrupt:
    sock.close()
    print("Stopped.")
