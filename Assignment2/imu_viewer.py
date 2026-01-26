import sys
import time
import threading
import numpy as np
import serial

import pygame
from pygame.locals import DOUBLEBUF, OPENGL, QUIT, KEYDOWN, K_ESCAPE, K_z

from OpenGL.GL import *
from OpenGL.GLU import *

# --------------------------
# Shared state
# --------------------------
latest_q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # w,x,y,z
latest_am = None  # (|a|, |mh|) optional
lock = threading.Lock()
running = True

good_frames = 0
bad_frames = 0
last_line = ""

def parse_quat_line(line: str):
    """
    Accept either:
      Q,w,x,y,z
    or:
      Q,w,x,y,z,|a|,|mh|
    """
    line = line.strip()
    if not line.startswith("Q,"):
        return None

    parts = line.split(",")
    if len(parts) < 5:
        return None

    try:
        w = float(parts[1]); x = float(parts[2]); y = float(parts[3]); z = float(parts[4])
        q = np.array([w, x, y, z], dtype=float)
        n = np.linalg.norm(q)
        if n < 1e-9:
            return None
        q = q / n

        # optional extra telemetry
        am = None
        if len(parts) >= 7:
            a_norm = float(parts[5])
            mh_norm = float(parts[6])
            am = (a_norm, mh_norm)

        return q, am
    except ValueError:
        return None

def serial_thread(port: str, baud: int):
    global latest_q, latest_am, running, good_frames, bad_frames, last_line

    try:
        ser = serial.Serial(port, baud, timeout=0.2)
        time.sleep(1.0)
    except Exception as e:
        print(f"Failed to open serial {port}: {e}")
        running = False
        return

    while running:
        try:
            line = ser.readline().decode(errors="ignore")
            if not line:
                continue
            last_line = line.strip()

            parsed = parse_quat_line(line)
            if parsed is None:
                bad_frames += 1
                continue

            q, am = parsed
            with lock:
                latest_q = q
                latest_am = am
            good_frames += 1
        except Exception:
            bad_frames += 1
            pass

    try:
        ser.close()
    except Exception:
        pass

# --------------------------
# Quaternion math / rotation
# --------------------------
def quat_to_rotmat(q):
    # q = [w,x,y,z]
    w, x, y, z = q
    R = np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=float)
    return R

def rotmat_to_glmat(R):
    M = np.eye(4, dtype=float)
    M[:3, :3] = R
    return M.T  # OpenGL column-major

def quat_mul(q1, q2):
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=float)

def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)

# --------------------------
# Drawing
# --------------------------
def draw_axes(L=1.8):
    glBegin(GL_LINES)
    glVertex3f(0, 0, 0); glVertex3f(L, 0, 0)
    glVertex3f(0, 0, 0); glVertex3f(0, L, 0)
    glVertex3f(0, 0, 0); glVertex3f(0, 0, L)
    glEnd()

def draw_cube(s=1.0):
    hs = 0.5 * s
    verts = [
        (-hs,-hs,-hs), ( hs,-hs,-hs), ( hs, hs,-hs), (-hs, hs,-hs),
        (-hs,-hs, hs), ( hs,-hs, hs), ( hs, hs, hs), (-hs, hs, hs),
    ]
    edges = [
        (0,1),(1,2),(2,3),(3,0),
        (4,5),(5,6),(6,7),(7,4),
        (0,4),(1,5),(2,6),(3,7)
    ]
    glBegin(GL_LINES)
    for a,b in edges:
        glVertex3fv(verts[a])
        glVertex3fv(verts[b])
    glEnd()

# --------------------------
# Main
# --------------------------
def main():
    if len(sys.argv) < 2:
        print("Usage: python imu_viewer.py <serial_port> [baud]")
        print("Example: python imu_viewer.py /dev/ttyUSB0 115200")
        return

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) >= 3 else 115200

    t = threading.Thread(target=serial_thread, args=(port, baud), daemon=True)
    t.start()

    pygame.init()
    pygame.display.set_caption("IMU Quaternion Viewer (TRIAD)")
    pygame.display.set_mode((900, 700), DOUBLEBUF | OPENGL)

    glEnable(GL_DEPTH_TEST)
    gluPerspective(60, (900/700), 0.05, 100.0)

    # Camera
    glTranslatef(0.0, 0.0, -5.0)

    # viewer-side zeroing
    q0_inv = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)

    clock = pygame.time.Clock()
    last_stats_t = time.time()

    global running
    while running:
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
                if event.key == K_z:
                    with lock:
                        q_now = latest_q.copy()
                    q0_inv = quat_conj(q_now)  # inverse for unit quaternion
                    print("Viewer zero set (Z).")

        with lock:
            q = latest_q.copy()
            am = latest_am

        # Apply viewer-side zeroing
        q_vis = quat_mul(q0_inv, q)
        q_vis /= np.linalg.norm(q_vis)

        # Build rotation
        R = quat_to_rotmat(q_vis)
        M = rotmat_to_glmat(R)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glPushMatrix()
        glMultMatrixf(M.astype(np.float32))
        draw_cube(1.4)
        glPopMatrix()

        draw_axes(2.0)

        pygame.display.flip()
        clock.tick(60)

        # Print stats once per second so you know data is coming in
        if time.time() - last_stats_t > 1.0:
            last_stats_t = time.time()
            if am is not None:
                print(f"OK={good_frames}  BAD={bad_frames}  |a|={am[0]:.3f}  |mh|={am[1]:.3f}  last='{last_line[:60]}'")
            else:
                print(f"OK={good_frames}  BAD={bad_frames}  last='{last_line[:60]}'")

    pygame.quit()

if __name__ == "__main__":
    main()

