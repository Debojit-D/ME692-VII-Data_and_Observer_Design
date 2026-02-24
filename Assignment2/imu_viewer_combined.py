"""
IMU Quaternion Viewer — COMBINED VISUALIZER (GyroOnly + TRIAD + Mahony) + ORBIT CAMERA

This viewer is for the **combined Arduino output** we made:
  Q, gw,gx,gy,gz,  tw,tx,ty,tz,  mw,mx,my,mz,  |a|,|mh|

It visualizes **3 boards at once** (side-by-side) using the 3 quaternions:
  - Gyro-only (G)
  - TRIAD (T)
  - Mahony (M)

Keeps the same features as your orbit viewer:
  ✅ Smooth motion (SLERP) per-estimator (removes jitter)
  ✅ Proper 3D board + local axes
  ✅ HUD overlay: FPS, OK/BAD frames, |a|, |mh|, last line
  ✅ Two zero modes:
        Z key  : viewer full zero (all estimators together)
        Y key  : yaw-only zero (keeps roll/pitch)
  ✅ Robust serial parsing + optional auto-reconnect
  ✅ Toggles: G(grid), A(axes), H(HUD)
  ✅ Orbit / Pan / Zoom camera (mouse)

Mouse controls:
  Left-drag                  : ORBIT
  Right-drag or Shift+Left   : PAN
  Mouse wheel                : ZOOM

Usage:
  python imu_viewer_combined_orbit.py /dev/ttyUSB0 115200
  python imu_viewer_combined_orbit.py COM5 115200
"""

import sys
import time
import threading
import numpy as np
import serial

import pygame
from pygame.locals import (
    DOUBLEBUF, OPENGL, QUIT, KEYDOWN,
    K_ESCAPE, K_z, K_y, K_g, K_a, K_h
)

from OpenGL.GL import *
from OpenGL.GLU import *


# ============================================================
# Shared state (serial thread -> render thread)
# ============================================================

# Each is quaternion [w,x,y,z]
latest_q_g = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # gyro-only
latest_q_t = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # triad
latest_q_m = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # mahony

latest_am = None  # (|a|, |mh|)
lock = threading.Lock()
running = True

good_frames = 0
bad_frames = 0
last_line = ""


# ============================================================
# Quaternion utilities
# ============================================================

def quat_norm(q):
    n = np.linalg.norm(q)
    if n < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    return q / n

def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]], dtype=float)

def quat_mul(q1, q2):
    w1,x1,y1,z1 = q1
    w2,x2,y2,z2 = q2
    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ], dtype=float)

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
    return M.T  # OpenGL uses column-major

def wrap_pi(a):
    return (a + np.pi) % (2*np.pi) - np.pi

def yaw_from_quat(q):
    # yaw about world Z (OpenGL world Z)
    w,x,y,z = q
    return np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

def yaw_only_quat(yaw):
    h = 0.5 * yaw
    return np.array([np.cos(h), 0.0, 0.0, np.sin(h)], dtype=float)

def slerp(q0, q1, t):
    q0 = quat_norm(q0)
    q1 = quat_norm(q1)
    dot = float(np.dot(q0, q1))

    if dot < 0.0:
        q1 = -q1
        dot = -dot

    dot = np.clip(dot, -1.0, 1.0)

    if dot > 0.9995:
        q = q0 + t * (q1 - q0)
        return quat_norm(q)

    theta_0 = np.arccos(dot)
    sin_0 = np.sin(theta_0)
    theta = theta_0 * t
    sin_t = np.sin(theta)

    s0 = np.sin(theta_0 - theta) / sin_0
    s1 = sin_t / sin_0
    return quat_norm((s0 * q0) + (s1 * q1))


# ============================================================
# Serial parsing (combined format)
# ============================================================

def parse_combined_line(line: str):
    """
    Accept combined line:
      Q, gw,gx,gy,gz,  tw,tx,ty,tz,  mw,mx,my,mz,  |a|,|mh|
    """
    line = line.strip()
    if not line.startswith("Q,"):
        return None

    parts = line.split(",")
    # Q + 12 quat values = 13, plus |a|,|mh| => 15 total
    if len(parts) < 13:
        return None

    try:
        # Gyro-only
        gw = float(parts[1]);  gx = float(parts[2]);  gy = float(parts[3]);  gz = float(parts[4])
        # TRIAD
        tw = float(parts[5]);  tx = float(parts[6]);  ty = float(parts[7]);  tz = float(parts[8])
        # Mahony
        mw = float(parts[9]);  mx = float(parts[10]); my = float(parts[11]); mz = float(parts[12])

        qg = quat_norm(np.array([gw, gx, gy, gz], dtype=float))
        qt = quat_norm(np.array([tw, tx, ty, tz], dtype=float))
        qm = quat_norm(np.array([mw, mx, my, mz], dtype=float))

        am = None
        if len(parts) >= 15:
            a_norm = float(parts[13])
            mh_norm = float(parts[14])
            am = (a_norm, mh_norm)

        return qg, qt, qm, am

    except Exception:
        return None


def serial_thread(port: str, baud: int):
    global latest_q_g, latest_q_t, latest_q_m, latest_am
    global running, good_frames, bad_frames, last_line

    ser = None
    while running:
        try:
            if ser is None:
                ser = serial.Serial(port, baud, timeout=0.2)
                time.sleep(1.0)
                print(f"[SER] Connected: {port} @ {baud}")

            line = ser.readline().decode(errors="ignore")
            if not line:
                continue

            last_line = line.strip()
            parsed = parse_combined_line(last_line)
            if parsed is None:
                bad_frames += 1
                continue

            qg, qt, qm, am = parsed
            with lock:
                latest_q_g = qg
                latest_q_t = qt
                latest_q_m = qm
                latest_am = am
            good_frames += 1

        except Exception as e:
            bad_frames += 1
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass
                ser = None
            print(f"[SER] Disconnected / error: {e}. Retrying in 1s...")
            time.sleep(1.0)

    if ser is not None:
        try:
            ser.close()
        except Exception:
            pass


# ============================================================
# Drawing helpers
# ============================================================

def draw_grid(size=7, step=0.7):
    glLineWidth(1.0)
    glBegin(GL_LINES)
    glColor3f(0.35, 0.35, 0.35)
    for i in range(-size, size+1):
        glVertex3f(i*step, -size*step, 0.0)
        glVertex3f(i*step,  size*step, 0.0)
        glVertex3f(-size*step, i*step, 0.0)
        glVertex3f( size*step, i*step, 0.0)
    glEnd()

def draw_world_axes(L=2.0):
    glLineWidth(3.0)
    glBegin(GL_LINES)
    glColor3f(1.0, 0.2, 0.2)  # X
    glVertex3f(0,0,0); glVertex3f(L,0,0)
    glColor3f(0.2, 1.0, 0.2)  # Y
    glVertex3f(0,0,0); glVertex3f(0,L,0)
    glColor3f(0.2, 0.4, 1.0)  # Z
    glVertex3f(0,0,0); glVertex3f(0,0,L)
    glEnd()
    glLineWidth(1.0)

def draw_local_axes(L=0.8):
    glLineWidth(4.0)
    glBegin(GL_LINES)
    glColor3f(1.0, 0.2, 0.2)  # X
    glVertex3f(0,0,0); glVertex3f(L,0,0)
    glColor3f(0.2, 1.0, 0.2)  # Y
    glVertex3f(0,0,0); glVertex3f(0,L,0)
    glColor3f(0.2, 0.4, 1.0)  # Z
    glVertex3f(0,0,0); glVertex3f(0,0,L)
    glEnd()
    glLineWidth(1.0)

def draw_box(L=1.1, W=2.1, H=0.18):
    hx, hy, hz = 0.5*L, 0.5*W, 0.5*H
    v = np.array([
        [-hx,-hy,-hz], [ hx,-hy,-hz], [ hx, hy,-hz], [-hx, hy,-hz],
        [-hx,-hy, hz], [ hx,-hy, hz], [ hx, hy, hz], [-hx, hy, hz],
    ], dtype=float)

    edges = [
        (0,1),(1,2),(2,3),(3,0),
        (4,5),(5,6),(6,7),(7,4),
        (0,4),(1,5),(2,6),(3,7)
    ]

    glLineWidth(2.0)
    glColor3f(1.0, 1.0, 1.0)
    glBegin(GL_LINES)
    for a,b in edges:
        glVertex3fv(v[a])
        glVertex3fv(v[b])
    glEnd()
    glLineWidth(1.0)

    draw_local_axes(0.8)

def draw_label_billboard(text, font, x, y, color=(240,240,240)):
    surf = font.render(text, True, color)
    screen = pygame.display.get_surface()
    screen.blit(surf, (x, y))

def draw_hud(screen, font, lines, color=(230,230,230)):
    y = 8
    for text in lines:
        surf = font.render(text, True, color)
        screen.blit(surf, (10, y))
        y += 18


# ============================================================
# Orbit camera
# ============================================================

class OrbitCamera:
    def __init__(self):
        self.yaw = 35.0
        self.pitch = 22.0
        self.dist = 8.0
        self.pan_x = 0.0
        self.pan_y = 0.0
        self._drag_orbit = False
        self._drag_pan = False
        self._last = (0, 0)

    def handle_event(self, event):
        if event.type == pygame.MOUSEWHEEL:
            self.dist *= (0.92 ** event.y)
            self.dist = float(np.clip(self.dist, 2.0, 60.0))

        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:
                mods = pygame.key.get_mods()
                if mods & pygame.KMOD_SHIFT:
                    self._drag_pan = True
                else:
                    self._drag_orbit = True
                self._last = pygame.mouse.get_pos()
            elif event.button == 3:
                self._drag_pan = True
                self._last = pygame.mouse.get_pos()

        if event.type == pygame.MOUSEBUTTONUP:
            if event.button == 1:
                self._drag_orbit = False
                self._drag_pan = False
            elif event.button == 3:
                self._drag_pan = False

        if event.type == pygame.MOUSEMOTION:
            x, y = event.pos
            lx, ly = self._last
            dx = x - lx
            dy = y - ly
            self._last = (x, y)

            if self._drag_orbit:
                self.yaw   += 0.35 * dx
                self.pitch += 0.35 * dy
                self.pitch = float(np.clip(self.pitch, -85.0, 85.0))

            if self._drag_pan:
                s = 0.0025 * self.dist
                self.pan_x += -s * dx
                self.pan_y +=  s * dy

    def apply(self):
        glTranslatef(self.pan_x, self.pan_y, -self.dist)
        glRotatef(self.pitch, 1.0, 0.0, 0.0)
        glRotatef(self.yaw,   0.0, 0.0, 1.0)


# ============================================================
# Main
# ============================================================

def main():
    if len(sys.argv) < 2:
        print("Usage: python imu_viewer_combined_orbit.py <serial_port> [baud]")
        print("Example: python imu_viewer_combined_orbit.py /dev/ttyUSB0 115200")
        return

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) >= 3 else 115200

    # Start serial thread
    t = threading.Thread(target=serial_thread, args=(port, baud), daemon=True)
    t.start()

    pygame.init()
    W, H = 1200, 800
    pygame.display.set_caption("IMU Combined Quaternion Viewer (Gyro + TRIAD + Mahony) — Orbit")
    screen = pygame.display.set_mode((W, H), DOUBLEBUF | OPENGL)

    font = pygame.font.SysFont("consolas", 16)

    glEnable(GL_DEPTH_TEST)
    glClearColor(0.05, 0.05, 0.07, 1.0)

    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60, (W / H), 0.05, 400.0)
    glMatrixMode(GL_MODELVIEW)

    show_grid = True
    show_axes = True
    show_hud  = True

    # Viewer zero: applies to ALL THREE
    q0_inv = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    yaw0 = 0.0
    yaw_zero_enabled = False

    # Smoothing state per estimator
    qg_render = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    qt_render = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    qm_render = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    smooth_strength = 0.18

    # Arrange three boards in world
    # (X offsets so you can compare side-by-side)
    offsets = {
        "G": np.array([-2.6, 0.0, 0.6], dtype=float),  # Gyro-only
        "T": np.array([ 0.0, 0.0, 0.6], dtype=float),  # TRIAD
        "M": np.array([ 2.6, 0.0, 0.6], dtype=float),  # Mahony
    }

    cam = OrbitCamera()
    clock = pygame.time.Clock()
    fps = 0.0

    global running
    while running:
        dt = clock.tick(60) / 1000.0
        fps = 0.9*fps + 0.1*(1.0/max(dt, 1e-6))

        for event in pygame.event.get():
            if event.type == QUIT:
                running = False

            cam.handle_event(event)

            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False

                if event.key == K_z:
                    # Full zero uses MAHONY (most stable global reference)
                    with lock:
                        q_ref = latest_q_m.copy()
                    q0_inv = quat_conj(q_ref)
                    yaw_zero_enabled = False
                    print("[VIEW] Full zero set (Z).")

                if event.key == K_y:
                    # Yaw-only zero based on current *already full-zeroed* mahony
                    with lock:
                        q_ref = latest_q_m.copy()
                    q_tmp = quat_mul(q0_inv, q_ref)
                    q_tmp = quat_norm(q_tmp)
                    yaw0 = yaw_from_quat(q_tmp)
                    yaw_zero_enabled = True
                    print("[VIEW] Yaw zero set (Y).")

                if event.key == K_g:
                    show_grid = not show_grid
                if event.key == K_a:
                    show_axes = not show_axes
                if event.key == K_h:
                    show_hud = not show_hud

        with lock:
            qg_in = latest_q_g.copy()
            qt_in = latest_q_t.copy()
            qm_in = latest_q_m.copy()
            am = latest_am

        def apply_view_zero(q_in):
            q_vis = quat_mul(q0_inv, q_in)
            q_vis = quat_norm(q_vis)
            if yaw_zero_enabled:
                yaw_now = yaw_from_quat(q_vis)
                dyaw = wrap_pi(yaw_now - yaw0)
                q_remove = yaw_only_quat(-dyaw)
                q_vis = quat_mul(q_remove, q_vis)
                q_vis = quat_norm(q_vis)
            return q_vis

        qg_vis = apply_view_zero(qg_in)
        qt_vis = apply_view_zero(qt_in)
        qm_vis = apply_view_zero(qm_in)

        # Smooth per estimator
        t_s = float(np.clip(smooth_strength, 0.01, 1.0))
        qg_render = slerp(qg_render, qg_vis, t_s)
        qt_render = slerp(qt_render, qt_vis, t_s)
        qm_render = slerp(qm_render, qm_vis, t_s)

        Mg = rotmat_to_glmat(quat_to_rotmat(qg_render))
        Mt = rotmat_to_glmat(quat_to_rotmat(qt_render))
        Mm = rotmat_to_glmat(quat_to_rotmat(qm_render))

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        glLoadIdentity()
        cam.apply()

        if show_grid:
            draw_grid(size=9, step=0.7)
        if show_axes:
            draw_world_axes(2.0)

        # Draw 3 boards
        def draw_board(M, off):
            glPushMatrix()
            glTranslatef(float(off[0]), float(off[1]), float(off[2]))
            glMultMatrixf(M.astype(np.float32))
            draw_box(L=1.1, W=2.1, H=0.18)
            glPopMatrix()

        draw_board(Mg, offsets["G"])
        draw_board(Mt, offsets["T"])
        draw_board(Mm, offsets["M"])

        # HUD + labels
        surf = pygame.display.get_surface()
        if show_hud:
            hud_lines = [
                f"FPS: {fps:5.1f}",
                f"OK frames: {good_frames}   BAD frames: {bad_frames}",
            ]
            if am is not None:
                hud_lines.append(f"|a|: {am[0]:.3f}   |mh|: {am[1]:.3f}")
            hud_lines.append("Mouse: L-drag orbit | Shift+L or R-drag pan | Wheel zoom")
            hud_lines.append("Keys: Z(full zero)  Y(yaw zero)  G(grid)  A(axes)  H(hud)  ESC(quit)")
            hud_lines.append(f"Last: {last_line[:110]}")
            draw_hud(surf, font, hud_lines)

            # On-screen estimator labels (fixed screen positions)
            draw_label_billboard("Gyro-only (G)", font, 120, H - 40, color=(255,200,200))
            draw_label_billboard("TRIAD (T)",     font, W//2 - 55, H - 40, color=(200,255,200))
            draw_label_billboard("Mahony (M)",    font, W - 220, H - 40, color=(200,220,255))

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
