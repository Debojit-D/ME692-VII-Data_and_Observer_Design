"""
IMU Quaternion Viewer (TRIAD / Mahony / etc.) — Better Visualization + ORBIT CAMERA

Keeps ALL your existing functionality:
  ✅ Smooth motion (SLERP-based) to remove jitter
  ✅ Draws a proper 3D “board” (box) + colored axes (RGB)
  ✅ Shows HUD overlay: FPS, OK/BAD frames, |a|, |mh|, last line
  ✅ Two zero modes:
        Z key  : viewer zero (full orientation)
        Y key  : yaw-only zero (keep roll/pitch)
  ✅ Auto-reconnect option (simple) + robust line parsing
  ✅ Optional “world frame” grid for better depth perception
  ✅ Toggles: G(grid), A(axes), H(HUD)

ADDED (without removing anything):
  ✅ Orbit / Pan / Zoom camera (natural perspective control)

Mouse controls (Orbit Camera):
  Left-drag            : ORBIT (rotate view around object)
  Right-drag or Shift+Left-drag : PAN
  Mouse wheel          : ZOOM
  Middle-drag (if you have) can be used like Left-drag (optional via OS)

Usage:
  python imu_viewer_better_orbit.py /dev/ttyUSB0 115200
  python imu_viewer_better_orbit.py COM5 115200

Keys:
  ESC  : quit
  Z    : set viewer zero (full quaternion)
  Y    : yaw-zero only (keeps roll/pitch, sets heading to zero)
  G    : toggle ground grid
  A    : toggle axes
  H    : toggle HUD
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

# --------------------------
# Shared state (serial thread -> render thread)
# --------------------------
latest_q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)  # w,x,y,z
latest_am = None  # (|a|, |mh|)
lock = threading.Lock()
running = True

good_frames = 0
bad_frames = 0
last_line = ""

# --------------------------
# Quaternion utilities
# --------------------------
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
    # Yaw about world Z:
    w,x,y,z = q
    return np.arctan2(2*(w*z + x*y), 1 - 2*(y*y + z*z))

def yaw_only_quat(yaw):
    # rotation about Z
    h = 0.5 * yaw
    return np.array([np.cos(h), 0.0, 0.0, np.sin(h)], dtype=float)

def slerp(q0, q1, t):
    """
    Spherical linear interpolation between unit quaternions.
    Handles hemisphere (shortest path).
    """
    q0 = quat_norm(q0)
    q1 = quat_norm(q1)
    dot = float(np.dot(q0, q1))

    # Hemisphere fix: if dot < 0, negate q1 for shortest path
    if dot < 0.0:
        q1 = -q1
        dot = -dot

    dot = np.clip(dot, -1.0, 1.0)

    # If very close, use lerp to avoid numerical issues
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

# --------------------------
# Serial parsing
# --------------------------
def parse_quat_line(line: str):
    """
    Accept:
      Q,w,x,y,z
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
        q = quat_norm(np.array([w, x, y, z], dtype=float))

        am = None
        if len(parts) >= 7:
            a_norm = float(parts[5])
            mh_norm = float(parts[6])
            am = (a_norm, mh_norm)
        return q, am
    except Exception:
        return None

def serial_thread(port: str, baud: int):
    global latest_q, latest_am, running, good_frames, bad_frames, last_line

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
            parsed = parse_quat_line(last_line)
            if parsed is None:
                bad_frames += 1
                continue

            q, am = parsed
            with lock:
                latest_q = q
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

# --------------------------
# Drawing helpers
# --------------------------
def draw_grid(size=5, step=1.0):
    """
    Draw a ground grid on world XY plane at Z=0.
    """
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
    # X red
    glColor3f(1.0, 0.2, 0.2)
    glVertex3f(0,0,0); glVertex3f(L,0,0)
    # Y green
    glColor3f(0.2, 1.0, 0.2)
    glVertex3f(0,0,0); glVertex3f(0,L,0)
    # Z blue
    glColor3f(0.2, 0.4, 1.0)
    glVertex3f(0,0,0); glVertex3f(0,0,L)
    glEnd()
    glLineWidth(1.0)

def draw_box(L=1.8, W=1.0, H=0.15):
    """
    Draw a simple rectangular board-like box with edges.
    Centered at origin, local axes.
      L along +X, W along +Y, H along +Z.
    """
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

    # Local axes on the board
    draw_local_axes(1.0)

def draw_local_axes(L=1.0):
    glLineWidth(4.0)
    glBegin(GL_LINES)
    # local X red
    glColor3f(1.0, 0.2, 0.2)
    glVertex3f(0,0,0); glVertex3f(L,0,0)
    # local Y green
    glColor3f(0.2, 1.0, 0.2)
    glVertex3f(0,0,0); glVertex3f(0,L,0)
    # local Z blue
    glColor3f(0.2, 0.4, 1.0)
    glVertex3f(0,0,0); glVertex3f(0,0,L)
    glEnd()
    glLineWidth(1.0)

def draw_hud(screen, font, lines, color=(230,230,230)):
    """
    Simple HUD using pygame font overlay.
    """
    y = 8
    for text in lines:
        surf = font.render(text, True, color)
        screen.blit(surf, (10, y))
        y += 18

# --------------------------
# Orbit camera (ADDED)
# --------------------------
class OrbitCamera:
    """
    Natural camera that lets you orbit/pan/zoom without removing any other features.
    World frame stays fixed; object rotates by quaternion.
    """
    def __init__(self):
        self.yaw = 35.0     # degrees
        self.pitch = 22.0   # degrees
        self.dist = 6.0
        self.pan_x = 0.0
        self.pan_y = 0.0

        self._drag_orbit = False
        self._drag_pan = False
        self._last = (0, 0)

    def handle_event(self, event):
        # Zoom
        if event.type == pygame.MOUSEWHEEL:
            self.dist *= (0.92 ** event.y)
            self.dist = float(np.clip(self.dist, 1.8, 40.0))

        if event.type == pygame.MOUSEBUTTONDOWN:
            if event.button == 1:  # left
                mods = pygame.key.get_mods()
                if mods & pygame.KMOD_SHIFT:
                    self._drag_pan = True
                else:
                    self._drag_orbit = True
                self._last = pygame.mouse.get_pos()
            elif event.button == 3:  # right
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
        # Camera transform
        glTranslatef(self.pan_x, self.pan_y, -self.dist)
        glRotatef(self.pitch, 1.0, 0.0, 0.0)
        glRotatef(self.yaw,   0.0, 0.0, 1.0)

# --------------------------
# Main
# --------------------------
def main():
    if len(sys.argv) < 2:
        print("Usage: python imu_viewer_better_orbit.py <serial_port> [baud]")
        print("Example: python imu_viewer_better_orbit.py /dev/ttyUSB0 115200")
        return

    port = sys.argv[1]
    baud = int(sys.argv[2]) if len(sys.argv) >= 3 else 115200

    # Start serial thread
    t = threading.Thread(target=serial_thread, args=(port, baud), daemon=True)
    t.start()

    pygame.init()
    W, H = 1000, 760
    pygame.display.set_caption("IMU Quaternion Viewer (Better + Orbit)")
    screen = pygame.display.set_mode((W, H), DOUBLEBUF | OPENGL)

    # HUD font (uses pygame, not OpenGL text)
    font = pygame.font.SysFont("consolas", 16)

    glEnable(GL_DEPTH_TEST)
    glClearColor(0.05, 0.05, 0.07, 1.0)

    # Perspective (unchanged)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    gluPerspective(60, (W / H), 0.05, 200.0)

    # We'll manage camera each frame via OrbitCamera
    glMatrixMode(GL_MODELVIEW)

    # Viewer controls
    show_grid = True
    show_axes = True
    show_hud  = True

    # Viewer-side zero:
    # - full zero: q0_inv
    # - yaw zero:  yaw0
    q0_inv = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    yaw0 = 0.0
    yaw_zero_enabled = False

    # Smoothing state
    q_render = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)
    smooth_strength = 0.18  # 0..1 (higher = snappier, lower = smoother)

    # Orbit camera
    cam = OrbitCamera()

    clock = pygame.time.Clock()
    last_stats_t = time.time()
    fps = 0.0

    global running
    while running:
        dt = clock.tick(60) / 1000.0
        fps = 0.9*fps + 0.1*(1.0/max(dt, 1e-6))

        for event in pygame.event.get():
            if event.type == QUIT:
                running = False

            # Orbit camera events (ADDED)
            cam.handle_event(event)

            # Key handling (unchanged)
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False

                # Full orientation zero
                if event.key == K_z:
                    with lock:
                        q_now = latest_q.copy()
                    q0_inv = quat_conj(q_now)  # inverse for unit quaternion
                    yaw_zero_enabled = False
                    print("[VIEW] Full zero set (Z).")

                # Yaw-only zero (NOTE: should use q_vis yaw, not raw q_now)
                if event.key == K_y:
                    with lock:
                        q_now = latest_q.copy()
                    q_tmp = quat_mul(q0_inv, q_now)
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
            q_in = latest_q.copy()
            am = latest_am

        # Apply zeroing
        q_vis = quat_mul(q0_inv, q_in)
        q_vis = quat_norm(q_vis)

        if yaw_zero_enabled:
            yaw_now = yaw_from_quat(q_vis)
            dyaw = wrap_pi(yaw_now - yaw0)
            q_remove = yaw_only_quat(-dyaw)
            q_vis = quat_mul(q_remove, q_vis)
            q_vis = quat_norm(q_vis)

        # Smooth motion: SLERP current render -> new q_vis
        t_s = np.clip(smooth_strength, 0.01, 1.0)
        q_render = slerp(q_render, q_vis, t_s)

        # Build rotation matrix and GL matrix
        R = quat_to_rotmat(q_render)
        M = rotmat_to_glmat(R)

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)

        # Camera (ADDED): replaces the fixed translate/rotate
        glLoadIdentity()
        cam.apply()

        # World helpers
        if show_grid:
            draw_grid(size=7, step=0.7)
        if show_axes:
            draw_world_axes(2.0)

        # Draw the board
        glPushMatrix()
        glMultMatrixf(M.astype(np.float32))
        draw_box(L=2.1, W=1.1, H=0.18)
        glPopMatrix()

        # HUD overlay (pygame blit)
        if show_hud:
            hud_lines = [
                f"FPS: {fps:5.1f}",
                f"OK frames: {good_frames}   BAD frames: {bad_frames}",
            ]
            if am is not None:
                hud_lines.append(f"|a|: {am[0]:.3f}   |mh|: {am[1]:.3f}")
            hud_lines.append("Mouse: L-drag orbit | Shift+L or R-drag pan | Wheel zoom")
            hud_lines.append("Keys: Z(full zero)  Y(yaw zero)  G(grid)  A(axes)  H(hud)  ESC(quit)")
            hud_lines.append(f"Last: {last_line[:80]}")

            surf = pygame.display.get_surface()
            draw_hud(surf, font, hud_lines)

        pygame.display.flip()

        # Print stats once per second (console)
        if time.time() - last_stats_t > 1.0:
            last_stats_t = time.time()
            if am is not None:
                print(f"OK={good_frames}  BAD={bad_frames}  |a|={am[0]:.3f}  |mh|={am[1]:.3f}  last='{last_line[:60]}'")
            else:
                print(f"OK={good_frames}  BAD={bad_frames}  last='{last_line[:60]}'")

    pygame.quit()

if __name__ == "__main__":
    main()

