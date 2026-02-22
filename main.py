import streamlit as st
import numpy as np
import matplotlib.pyplot as plt

# helpers (by chat)
def pose(x, y, theta):
    return np.array([x, y, theta])

def rot(th):
    return np.array([[np.cos(th), -np.sin(th)],
                     [np.sin(th),  np.cos(th)]])

def compose(a, b):
    R = rot(a[2])
    t = np.array([a[0], a[1]]) + R @ np.array([b[0], b[1]])
    return np.array([t[0], t[1], a[2] + b[2]])

def wrap_angle(th):
    return np.arctan2(np.sin(th), np.cos(th))

# drawing (by chat)
def draw_box(ax, p, w, h, color):
    x, y, th = p
    corners = np.array([
        [-w/2, -h/2],
        [ w/2, -h/2],
        [ w/2,  h/2],
        [-w/2,  h/2],
        [-w/2, -h/2],
    ])
    world = (rot(th) @ corners.T).T + np.array([x, y])
    ax.plot(world[:, 0], world[:, 1], color=color)

def draw_heading(ax, p, color, length=0.6):
    x, y, th = p
    v = rot(th) @ np.array([length, 0])
    ax.arrow(x, y, v[0], v[1], head_width=0.12, color=color)

def annotate_point(ax, label, x, y, color):
    ax.scatter(x, y, color=color)
    ax.text(x - 0.25, y + 0.25, label, color=color)

# Shooter visualization
st.title("Shooter Face-Target Visualizer")

st.subheader("Bounding Box Sizes")
col1, col2 = st.columns(2)
with col1:
    ROBOT_W = st.number_input("Robot Width", value=0.8)
    ROBOT_H = st.number_input("Robot Height", value=0.8)
with col2:
    SHOOTER_W = st.number_input("Shooter Width", value=0.3)
    SHOOTER_H = st.number_input("Shooter Height", value=0.3)

st.subheader("Robot Pose")
rx = st.number_input("Robot X", -2.0, 8.0, 0.0)
ry = st.number_input("Robot Y", -4.0, 4.0, 0.0)
rtheta = st.number_input("Robot yaw (deg)", -180, 180, 0)

st.subheader("Shooter Offset")
sx = st.number_input("Shooter X offset", -1.0, 1.0, 0.3)
sy = st.number_input("Shooter Y offset", -1.0, 1.0, 0.2)
stheta = st.number_input("Shooter yaw (deg)", -180, 180, 0)

st.subheader("Target")
tx = st.number_input("Target X", -2.0, 10.0, 5.0)
ty = st.number_input("Target Y", -5.0, 5.0, 1.0)

# Setup
fieldTrobot_old = pose(rx, ry, np.deg2rad(rtheta))
robotTshooter = pose(sx, sy, np.deg2rad(stheta))
fieldTtarget = pose(tx, ty, 0)

def shooter_pos(theta):
    return np.array([rx, ry]) + rot(theta) @ np.array([sx, sy])

theta = fieldTrobot_old[2]

# iterative solve
for _ in range(30):
    xs, ys = shooter_pos(theta)
    desired = np.arctan2(ty - ys, tx - xs) - robotTshooter[2]
    theta_next = wrap_angle(desired)

    if abs(wrap_angle(theta_next - theta)) < np.deg2rad(1e-3):
        theta = theta_next
        break
    theta = theta_next

fieldTrobot_new = pose(rx, ry, theta)
fieldTshooter_old = compose(fieldTrobot_old, robotTshooter)
fieldTshooter_new = compose(fieldTrobot_new, robotTshooter)

st.subheader("Aim Equation")

st.latex(r"""
\theta_r + \theta_s =
\operatorname{atan2}(y_t - y_s(\theta_r),\; x_t - x_s(\theta_r))
""")

st.latex(r"""
\theta_r =
\operatorname{atan2}(y_t - y_s(\theta_r),\; x_t - x_s(\theta_r)) - \theta_s
""")

fig, ax = plt.subplots()

draw_box(ax, fieldTrobot_old, ROBOT_W, ROBOT_H, 'gray')
draw_heading(ax, fieldTrobot_old, 'gray')
draw_box(ax, fieldTshooter_old, SHOOTER_W, SHOOTER_H, 'orange')
draw_heading(ax, fieldTshooter_old, 'orange')

draw_box(ax, fieldTrobot_new, ROBOT_W, ROBOT_H, 'green')
draw_heading(ax, fieldTrobot_new, 'green')
draw_box(ax, fieldTshooter_new, SHOOTER_W, SHOOTER_H, 'lime')
draw_heading(ax, fieldTshooter_new, 'lime')

xt, yt = tx, ty
xs_new, ys_new = fieldTshooter_new[0], fieldTshooter_new[1]

annotate_point(ax, r"$T(x_t,y_t)$", xt, yt, "red")
# annotate_point(ax, r"$S_{old}$", fieldTshooter_old[0], fieldTshooter_old[1], "orange")
annotate_point(ax, r"$S_{new}$", xs_new, ys_new, "lime")

# shooter -> target vector
ax.arrow(xs_new, ys_new,
         xt - xs_new,
         yt - ys_new,
         color="blue",
         head_width=0.08)

ax.text((xs_new+xt)/2,
        (ys_new+yt)/2,
        r"$\vec{v} = (x_t-x_s,\; y_t-y_s)$",
        color="blue")

ax.set_aspect('equal')
ax.set_xlim(-2, 8)
ax.set_ylim(-4, 4)
ax.grid()

st.pyplot(fig)

# debug stuff
new_vec = np.array([xt - xs_new, yt - ys_new])
new_angle = np.arctan2(new_vec[1], new_vec[0])
shooter_yaw_new = fieldTshooter_new[2]
angle_err = wrap_angle(new_angle - shooter_yaw_new)

st.write("Shooter pointing error (deg):", float(np.degrees(angle_err)))
st.write("Robot yaw solution (deg):", float(np.degrees(theta)))