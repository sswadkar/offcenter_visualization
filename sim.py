import numpy as np

def rot(th):
    return np.array([[np.cos(th), -np.sin(th)],
                     [np.sin(th),  np.cos(th)]])

def wrap_angle(th):
    return np.arctan2(np.sin(th), np.cos(th))

def solve_yaw(rx, ry, rtheta, sx, sy, tx, ty,
              max_iter=50, tol_deg=0.01):

    # match your iterative initial guess
    theta = rtheta
    tol = np.deg2rad(tol_deg)

    for i in range(max_iter):

        c = np.cos(theta)
        s = np.sin(theta)

        # shooter position
        xs = rx + sx*c - sy*s
        ys = ry + sx*s + sy*c

        dx = tx - xs
        dy = ty - ys

        # alpha = field angle from shooter to target
        alpha = np.arctan2(dy, dx)

        # residual (same as iterative logic)
        f = wrap_angle(theta - (alpha))

        # shooter position derivatives wrt theta
        dxs_dtheta = -sx*s - sy*c
        dys_dtheta =  sx*c - sy*s

        # dx, dy derivatives
        dx_d = -dxs_dtheta
        dy_d = -dys_dtheta

        denom = dx*dx + dy*dy
        if denom < 1e-9:
            return max_iter, False

        # derivative of atan2(dy, dx)
        d_alpha = (dx*dy_d - dy*dx_d) / denom

        # derivative of f
        fprime = 1 - d_alpha

        if abs(fprime) < 1e-6:
            return max_iter, False

        theta_next = theta - f / fprime
        theta_next = wrap_angle(theta_next)

        if abs(wrap_angle(theta_next - theta)) < tol:
            return i+1, True

        theta = theta_next

    return max_iter, False

def run_sim(N=10000, max_iter=50, tol_deg=0.01):

    iterations = []
    failures = 0
    worst_case = None
    worst_iters = 0

    for _ in range(N):

        rx = np.random.uniform(-2, 8)
        ry = np.random.uniform(-4, 4)
        rtheta = np.random.uniform(-np.pi, np.pi)

        sx = np.random.uniform(-0.3, 0.2)
        sy = np.random.uniform(-0.3, 0.3)
        # ts = np.random.uniform(-np.pi, np.pi)

        ROBOT_W = 0.8
        ROBOT_H = 0.8

        while True:
            tx = np.random.uniform(-2, 10)
            ty = np.random.uniform(-5, 5)

            # transform target into robot frame
            dx = tx - rx
            dy = ty - ry

            c = np.cos(-rtheta)
            s = np.sin(-rtheta)

            # rotate into robot local frame
            x_local = dx * c - dy * s
            y_local = dx * s + dy * c

            if not (-ROBOT_W / 2 <= x_local <= ROBOT_W / 2 and
                    -ROBOT_H / 2 <= y_local <= ROBOT_H / 2):
                break

        steps, converged = solve_yaw(
            rx, ry, rtheta,
            sx, sy,
            tx, ty,
            max_iter=max_iter,
            tol_deg=tol_deg
        )

        if converged:
            iterations.append(steps)
            if steps > worst_iters:
                worst_iters = steps
                worst_case = (rx, ry, rtheta, sx, sy, tx, ty)
        else:
            failures += 1

    print("\n=== RESULTS ===")
    print(f"Trials: {N}")
    print(f"Failures: {failures}")

    if iterations:
        print(f"Avg Iterations: {np.mean(iterations):.2f}")
        print(f"Max Iterations: {np.max(iterations)}")
        print(f"Min Iterations: {np.min(iterations)}")

    if worst_case:
        print("\nWorst Converging Case:")
        print("rx, ry, rtheta, sx, sy, tx, ty =")
        print(worst_case)

if __name__ == "__main__":
    run_sim(N=20000, max_iter=50, tol_deg=0.01)