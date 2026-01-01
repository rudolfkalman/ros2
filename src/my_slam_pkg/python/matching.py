import numpy as np
import matplotlib.pyplot as plt

Points = [np.array([-0.990066, -0.502765]), np.array([-0.946230, -0.449131]), np.array([-0.923050, -0.463866]),
    np.array([-0.845967, -0.423427]), np.array([-0.846124, -0.407516]), np.array([-0.794727, -0.360431]),
    np.array([-0.783218, -0.348086]), np.array([-0.727840, -0.319847]), np.array([-0.723345, -0.327545]),
    np.array([-0.664758, -0.271922]), np.array([-0.652772, -0.269313]), np.array([-0.611586, -0.217798]),
    np.array([-0.580298, -0.223027]), np.array([-0.555132, -0.169916]), np.array([-0.507548, -0.171740]),
    np.array([-0.484088, -0.152736]), np.array([-0.450418, -0.120520]), np.array([-0.404360, -0.106692]),
    np.array([-0.374848, -0.072905]), np.array([-0.339986, -0.065370]), np.array([-0.308692, -0.033436]),
    np.array([-0.261322, -0.013986]), np.array([-0.232925, 0.018149]), np.array([-0.197544, 0.033949]),
    np.array([-0.167206, 0.061422]), np.array([-0.127771, 0.087433]), np.array([-0.100848, 0.112879]),
    np.array([-0.058377, 0.126223]), np.array([-0.034330, 0.152678]), np.array([0.009517, 0.172289]),
    np.array([0.037507, 0.194863]), np.array([0.074056, 0.215773]), np.array([0.102762, 0.243694]),
    np.array([0.143857, 0.256642]), np.array([0.179043, 0.279898]), np.array([0.207044, 0.296611]),
    np.array([0.250845, 0.326826]), np.array([0.285383, 0.339882]), np.array([0.313045, 0.368843]),
    np.array([0.353731, 0.377079]), np.array([0.387503, 0.406056]), np.array([0.414120, 0.421939]),
    np.array([0.454254, 0.454354]), np.array([0.489371, 0.464529]), np.array([0.520586, 0.487899]),
    np.array([0.564335, 0.511315]), np.array([0.593220, 0.527446]), np.array([0.627013, 0.556547]),
    np.array([0.665273, 0.565593]), np.array([0.694222, 0.593949]), np.array([0.729257, 0.617268]),
    np.array([0.827804, 0.849781]), np.array([0.881833, 0.799575]), np.array([0.921831, 0.824033]),
    np.array([0.941507, 0.811238]), np.array([1.001164, 0.777141])]

Rotated_Points = []

def Calc_Calc_Affine(Vec2, Theta, Displacement):
    Calc_Calc_Affine = np.array([[np.cos(Theta), -np.sin(Theta), Displacement[0]],
                       [np.sin(Theta),  np.cos(Theta), Displacement[1]],
                       [0,              0,             1]])
    Vec3 = np.array([Vec2[0], Vec2[1], 1])
    Rotated_Vec3 = Calc_Calc_Affine @ Vec3
    return np.array([Rotated_Vec3[0], Rotated_Vec3[1]])

def MakeRotate_Mat(theta):
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta),  np.cos(theta)]])

def calc_J(p, x, y):
    # y(p) = R(theta) * [x, y] + [tx, ty] Jacobian of y(p)
    theta = p[2]
    J = np.array([
        [1, 0, -x*np.sin(theta) - y*np.cos(theta)],
        [0, 1,  x*np.cos(theta) - y*np.sin(theta)]
    ])
    return J

def calc_grad_y(Rp, Sigma_inv):
    # ∇y f   grad vector
    return - Sigma_inv @ Rp

def calc_Hf_y(Sigma_inv):
    # Hf     hessian matrix
    return -Sigma_inv

def matching_manual(all_scan, current_scan):
    box_size = 0.1
    stack = np.stack(all_scan)
    min_x, min_y = np.min(stack, axis=0)
    max_x, max_y = np.max(stack, axis=0)

    width = int((max_x - min_x)/box_size)+1
    height = int((max_y - min_y)/box_size)+1

    grid = [[[] for _ in range(height)] for _ in range(width)]
    average_and_covariance = [[None for _ in range(height)] for _ in range(width)]

    # put the scan points into a grid
    for pt in all_scan:
        gx = int((pt[0]-min_x)/box_size)
        gy = int((pt[1]-min_y)/box_size)
        grid[gx][gy].append(pt)

    # calc average and covariance matrix
    for i in range(width):
        for j in range(height):
            if len(grid[i][j]) < 3:
                continue
            pts = np.stack(grid[i][j])
            mean = np.mean(pts, axis=0)
            cov = np.cov(pts, rowvar=False)
            average_and_covariance[i][j] = (mean, cov)

    # init
    p = np.zeros(3)
    while True:
        grad_total = np.zeros(3)
        H_total = np.zeros((3,3))
        # convert the score to the grad vector and hessian matrix
        for i in range(len(current_scan)):
            x, y = current_scan[i]
            # calc the pos of the grid
            gx = int((all_scan[i][0]-min_x)/box_size)
            gy = int((all_scan[i][1]-min_y)/box_size)
            cell = average_and_covariance[gx][gy]
            if cell is None:
                continue
            mean, cov = cell
            Sigma_inv = np.linalg.inv(cov)
            R = MakeRotate_Mat(p[2])
            Rp = R @ np.array([x,y]) + (np.array([p[0], p[1]]) - mean)
            # grad vector and hessian matrix
            grad_y = calc_grad_y(Rp, Sigma_inv)
            Hf = calc_Hf_y(Sigma_inv)
            # Jacobian
            J = calc_J(p, x, y)
            # calc grad_total and hessian_total
            grad_total += J.T @ grad_y
            H_total += J.T @ Hf @ J

        delta_p = -np.linalg.inv(H_total) @ grad_total
        p_prev = p.copy()
        p += delta_p
        # check the convergence of delta_p
        if np.linalg.norm(p - p_prev) < 1e-3:
            break
    return p

# rotate points
for i in range(len(Points)):
    Rotated_Points.append(Calc_Calc_Affine(Points[i], np.pi/3, np.array([1,1])))

delta_p = matching_manual(Points, Rotated_Points)

Rotated_Points_by_delta_p = []
for i in range(len(Points)):
    # inverse transform
    Rotated_Points_by_delta_p.append(Calc_Calc_Affine(Rotated_Points[i], delta_p[2], np.array([delta_p[0], delta_p[1]])))

# plot
plt_points = np.stack(Points)
plt_rotated_points = np.stack(Rotated_Points)
plt_rotated_points_by_delta_p = np.stack(Rotated_Points_by_delta_p)

plt.figure(figsize=(8, 8))
plt.scatter(plt_points[:,0], plt_points[:,1], c='blue', marker='o', label='Original Points')
plt.scatter(plt_rotated_points[:,0], plt_rotated_points[:,1], c='red', marker='x', label='Rotated_Points')
plt.scatter(plt_rotated_points_by_delta_p[:,0], plt_rotated_points_by_delta_p[:,1], c='green', marker='^', label='Recovered Points')

plt.axis('equal')
plt.legend()
plt.title('Point Cloud Matching Visualization')
plt.xlabel('X')
plt.ylabel('Y')
plt.grid(True)
print("delta_p:", delta_p)
plt.savefig("result.png")
plt.show()
