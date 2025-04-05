import matplotlib.pyplot as plt
import os

# log路径
THEORETICAL_LOG = os.path.expanduser("~/ros2_ws/src/theoretical_log.txt")
ACTUAL_LOG = os.path.expanduser("~/ros2_ws/src/log.txt")
output_image_path = "trajectory_comparison_static.png"  # 保存静态图的路径


def parse_log(file_path, skip_header=True):
    xs, ys = [], []
    try:
        with open(file_path, "r") as f:
            lines = f.readlines()[1:] if skip_header else f.readlines()
            for line in lines:
                parts = line.strip().split(",")
                if len(parts) >= 2:
                    try:
                        xs.append(float(parts[0]))
                        ys.append(float(parts[1]))
                    except ValueError:
                        continue
    except FileNotFoundError:
        print(f"File not found: {file_path}")
    return xs, ys


# 解析日志文件
xs_t, ys_t = parse_log(THEORETICAL_LOG)
xs_a, ys_a = parse_log(ACTUAL_LOG)

# 初始化画布
fig, ax = plt.subplots()
ax.plot(xs_t, ys_t, 'b--', label='Theoretical Path')  # 蓝色虚线表示理论路径
ax.plot(xs_a, ys_a, 'r-', label='Actual Path')        # 红色实线表示实际路径

# 设置图形属性
ax.set_title("Theoretical vs Actual Trajectory")
ax.set_xlabel("X Position (m)")
ax.set_ylabel("Y Position (m)")
ax.grid(True)
ax.set_aspect('equal')
ax.legend()

# 保存静态图
plt.savefig(output_image_path)
print(f"Static trajectory comparison saved to {output_image_path}")

# 显示图形
plt.show()