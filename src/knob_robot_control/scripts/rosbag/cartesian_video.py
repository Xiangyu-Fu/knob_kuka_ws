import rosbag
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from tqdm import tqdm

# 打开 rosbag 文件并读取 /fri_cartesian_pose topic 数据
bag = rosbag.Bag('fortiss_knob.baglg_2024-09-24-17-41-01.bag')
positions = []
timestamps = []

for topic, msg, t in bag.read_messages(topics=['/fri_cartesian_pose']):
    x = msg.pose.position.x
    y = msg.pose.position.y
    z = msg.pose.position.z
    positions.append((x, y, z))
    timestamps.append(t.to_sec())  # 提取rosbag时间戳为秒数
    
bag.close()

# 每1000个数据取一个
sampled_positions = positions[::10]
sampled_timestamps = timestamps[::10]

# 提取 x, y, z 的位置
x_data, y_data, z_data = zip(*sampled_positions)

# 创建一个子图布局显示所有数据
fig, ax = plt.subplots(figsize=(8, 6))  # 创建单个子图
ax.set_title('X, Y, Z Position')

# 初始化用于存储时间戳文本对象的变量
timestamp_text = None

# 设置时间窗口
window_size = 500  # 窗口大小为50个数据点

# 初始化函数
def init():
    ax.plot([], [], 'b-', label="X")  # 初始化 x 线条
    ax.plot([], [], 'g-', label="Y")  # 初始化 y 线条
    ax.plot([], [], 'r-', label="Z")  # 初始化 z 线条
    ax.legend(loc='upper right')  # 添加图例
    return ax

# 更新函数：每次更新每个 subplot 的数据
def update(frame):
    global timestamp_text

    start = max(0, frame - window_size)  # 滑动窗口的起点
    end = frame + 1  # 当前帧的终点

    # 清除之前的内容，但保留标题和图例
    ax.clear()
    ax.set_title('X, Y, Z Position')
    ax.set_xlim(sampled_timestamps[start], sampled_timestamps[end-1])  # 设置 x 轴为时间戳
    ax.set_ylim(min(min(x_data), min(y_data), min(z_data)), max(max(x_data), max(y_data), max(z_data)))

    # 绘制 x, y, z 数据，使用时间戳作为 x 轴
    ax.plot(sampled_timestamps[start:end], x_data[start:end], 'b-', label="X")
    ax.plot(sampled_timestamps[start:end], y_data[start:end], 'g-', label="Y")
    ax.plot(sampled_timestamps[start:end], z_data[start:end], 'r-', label="Z")

    # 添加图例
    ax.legend(loc='upper right')

    # 清除之前的时间戳文本
    if timestamp_text is not None:
        timestamp_text.remove()

    # 显示新的时间戳文本
    timestamp_text = fig.text(0.5, 0.03, f"ROS Timestamp: {sampled_timestamps[frame]:.2f}", ha='center', fontsize=12)

    return ax

# 创建动画并包装帧生成器
frames = range(len(x_data))
frames_with_progress = tqdm(frames, desc="Generating animation frames")

# 创建动画并显示进度
ani = animation.FuncAnimation(fig, update, frames=frames_with_progress, init_func=init, blit=False)

# 使用 ffmpeg 将动画保存为 mp4，并显示生成进度
ani.save('output_cartesian_pose.mp4', writer='ffmpeg', fps=30)

print("动画已保存为 output_cartesian_pose.mp4")
