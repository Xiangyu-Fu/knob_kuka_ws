import rosbag
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from tqdm import tqdm

# 打开 rosbag 文件并读取 /tcp_force topic 数据
bag = rosbag.Bag('fortiss_knob.baglg_2024-09-24-17-41-01.bag')
tcp_force_data = []
timestamps = []

for topic, msg, t in bag.read_messages(topics=['/tcp_force']):
    tcp_force_data.append(msg.data)
    timestamps.append(t.to_sec())

tcp_force_data = tcp_force_data[::10]
timestamps = timestamps[::10]

bag.close()

# 创建一个子图布局显示 /tcp_force 数据
fig, ax = plt.subplots(figsize=(8, 6))  # 创建单个子图
ax.set_title('/tcp_force Data')

# 初始化函数
def init():
    ax.plot([], [], 'b-', label="/tcp_force")  # 初始化线条
    ax.legend(loc='upper right')  # 添加图例
    return ax

# 更新函数
def update(frame):
    ax.clear()
    ax.set_title('/tcp_force Data')
    ax.set_xlim(0, len(tcp_force_data))
    ax.set_ylim(min(tcp_force_data), max(tcp_force_data))
    ax.plot(range(frame + 1), tcp_force_data[:frame + 1], 'b-')

    return ax

# 创建动画并显示进度
frames = range(len(tcp_force_data))
frames_with_progress = tqdm(frames, desc="Generating /tcp_force animation")

ani = animation.FuncAnimation(fig, update, frames=frames_with_progress, init_func=init, blit=False)
ani.save('tcp_force_animation.mp4', writer='ffmpeg', fps=30)
