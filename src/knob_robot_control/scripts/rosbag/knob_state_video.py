import rosbag
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
from tqdm import tqdm

# 打开 rosbag 文件并读取 /knob_state topic 数据
bag = rosbag.Bag('fortiss_knob.baglg_2024-09-24-17-41-01.bag')
knob_position_data = []
timestamps = []

for topic, msg, t in bag.read_messages(topics=['/knob_state']):
    knob_position_data.append(msg.position.data)
    timestamps.append(t.to_sec())

bag.close()

# 创建一个子图布局显示 /knob_state 数据
fig, ax = plt.subplots(figsize=(8, 6))  # 创建单个子图
ax.set_title('/knob_state Position Data')

# 初始化时间戳文本
timestamp_text = None

# 初始化函数
def init():
    ax.plot([], [], 'g-', label="/knob_state")  # 初始化线条
    ax.legend(loc='upper right')  # 添加图例
    return ax

# 更新函数
def update(frame):
    global timestamp_text

    ax.clear()
    ax.set_title('/knob_state Position Data')
    ax.set_xlim(min(timestamps), max(timestamps))  # x 轴为时间戳范围
    ax.set_ylim(min(knob_position_data), max(knob_position_data))  # y 轴为位置数据范围
    ax.plot(timestamps[:frame + 1], knob_position_data[:frame + 1], 'g-')

    # 清除之前的时间戳文本
    if timestamp_text is not None:
        timestamp_text.remove()

    # 显示新的时间戳文本
    timestamp_text = fig.text(0.5, 0.03, f"ROS Timestamp: {timestamps[frame]:.2f}", ha='center', fontsize=12)

    return ax

# 创建动画并显示进度
frames = range(len(timestamps))
frames_with_progress = tqdm(frames, desc="Generating /knob_state animation")

ani = animation.FuncAnimation(fig, update, frames=frames_with_progress, init_func=init, blit=False)
ani.save('knob_state_animation_with_timestamp.mp4', writer='ffmpeg', fps=30)
