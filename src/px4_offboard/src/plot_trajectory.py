import rclpy
from rclpy.node import Node
from px4_msgs.msg import TrajectorySetpoint
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import threading

class Plotter(Node):
    def __init__(self):
        super().__init__('plotter_node')
        
        # 订阅 C++ 节点发出的轨迹设定点
        self.subscription = self.create_subscription(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            self.listener_callback,
            10)
        
        # 【修改点1】增大缓冲区，确保能存下整个 8 字的轨迹
        # 20Hz 发送频率下，2000个点可以记录 100秒 的数据，足够画好几个 8 字了
        self.max_len = 2000 
        self.time_data = deque(maxlen=self.max_len)
        self.x_data = deque(maxlen=self.max_len)
        self.y_data = deque(maxlen=self.max_len)
        
        self.start_time = None

    def listener_callback(self, msg):
        if self.start_time is None:
            self.start_time = msg.timestamp / 1e6 
            
        current_time = (msg.timestamp / 1e6) - self.start_time
        
        # NED 坐标系
        x = msg.position[0]
        y = msg.position[1]

        self.time_data.append(current_time)
        self.x_data.append(x)
        self.y_data.append(y)

def main():
    rclpy.init()
    plotter = Plotter()

    # 设置画布
    fig = plt.figure(figsize=(10, 8)) # 调整一下窗口比例
    fig.suptitle("Real-time Trajectory Monitor (NED Frame)")

    # 子图 1: X-T
    ax1 = fig.add_subplot(2, 2, 1)
    line_xt, = ax1.plot([], [], 'r-')
    ax1.set_title("X (North) - Time")
    ax1.grid(True)

    # 子图 2: Y-T
    ax2 = fig.add_subplot(2, 2, 2)
    line_yt, = ax2.plot([], [], 'b-')
    ax2.set_title("Y (East) - Time")
    ax2.grid(True)

    # 子图 3: X-Y (Top View)
    ax3 = fig.add_subplot(2, 2, (3, 4))
    line_xy, = ax3.plot([], [], 'g-', lw=2)
    ax3.set_title("X - Y Trajectory (Top View)")
    ax3.set_xlabel("Y (East) [m]") 
    ax3.set_ylabel("X (North) [m]")
    ax3.grid(True)
    
    # 【修改点2】强制设置 XY 轴比例相同，否则 8 字会变成 00 或者扁的
    ax3.set_aspect('equal', adjustable='box')

    # 【修改点3】直接写死范围。
    # 既然 C++ 代码里半径 radius=5.0，那么我们设成 +/- 8.0 绝对能显示全
    FIXED_LIMIT = 8.0
    ax3.set_xlim(-FIXED_LIMIT, FIXED_LIMIT)
    ax3.set_ylim(-FIXED_LIMIT, FIXED_LIMIT)

    def update(frame):
        t = list(plotter.time_data)
        x = list(plotter.x_data)
        y = list(plotter.y_data)

        if not t:
            return line_xt, line_yt, line_xy

        # 更新 X-T
        line_xt.set_data(t, x)
        ax1.set_xlim(max(0, t[-1] - 10), t[-1] + 1)
        if x: ax1.set_ylim(min(x)-1, max(x)+1)

        # 更新 Y-T
        line_yt.set_data(t, y)
        ax2.set_xlim(max(0, t[-1] - 10), t[-1] + 1)
        if y: ax2.set_ylim(min(y)-1, max(y)+1)

        # 更新 X-Y
        line_xy.set_data(y, x) 
        
        # 注意：这里不再动态设置 ax3 的 limit，使用初始化时的固定范围 (-8, 8)
        # 这样图像非常稳定，不会跳动，也不会切边

        return line_xt, line_yt, line_xy

    # 启动 ROS 线程
    thread = threading.Thread(target=rclpy.spin, args=(plotter,), daemon=True)
    thread.start()

    # 启动动画
    ani = animation.FuncAnimation(fig, update, interval=100)
    plt.tight_layout()
    plt.show()

    rclpy.shutdown()

if __name__ == '__main__':
    main()