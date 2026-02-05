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
        
        # 数据缓存 (只保留最近 200 个点，防止内存溢出)
        self.max_len = 200
        self.time_data = deque(maxlen=self.max_len)
        self.x_data = deque(maxlen=self.max_len)
        self.y_data = deque(maxlen=self.max_len)
        
        self.start_time = None

    def listener_callback(self, msg):
        # 记录起始时间
        if self.start_time is None:
            self.start_time = msg.timestamp / 1e6 # 转换为秒
            
        current_time = (msg.timestamp / 1e6) - self.start_time
        
        # NED 坐标系：
        # position[0] = North (X)
        # position[1] = East  (Y)
        x = msg.position[0]
        y = msg.position[1]

        # 存入队列
        self.time_data.append(current_time)
        self.x_data.append(x)
        self.y_data.append(y)

def main():
    rclpy.init()
    plotter = Plotter()

    # --- 设置 Matplotlib 画布 ---
    fig = plt.figure(figsize=(12, 6))
    fig.suptitle("Real-time Trajectory Monitor (NED Frame)")

    # 子图 1: X-T (北向位置随时间变化)
    ax1 = fig.add_subplot(2, 2, 1)
    line_xt, = ax1.plot([], [], 'r-')
    ax1.set_title("X (North) - Time")
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("Position (m)")
    ax1.grid(True)

    # 子图 2: Y-T (东向位置随时间变化)
    ax2 = fig.add_subplot(2, 2, 2)
    line_yt, = ax2.plot([], [], 'b-')
    ax2.set_title("Y (East) - Time")
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Position (m)")
    ax2.grid(True)

    # 子图 3: X-Y (2D 平面轨迹 - 8字形)
    ax3 = fig.add_subplot(2, 2, (3, 4)) # 占据下方一整行
    line_xy, = ax3.plot([], [], 'g-', lw=2)
    ax3.set_title("X - Y Trajectory (Top View)")
    ax3.set_xlabel("Y (East) [m]") # 注意: 数学坐标系通常横轴是Y(East)
    ax3.set_ylabel("X (North) [m]")
    ax3.grid(True)
    ax3.axis('equal') # 保证比例一致，这样圆才是圆，8字才是8字

    # 动画更新函数
    def update(frame):
        # 将数据转为 list 绘图
        t = list(plotter.time_data)
        x = list(plotter.x_data)
        y = list(plotter.y_data)

        if not t:
            return line_xt, line_yt, line_xy

        # 更新 X-T
        line_xt.set_data(t, x)
        ax1.set_xlim(max(0, t[-1] - 10), t[-1] + 1) # 滚动窗口 10秒
        ax1.set_ylim(min(x)-1, max(x)+1)

        # 更新 Y-T
        line_yt.set_data(t, y)
        ax2.set_xlim(max(0, t[-1] - 10), t[-1] + 1)
        ax2.set_ylim(min(y)-1, max(y)+1)

        # 更新 X-Y
        line_xy.set_data(y, x) # 注意这里我把 Y(East) 作为横轴，X(North) 作为纵轴，符合地图直觉
        ax3.set_xlim(-6, 6) # 固定范围，方便观察 8 字
        ax3.set_ylim(-6, 6)

        return line_xt, line_yt, line_xy

    # 启动 ROS 2 接收线程（防止 GUI 卡死 ROS）
    thread = threading.Thread(target=rclpy.spin, args=(plotter,), daemon=True)
    thread.start()

    # 启动动画
    ani = animation.FuncAnimation(fig, update, interval=100) # 100ms 刷新一次
    plt.tight_layout()
    plt.show()

    # 关闭时清理
    rclpy.shutdown()

if __name__ == '__main__':
    main()