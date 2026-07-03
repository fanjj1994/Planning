import rclpy
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

class PlotData(Node):
    def __init__(self):
        super().__init__('data_plot_node')
        self.get_logger().info("data_plot_node created")


def main():
    rclpy.init()
    plot_node = PlotData()

    try:
        rclpy.spin(plot_node)
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        rclpy.shutdown()#防止按ctrl+c时，程序异常退出，导致无法正常关闭节点


if __name__ == '__main__':
    main()
