from platform import node

import rclpy
from rclpy.node import Node
from base_msgs.msg import PlotInfo
import matplotlib.pyplot as plt
import numpy as np

class PlotData(Node):
    def __init__(self):
        super().__init__("data_plot_node")
        self.get_logger().info("Data Plot Node has been started.")

        # Subscribe to the topic where the data is published
        self.subscrption = self.create_subscription(
            PlotInfo,
            "planning_core/plot_info",
            self.plot_info_callback,
            10
        )

    # Callback function to plot the received data
    def plot_info_callback(self, plot_info):
        # Process the received PlotInfo message and update the plot
        plt.clf()  # Clear the current figure
        if not plot_info.trajectory_info.local_trajectory: # check if the trajectory is empty
            self.get_logger().warn("Received empty trajectory, skipping plot update.")
            return
        
        # Extract data from the PlotInfo message
        s = np.asarray(
            [
                point.path_point.s 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )
    
        l = np.asarray(
            [
                point.path_point.l 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        dl_ds = np.asarray(
            [
                point.path_point.dl_ds 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        theta = np.asarray(
            [
                point.path_point.theta 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        kappa = np.asarray(
            [
                point.path_point.kappa 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        dkappa = np.asarray(
            [
                point.path_point.dkappa 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        t = np.asarray(
            [
                point.speed_point.t 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        s_2path = np.asarray(
            [
                point.speed_point.s_2path 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        speed = np.asarray(
            [
                point.speed_point.speed 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        acceleration = np.asarray(
            [
                point.speed_point.acceleration 
                for point in plot_info.trajectory_info.local_trajectory
            ]
        )

        # Create subplots for each variable
        fig1 = plt.subplot(4,1,1)
        fig2 = plt.subplot(4,1,2)
        fig3 = plt.subplot(4,1,3)
        fig4 = plt.subplot(4,1,4)

        plt.sca(fig1)
        # plot ego car's s-l
        fig1.plot(s, l, label="s vs l",linestyle='solid', color='green')

        # plot tps' s-l
        for tp in plot_info.obs_info:
            # calculate tp's bounding box in s-l space
            tpLeftBound = tp.s - (tp.obs_length / 2.0)
            tpRightBound = tp.s + (tp.obs_length / 2.0)
            tpUpperBound = tp.l + (tp.obs_width / 2.0)
            tpLowerBound = tp.l - (tp.obs_width / 2.0)

            # plot polygon for tp
            tp_polygon = plt.Polygon(
                xy = [
                    [tpLeftBound, tpLowerBound],
                    [tpLeftBound, tpUpperBound],
                    [tpRightBound, tpUpperBound],
                    [tpRightBound, tpLowerBound]
                 ],
                color = 'blue',
                alpha = 0.8             
            )
            fig1.add_patch(tp_polygon)

        fig1.set_xlabel('s')
        fig1.set_ylabel('l')
        fig1.legend()
        fig1.grid()
        fig1.set_title('S-L Plot')

        plt.sca(fig2)
        fig2.plot(s, dl_ds, label="s vs dl/ds", linestyle='solid', color='red')
        fig2.plot(s, theta, label="s vs theta", linestyle='solid', color='orange')
        fig2.plot(s, kappa, label="s vs kappa", linestyle='solid', color='cyan')
        fig2.plot(s, dkappa, label="s vs dkappa", linestyle='solid', color='magenta')
        fig2.set_xlabel('s')
        fig2.set_ylabel('l-s related parameters')
        fig2.legend()
        fig2.grid()
        fig2.set_title('l-s related parameters')

        plt.sca(fig3)
        fig3.plot(t, s_2path, label="t vs s_2path", linestyle='solid', color='green')

        # plot tps' s-t
        for tp in plot_info.obs_info:
            # calculate tp's bounding box in s-t space
            deltaS = tp.ds_dt_2path * (tp.t_out - tp.t_in)

            tpLeftLowerPt = tp.s_2path - (tp.obs_length / 2.0)
            tpLeftUpperPt = tp.s_2path + (tp.obs_length / 2.0)
            tpRightLowerPt = tpLeftLowerPt + deltaS
            tpRightUpperPt = tpLeftUpperPt + deltaS

            # plot polygon for tp
            tp_polygon = plt.Polygon(
                xy = [
                    [tp.t_in, tpLeftLowerPt],
                    [tp.t_in, tpLeftUpperPt],
                    [tp.t_out, tpRightUpperPt],
                    [tp.t_out, tpRightLowerPt]
                 ],
                color = 'blue',
                alpha = 0.8             
            )
            fig3.add_patch(tp_polygon)

        fig3.set_xlabel('t')
        fig3.set_ylabel('s_2path')
        fig3.legend()
        fig3.grid()
        fig3.set_title('S-T Plot')

        plt.sca(fig4)
        fig4.plot(t, speed, label="t vs speed", linestyle='solid', color='red')
        fig4.plot(t, acceleration, label="t vs acceleration", linestyle='solid', color='orange')
        fig4.set_xlabel('t')
        fig4.set_ylabel('speed and acceleration')
        fig4.legend()
        fig4.grid()
        fig4.set_title('Speed and Acceleration Plot')

        plt.pause(0.05)  # Pause to update the plot


def main(args=None):
    rclpy.init(args=args)
    plotNode = PlotData()
    try:
        rclpy.spin(plotNode)
    except KeyboardInterrupt:
        print("Interrupt by user, shutting down.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
