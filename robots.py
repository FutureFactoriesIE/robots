import base64
import functools
import io
import itertools
import math
from dataclasses import dataclass, field
from typing import List, Tuple, NamedTuple, Iterator

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

# plot properties
ax = plt.axes(projection='3d')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
ax.set_xlim3d(-600, 600)
ax.set_ylim3d(-600, 600)
ax.set_zlim3d(0, 900)


class Coord3D(NamedTuple):
    """A class representing a 3D coordinate"""

    x: float
    """The x-coordinate"""
    y: float
    """The y-coordinate"""
    z: float
    """The z-coordinate"""


@dataclass
class PlotConfig:
    """An object that stores config options for plotting the robot's position
    using `Robot.get_plot()`
    """

    line_color: str = 'black'
    """The color of the line to plot"""

    enable_tool_history: bool = False
    """Whether to enable showing a dotted line representing the path
    that the robot's tool took since this config option was enabled
    """

    tool_history_color: str = 'blue'
    """The color of the tool history dotted line"""

    enable_axis_lines: bool = False
    """Whether to enable showing red, green, and blue lines indicating
    the rotation of each of the robot's joints
    """

    axis_line_length: int = 50
    """How long the axis lines should be"""

    enable_tool_position: bool = True
    """Whether to enable showing the coordinates of the tool's position
    at the top of the plot
    """


@dataclass
class ToolHistory:
    """An object that stores the robot's tool history as a series of coordinates"""

    xs: List[float] = field(default_factory=list)
    """A list containing all of the X coordinates of the tool's history"""

    ys: List[float] = field(default_factory=list)
    """A list containing all of the Y coordinates of the tool's history"""

    zs: List[float] = field(default_factory=list)
    """A list containing all of the Z coordinates of the tool's history"""

    def clear(self):
        """Clear the tool history"""
        self.xs.clear()
        self.ys.clear()
        self.zs.clear()

    def add_coord(self, x: float, y: float, z: float):
        """Add a coordinate to the tool history"""
        self.xs.append(x)
        self.ys.append(y)
        self.zs.append(z)

    def to_plot3d(self) -> Tuple[List[float], List[float], List[float]]:
        """Packages up the tool history in a tuple to make plotting easier"""
        return self.xs, self.ys, self.zs


class Robot:
    """A class to aid in plotting and calculating the joint positions of a robot

    Attributes
    ----------
    plot_config : PlotConfig
        This robot's personal `PlotConfig` object that stores config options for
        plotting the robot's position using `Robot.get_plot()`
    _link_lengths : Tuple[int, ...]
        The lengths (in mm) of each link of the robot in order
    _joint_angles : List[float]
        The joint angles of the robots

        Note: This attribute should not be directly referenced; instead use
        the `Robot.joint_angles` property
    _tool_history : ToolHistory
        This robot's personal `ToolHistory` object that stores the coordinate
        history of the tool
    """

    def __init__(self, link_lengths: Tuple[int, ...], num_joints: int):
        """
        Parameters
        ----------
        link_lengths : Tuple[int, ...]
            The lengths (in mm) of each link of the robot in order
        num_joints : int
            How many joints the robot has
        """

        self._link_lengths = link_lengths
        self._joint_angles = [0.0] * num_joints
        self.plot_config = PlotConfig()
        self._tool_history = ToolHistory()

    @property
    def joint_angles(self) -> List[float]:
        """The current joint angles of the robot (in degrees)"""
        return self._joint_angles

    @joint_angles.setter
    def joint_angles(self, value: List[float]):
        """Set the current joint angles of the robot (in degrees)

        Parameters
        ----------
        value : List[float]
            The new joint angles to set the robot to
        """

        if not isinstance(value, list):
            raise ValueError(f'joint_angles must be a list, not {type(value)}')
        elif len(value) != len(self._joint_angles):
            raise ValueError(f'joint_angles must have {len(self._joint_angles)} values, not {len(value)}')
        self._joint_angles = value

    @property
    def d_h_table(self) -> np.ndarray:
        """The robot's Denavit-Hartenberg table for use in forward kinematics"""
        # should be overridden by subclasses
        raise NotImplementedError

    def clear_tool_history(self):
        """Clear the robot's tool history"""
        self._tool_history.clear()

    def get_fk_frames(self) -> List[np.ndarray]:
        """Get the output of performing forward kinematics on this robot's joint angles"""
        d_h_table = self.d_h_table
        frames = []
        for i in range(len(d_h_table)):
            frames.append(
                np.array(
                    [[
                        np.cos(d_h_table[i, 0]),
                        -np.sin(d_h_table[i, 0]) * np.cos(d_h_table[i, 1]),
                        np.sin(d_h_table[i, 0]) * np.sin(d_h_table[i, 1]),
                        d_h_table[i, 2] * np.cos(d_h_table[i, 0])
                    ],
                        [
                            np.sin(d_h_table[i, 0]),
                            np.cos(d_h_table[i, 0]) * np.cos(d_h_table[i, 1]),
                            -np.cos(d_h_table[i, 0]) * np.sin(d_h_table[i, 1]),
                            d_h_table[i, 2] * np.sin(d_h_table[i, 0])
                        ],
                        [
                            0,
                            np.sin(d_h_table[i, 1]),
                            np.cos(d_h_table[i, 1]), d_h_table[i, 3]
                        ], [0, 0, 0, 1]]))

        return frames

    def get_accumulated_frames(self) -> Iterator[np.ndarray]:
        """Performs matrix multiplication on all previous forward kinematic frames
        so that each iteration gives the position/rotation frame of each joint
        """

        frames = self.get_fk_frames()
        base = np.identity(4)
        return itertools.accumulate(frames, np.matmul, initial=base)

    def get_coordinates(self, *, apply_rounding: bool = False) -> List[Tuple[int, int, int]]:
        """Gets the x, y, and z coordinates of each joint by performing forward kinematics

        Parameters
        ----------
        apply_rounding: bool, default=False
            Should the x, y, and z values be rounded to the nearest whole number

        Returns
        -------
        List[Tuple[int, int, int]]
            A list of tuples in the form of (x, y, z)
        """

        result = [tuple(current[0:3, 3]) for current in self.get_accumulated_frames()]
        if apply_rounding:
            # noinspection PyTypeChecker
            return list(map(lambda x: tuple(map(lambda y: round(y), x)), result))
        return result

    def get_plot(self) -> bytes:
        """Returns JPG encoded bytes that represent a plot of the robot's current position

        The plot can be configured using the `Robot().plot_config` attribute
        """

        # clearing the lists is faster than cla()
        # changed in v9 - fix for new version of matplotlib
        while len(ax.lines):
            ax.lines[0].remove()
        while len(ax.texts):
            ax.texts[0].remove()

        # organize the coords
        xs = []
        ys = []
        zs = []
        for current in self.get_accumulated_frames():
            xs.append(current[0, 3])
            ys.append(current[1, 3])
            zs.append(current[2, 3])

        # dots showing tool history
        if self.plot_config.enable_tool_history:
            self._tool_history.add_coord(xs[-1], ys[-1], zs[-1])
            ax.plot3D(*self._tool_history.to_plot3d(), ls=':', color=self.plot_config.tool_history_color)

        # x, y, z lines
        if self.plot_config.enable_axis_lines:
            axll = self.plot_config.axis_line_length
            for frame, x, y, z in zip(self.get_accumulated_frames(), xs, ys, zs):
                ax.plot3D([x, x + frame[0, 0] * axll], [y, y + frame[1, 0] * axll], [z, z + frame[2, 0] * axll],
                          color='red')
                ax.plot3D([x, x + frame[0, 1] * axll], [y, y + frame[1, 1] * axll], [z, z + frame[2, 1] * axll],
                          color='green')
                ax.plot3D([x, x + frame[0, 2] * axll], [y, y + frame[1, 2] * axll], [z, z + frame[2, 2] * axll],
                          color='blue')

        # actual robot path
        ax.plot3D(xs, ys, zs, color=self.plot_config.line_color)

        # format and display the tool position
        if self.plot_config.enable_tool_position:
            coords = f'({round(xs[-1])}, {round(ys[-1])}, {round(zs[-1])})'
            ax.text2D(0.05, 0.95, f"Tool position: {coords}", transform=ax.transAxes)

        # save plot as bytes
        with io.BytesIO() as plt_bytes:
            plt.savefig(plt_bytes, format='jpg')
            plt_bytes.seek(0)
            return plt_bytes.read()

    def get_pil_image(self) -> Image.Image:
        """Convert the robot's current position plot into a PIL Image object

        Returns
        -------
        PIL.Image.Image
            The converted data as a PIL Image object
        """

        with io.BytesIO(self.get_plot()) as data:
            return Image.open(data).copy()

    def get_base64_plot(self) -> str:
        """A base64 representation of the robot's current position plot"""
        return base64.b64encode(self.get_plot()).decode()

    def save_plot(self, filename: str):
        """Saves the robot's current position plot to a JPG file

        Parameters
        ----------
        filename : str
            The filename to save the image as (do not include the extension)
        """

        with io.BytesIO(self.get_plot()) as data:
            image = Image.open(data)
            image.save(filename + '.jpg')

    def save_gif(self, all_joint_angles: List[List[float]], filename: str, fps: int = 4):
        """Create a GIF from a list of lists of joint angles

        This method creates a plot for each item in `all_joint_angles` and combines all of these
        plots into a single GIF

        Parameters
        ----------
        all_joint_angles : List[List[float]]
            A list of lists of joint angles to create plots from
        filename : str
            The filename to save the GIF as (do not include the extension)
        fps: int
            The amount of images to show per second in the final GIF
        """

        def generate_plots():
            for joint_angles in all_joint_angles:
                self.joint_angles = joint_angles
                yield self.get_plot()

        images = (Image.open(io.BytesIO(plot)) for plot in generate_plots())
        first_image = next(images)
        first_image.save(f'{filename}.gif', format='GIF', append_images=images, save_all=True,
                         duration=len(all_joint_angles) // fps, loop=0)

    def get_position_frame(self, frame_index: int) -> np.ndarray:
        """Returns the position/rotation frame that results from performing forward
        kinematics on this robot's joint angles and then multiplying the frames
        together

        Parameters
        ----------
        frame_index : int
            The index of the frame to get the position/rotation frame of
        """

        return functools.reduce(lambda a, b: a @ b, self.get_fk_frames()[:frame_index])

    @staticmethod
    def get_velocity(start_pos: Coord3D, end_pos: Coord3D, t: float) -> Tuple[float, Coord3D]:
        """Get the velocity of a start and end position over a set amount of time

        Parameters
        ----------
        start_pos : Coord3D
            The start position
        end_pos : Coord3D
            The end position
        t : float
            The time it took to get from the start position to the end position

        Returns
        -------
        Tuple[float, Coord3D]
            A tuple with the zeroth element representing the actual velocity and
            the first element a `Coord3D` object with the x, y, and z values
            corresponding to the x, y, and z velocities, respectively
        """

        # distances of each axis
        dx = end_pos.x - start_pos.x
        dy = end_pos.y - start_pos.y
        dz = end_pos.z - start_pos.z

        # total distance
        d = math.sqrt(dx ** 2 + dy ** 2 + dz ** 2)

        # result as (actual velocity, Coord3D[axis_velocities])
        return d / t, Coord3D(dx / t, dy / t, dz / t)


class WhiteRobot(Robot):
    """A subclass of `Robot` that represents the white robots in the cell"""

    def __init__(self):
        # Link lengths in millimeters
        a1 = 275  # Length of link 1
        a2 = 190  # Length of link 2
        a3 = 700  # Length of link 3
        a4 = 190  # Length of link 4
        a5 = 500  # Length of link 5
        a6 = 162  # Length of link 6
        a7 = 210  # Length of link 7 + gripper

        super().__init__((a1, a2, a3, a4, a5, a6, a7), 6)

    @property
    def d_h_table(self):
        a1, a2, a3, a4, a5, a6, a7 = self._link_lengths
        theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = self.joint_angles
        return np.array([[np.deg2rad(theta_1),
                          np.deg2rad(90), 0, a1],
                         [np.deg2rad(theta_2 - 90),
                          np.deg2rad(180), -a3, a2],
                         [np.deg2rad(theta_3 - 90),
                          np.deg2rad(-90), 0, a4],
                         [np.deg2rad(theta_4),
                          np.deg2rad(90), 0, a5],
                         [np.deg2rad(theta_5),
                          np.deg2rad(-90), 0, a6],
                         [np.deg2rad(theta_6), 0, 0, a7]])


class BlueRobot(Robot):
    """A subclass of `Robot` that represents the blue robots in the cell"""

    def __init__(self):
        # Link lengths in millimeters
        a1 = 330  # Length of link 1
        a2 = 40  # Length of link 2
        a3 = 385  # Length of link 3
        a4_5 = 340  # Length of link 4 and 5
        a6 = 160  # Length of link 6 + gripper tip

        super().__init__((a1, a2, a3, a4_5, a6), 6)

    @property
    def d_h_table(self):
        a1, a2, a3, a4_5, a6 = self._link_lengths
        theta_1, theta_2, theta_3, theta_4, theta_5, theta_6 = self.joint_angles
        return np.array([[np.deg2rad(theta_1 + 90),
                          np.deg2rad(90), a2, a1],
                         [np.deg2rad(theta_2 + 90), 0, a3, 0],
                         [np.deg2rad(theta_3),
                          np.deg2rad(90), 0, 0],
                         [np.deg2rad(theta_4),
                          np.deg2rad(-90), 0, a4_5],
                         [np.deg2rad(theta_5),
                          np.deg2rad(90), 0, 0],
                         [np.deg2rad(theta_6), 0, 0, a6]])


if __name__ == '__main__':
    R01 = WhiteRobot()
    R02 = BlueRobot()
    R03 = BlueRobot()
