# robots
This module allows for easy calculations and presentation of robots. The main component of the robots module is performing forward kinematics on the robot’s joint angles and plotting them on a graph using matplotlib. Velocities can also be calculated for each joint of the robot.

This README is a word-for-word copy of my original User Guide which can be found [here](https://docs.google.com/document/d/1sPeCcuaspeQZvhTyePz7-apKUxT0v-8-mqNif_TsjL0/edit?usp=sharing).

The robots module itself can be found [here](robots.py).

Actual documentation can be found [here](https://sites.google.com/view/robots-docs/home).

## Forward Kinematics
Calculating forward kinematics from joint angles is simple using the [`Robot`](https://github.com/FutureFactoriesIE/robots/blob/61b47b4b243d1e2318eb9c994401fba25a0e7181/robots.py#L95) class. Set the joint angles using the [`Robot.joint_angles`](https://github.com/FutureFactoriesIE/robots/blob/61b47b4b243d1e2318eb9c994401fba25a0e7181/robots.py#L131) property and call the [`get_fk_frames()`](https://github.com/FutureFactoriesIE/robots/blob/61b47b4b243d1e2318eb9c994401fba25a0e7181/robots.py#L161) method.

```python
from robots import WhiteRobot
R01 = WhiteRobot()
R01.joint_angles = [0, 0, 90, 0, -90, 0]
frames = R01.get_fk_frames()
```

The output of this method is the result of calculating forward kinematics on the current robot’s joint angles (see the [documentation](https://sites.google.com/view/robots-docs/home) for the actual return type). Therefore to find the actual position of joints, you will need to multiply each matrix by its previous matrices. While you can use the `numpy` [`@`](https://numpy.org/doc/stable/reference/generated/numpy.matmul.html#numpy.matmul) operator and go frame by frame, the [`Robot`](https://github.com/FutureFactoriesIE/robots/blob/61b47b4b243d1e2318eb9c994401fba25a0e7181/robots.py#L95) class has a method that does this more efficiently:

```python
position_frames = R01.get_accumulated_frames()
```

This method multiplies each frame returned from [`get_fk_frames()`](https://github.com/FutureFactoriesIE/robots/blob/61b47b4b243d1e2318eb9c994401fba25a0e7181/robots.py#L161) by its previous frames using [`itertools.accumulate`](https://docs.python.org/3/library/itertools.html#itertools.accumulate), returning an iterator of the actual position/rotation frames of each of the robot’s joints.

## Creating Plots
Creating visuals for the white and blue robots can be done in as few as 4 lines:

```python
from robots import WhiteRobot
R01 = WhiteRobot()
R01.joint_angles = [0, 0, 90, 0, -90, 0]
plot_bytes = R01.get_plot()
```

The return value of [`get_plot()`](https://github.com/FutureFactoriesIE/robots/blob/61b47b4b243d1e2318eb9c994401fba25a0e7181/robots.py#L197) is a bytes representation of a plot in JPG format. To save the plot as an actual image file, use the [`save_plot()`](https://github.com/FutureFactoriesIE/robots/blob/61b47b4b243d1e2318eb9c994401fba25a0e7181/robots.py#L249) method instead.

```python
R01.save_plot('plot')
```

![plot.jpg](https://drive.google.com/uc?export=view&id=1fFxHwNMkpjjlBaY3EJMOAH8ymQiNr3km)
plot.jpg

There is a secondary option for creating a plot: [`get_base64_plot()`](https://github.com/FutureFactoriesIE/robots/blob/61b47b4b243d1e2318eb9c994401fba25a0e7181/robots.py#L245). This method returns a base64 encoded string of the plot, which can be used in conjunction with the [edge-interface](https://github.com/FutureFactoriesIE/edge-interface) module to display the plot on a web interface.

An example of this is show below:

```python
from edge_interface import EdgeInterface
from robots import WhiteRobot

interface = EdgeInterface(__name__)
interface.add_page('/', 'index.html')
interface.start_server()

R01 = WhiteRobot()
R01.joint_angles = [0, 0, 90, 0, -90, 0]

interface.pages['/'].set_image_base64('plot1', R01.get_base64_plot())
```

GIFs can also be created to show the path of the robot over time. Use the [`save_gif()`](https://github.com/FutureFactoriesIE/robots/blob/61b47b4b243d1e2318eb9c994401fba25a0e7181/robots.py#L262) method and pass in a list of lists of joint angles. A series of plots will be created for each list of joint angles passed in, which will then be combined sequentially to form a GIF.

```python
from robots import WhiteRobot
R01 = WhiteRobot()
all_joint_angles = [
    [0, 0, 0.0, 0, 0, 0],
    [0, 0, 0.5, -1, 0, 0],
    [0, 0, 1.0, -2, 0, 0],
    [0, 0, 1.5, -3, 0, 0],
    [0, 0, 2.0, -4, 0, 0],
    [0, 0, 2.5, -5, 0, 0],
    [0, 0, 3.0, -6, 0, 0],
    [0, 0, 3.5, -7, 0, 0],
    [0, 0, 4.0, -8, 0, 0],
    [0, 0, 4.5, -9, 0, 0]
]
R01.save_gif(all_joint_angles, 'plot')
```

To change how the plots are rendered, as well as to add the tool history to the GIF, see the [**Configuring Plots**](#configuring-plots) section of this guide.


## Configuring Plots
There are a few settings that can be configured to change the way the plot looks, as well as how fast the plot can be rendered. Each config option can be accessed using the [`Robot.plot_config`](https://github.com/FutureFactoriesIE/robots/blob/61b47b4b243d1e2318eb9c994401fba25a0e7181/robots.py#L127) attribute, which is a [`PlotConfig`](https://github.com/FutureFactoriesIE/robots/blob/61b47b4b243d1e2318eb9c994401fba25a0e7181/robots.py#L35) object. Changing these options is as simple as setting the attributes of this [`PlotConfig`](https://github.com/FutureFactoriesIE/robots/blob/61b47b4b243d1e2318eb9c994401fba25a0e7181/robots.py#L35) object.

```python
from robots import WhiteRobot
R01 = WhiteRobot()
```

The default [`PlotConfig`](https://github.com/FutureFactoriesIE/robots/blob/61b47b4b243d1e2318eb9c994401fba25a0e7181/robots.py#L35) and [`joint_angles`](https://github.com/FutureFactoriesIE/robots/blob/61b47b4b243d1e2318eb9c994401fba25a0e7181/robots.py#L131) for the [`WhiteRobot`](https://github.com/FutureFactoriesIE/robots/blob/61b47b4b243d1e2318eb9c994401fba25a0e7181/robots.py#L334) will create plots that render like this:

![plot-config-0.gif](https://drive.google.com/uc?export=view&id=1cCyTw-xPpafmTppx5gpY1FKnDSTmq_2J)

| Config Attribute | Default Value | Image |
| ---- | ---- | ---- |
| `R01.plot_config.line_color = 'red'` | `'black'` | ![plot-config-1.gif](https://drive.google.com/uc?export=view&id=1SlY97ttd5-dPYjwUeXBLk_QRinJSKjV1) |
| `R01.plot_config.enable_tool_history = True` | `False` | ![plot-config-2.gif](https://drive.google.com/uc?export=view&id=1CVWJ92f8rfoJV80e3YDvIZwut0GFQRy2)
| `R01.plot_config.tool_history_color = 'green'` | `'blue'` | ![plot-config-3.gif](https://drive.google.com/uc?export=view&id=1ufI1gkAVa6JXcxzOxdQKEFfF9kiBSjBd) |
| `R01.plot_config.enable_axis_lines = True` | `False` | ![plot-config-4.gif](https://drive.google.com/uc?export=view&id=1D-_sgeihGqfHBeHH4s_A3mYlLm1zBd9Y) |
| `R01.plot_config.axis_line_length = 100` | `50` | ![plot-config-5.gif](https://drive.google.com/uc?export=view&id=1M59tSf7vcWIHu9agM24jTxRGTpckHnCa) |
| `R01.plot_config.enable_tool_position = False` | `True` | ![plot-config-5.gif](https://drive.google.com/uc?export=view&id=1sMi5RPLdUtxqeGrUNT2PBy_lALJj1jsc) |

***Note: Some images are not loading correctly on GitHub, so to view them, use the link to the original User Guide [here](https://docs.google.com/document/d/1sPeCcuaspeQZvhTyePz7-apKUxT0v-8-mqNif_TsjL0/edit?usp=sharing).***
