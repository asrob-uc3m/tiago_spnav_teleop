# tiago_spnav_teleop

TIAGO (and TIAGo++) arm teleoperation with 3Dconnexion's SpaceMouse.

## Running in simulation

We recommend Docker. It is assumed that you have cloned this repository inside your current working directory.

#### For single-arm TIAGo

```
docker run -it --privileged --network host -e DISPLAY \
           -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
           -v $XAUTHORITY:/root/.Xauthority:ro \
           -v /dev/bus/usb:/dev/bus/usb \
           -v $PWD/tiago_spnav_teleop:/tiago_public_ws/src/tiago_spnav_teleop \
       palroboticssl/tiago_tutorials:noetic
```

Then (assuming you have run `catkin build tiago_spnav_teleop`):

```
roslaunch tiago_spnav_teleop tiago_gazebo.launch
```

### For dual-arm TIAGo++

```
docker run -it --privileged --network host -e DISPLAY \
           -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
           -v $XAUTHORITY:/root/.Xauthority:ro \
           -v /dev/bus/usb:/dev/bus/usb \
           -v $PWD/tiago_spnav_teleop:/tiago_dual_public_ws/src/tiago_spnav_teleop \
       palroboticssl/tiago_dual_tutorials:noetic
```

Then (assuming you have run `catkin build tiago_spnav_teleop`):

```
roslaunch tiago_spnav_teleop tiago_dual_gazebo.launch arm:=right
```

Alternatively, use `arm:=left` or `arm:=both` to achieve the obvious result.

### Post-demo actions

If you want to restore the default controllers after closing this application, run:

```
rosrun tiago_spnav_teleop stop_spnav.py
```

Append `--arm left`, `--arm right` or `--arm both` correspondingly if using TIAGo++.

## Citation

If you found this project useful, please consider citing the following work:

Calzada García, A., Łukawski, B., Victores, J., & Balaguer, C. (2024). Teleoperation of the robot TIAGo with a 3D mouse controller. In Actas del Simposio de Robótica, Bioingeniería y Visión por Computador (pp. 133–138). Universidad de Extremadura.

```bibtex
@inproceedings{calzada2024srbv,
    author={Calzada García, Ana and Łukawski, Bartek and Victores, Juan G. and Balaguer, Carlos},
    title={{Teleoperation of the robot TIAGo with a 3D mouse controller}},
    booktitle={Actas del Simposio de Robótica, Bioingeniería y Visión por Computador},
    year={2024},
    pages={133--138},
    publisher={Universidad de Extremadura},
    url={http://hdl.handle.net/10662/21260},
}
```
