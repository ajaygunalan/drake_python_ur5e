# UR5e Manipulation Examples

## Dependencies

- [Drake](https://drake.mit.edu/) - Robotics simulation
- [manipulation](https://manipulation.csail.mit.edu/) - MIT textbook library
- [ur_rtde](https://sdurobotics.gitlab.io/ur_rtde/) - Hardware only

## Setup

```bash
git clone https://github.com/ajaygunalan/drake.git
cd drake
python3 -m venv venv && source venv/bin/activate
pip install drake manipulation jupyter --extra-index-url https://drake-packages.csail.mit.edu/whl/nightly/
```

## Run Simulation

```bash
jupyter notebook chap_3_basic_pick_n_place/example_3_11_pick_ur5e.ipynb
```

## Run on Hardware

```bash
pip install ur-rtde
```

Then in notebook, replace simulation driver with:
```python
from ur5e_description.ur5e_driver import UR5eDriver
driver = builder.AddSystem(UR5eDriver(robot_ip="192.168.1.102"))
```
