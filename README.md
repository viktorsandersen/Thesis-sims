# Installation
Install mujoco
```
pip install mujoco
pip install mediapy
```

download model from github and copy over the UR5e
```
git clone git@github.com:google-deepmind/mujoco_menagerie.git
```


system dependencies
```
sudo apt install tree
sudo apt install ffmpeg
```

# Scipy version
Newest version of scipy is not compatible with the robotics toolbox, so we have to downgrade to the previous version
```pip install scipy==1.11.0```

# Problems with matplotlib
```bash
pip uninstall matplotlib
sudo apt-get remove python3-matplotlib
pip install matplotlib```
Test
# Thesis-sims
