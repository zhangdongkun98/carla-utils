# carla-utils


## Clone this repo
```bash
git clone https://github.com/zhangdongkun98/carla-utils.git
cd carla-utils
```


## Requirements
```
- CARLA (0.9.9 / 0.9.10 / 0.9.11)
- python 3.7
- pytorch
- (optional) ROS
```

```bash
sudo apt-get install -y libjpeg-dev libtiff5-dev
### for ubuntu 16.04
conda install -c anaconda libpng

### for CARLA 0.9.10
conda install xerces-c==3.2.0

### for CARLA 0.9.12
sudo apt install libomp5

### for CARLA 0.9.13
sudo apt install vulkan-utils
```

## Installation


```bash
pip install -r requirements.txt
echo "export CARLA_ROOT=/your/carla/server/path" >> ~/.bashrc
pip install -e .
```


## Usage

```bash
cd $CARLA_ROOT
./CarlaUE4.sh
cd PythonAPI/util/ && ./config.py

<==>

python -m carla_utils.run
python -m carla_utils.config

### visualize via matplotlib, open3d, ROS, and carlaviz
python -m carla_utils.viz.plt
python -m carla_utils.viz.open3d
python -m carla_utils.viz.ros
python -m carla_utils.viz.carlaviz
```


## demo

```bash
cd scripts
python run_env.py --render
```

