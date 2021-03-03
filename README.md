

# carla-utils

## 0. Requirements
```
- CARLA 0.9.9.4, python 3.7
- pytorch
- (optional) ROS
```

## 1. setup

```bash
echo "export CARLAPATH=/your/carla/server/path" >> ~/.bashrc
```

```bash
pip install carla-utils
```
or

```bash
git clone https://github.com/zhangdongkun98/carla-utils.git
cd carla-utils
pip install -e .
```

## 2. Usage

```bash
cd /your/path/to/carla
./CarlaUE4.sh
cd PythonAPI/util/ && ./config.py

<==>

python -m carla_utils.run
python -m carla_utils.config
```

## others
	config/sensor:
		sensor_dict: 使用 image_size_x, image_size_y， 去掉img_length，img_width，与carla的blueprint一致




	- system
	- basic
	
	- augment
	    - tools.py
	
	- sensor
	- world_map
	
	- agents
	- utils   # contains class easy to use
	
	- benchmark ?
	- ros