from setuptools import setup, find_packages

setup(
    name='carla-utils',
    packages=find_packages(),
    version='0.0.2',
    author='Zhang Dongkun, Wang Yunkai',
    author_email='zhangdongkun98@gmail.com, wangyunkai.zju@gmail.com',
    url='https://github.com/zhangdongkun98/carla-utils',
    description='carla',
    install_requires=[
        'numpy',
        'scipy',
        'matplotlib',
        'networkx',
        'pygame==1.9.6',
        'PyYAML',
        'python-intervals',
        'opencv-python',
        'open3d-python==0.7.0.0'
    ],

    include_package_data=True
)
