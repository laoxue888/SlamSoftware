
# 前言

GraphExecuter运行环境配置

# 运行环境

```shell
# 安装Ananconda 
cd /root/workspace/downloads
wget https://repo.anaconda.com/archive/Anaconda3-2025.06-0-Linux-x86_64.sh

chmod +x Anaconda3-2025.06-0-Linux-x86_64.sh
bash Anaconda3-2025.06-0-Linux-x86_64.sh

# 重新打开终端，创建虚拟环境
conda create -n visual_slam python=3.10 -y # ros2 humble用的是python3.10，ros2 jazzy用的是3.12

# 切换到新创建的虚拟环境
conda activate visual_slam

# 安装jupyter notebook
conda install jupyter notebook -y
# conda install jupyter_contrib_nbextensions -y

# 安装C++ kernel
conda install xeus-cling -c conda-forge -y

# 检查是否成功安装了kernel
jupyter kernelspec list

conda install -c conda-forge libstdcxx-ng -y

cd /root/workspace/GraphExecuter/graph_executer
pip install -r requirements_moveit2_yolobb_ws.txt

cd /root/workspace/downloads
git clone https://github.com/laoxue888/NodeGraphQt.git
cd /root/workspace/downloads/NodeGraphQt
pip install -e . 

pip install paho-mqtt
```