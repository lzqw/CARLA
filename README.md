# Set up CARLA simulator and program
```bash
git clone https://github.com/lzqw/CARLA.git
```
```bash
cd CARLA
```
- Download CARLA simulator (若使用服务器则不需要本地装carla)
- 链接: https://pan.baidu.com/s/1f0NZF5mLkmklIPJp4vry9Q 提取码: cqqp 
--来自百度网盘超级会员v6的分享
```bash
wget https://leaderboard-public-contents.s3.us-west-2.amazonaws.com/CARLA_Leaderboard_2.0.tar.xz
```
```bash
tar -xf CARLA_Leaderboard_2.0.tar.xz
```

- Install python dependency
```bash
conda create -n carlapy python=3.7
```
```bash
conda activate carlapy
```
```bash
pip3 install -r requirements.txt
```
# Run simulator 
- Run locally:
```bash
cd {PATH TO CARLA simulator}
```
```bash
./CarlaUE4.sh -quality-level=Low -world-port=2000 #quality:Low or Epic
```
```bash
cd {PATH TO CODE}
```
```bash
python run_perception.py --host 127.0.0.1
```
- Run on server:
```bash
cd /opt/CARLA_Leaderboard_20 #server
```
```bash
./CarlaUE4.sh -carla-rpc-port=2000 -RenderOffScreen -graphicsadaper=1 #server
```
```bash
cd {PATH TO CODE} #local
```
```bash
python run_perception.py --host {IP OF SERVER} #local
```
- /CODE/agent/test_agent/agent_config.txt 中设置手动或自动 

- 在/CODE/agent/test_agent/agent.py中修改程序，run_step函数可获取周围物体的bounding box信息，并最终给出控制量。


