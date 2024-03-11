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



