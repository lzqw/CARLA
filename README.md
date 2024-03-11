# Set up CARLA simulator and program
```bash
git clone https://github.com/lzqw/CARLA.git
```
```bash
cd CARLA
```
- Download CARLA simulator (若使用服务器则不需要本地装carla)
```bash
wget https://leaderboard-public-contents.s3.us-west-2.amazonaws.com/CARLA_Leaderboard_2.0.tar.xz
```
- Unzip in the same folder with CARLA
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

