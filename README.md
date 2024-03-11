# Set up CARLA simulator and program
```bash
git clone https://github.com/lzqw/CARLA.git
```

## Install python environment and dependency
```bash
cd CARLA
```
- Download CARLA simulator
```bash
wget https://leaderboard-public-contents.s3.us-west-2.amazonaws.com/CARLA_Leaderboard_2.0.tar.xz
```
- Unzip in the same folder with CARLA
```bash
tar -xf CARLA_Leaderboard_2.0.tar.xz
```
```bash
conda create -n carlapy python=3.7
```
```bash
conda activate carlapy
```
- Install dependency of CARLA simulator
```bash
cd CARLA_Leaderboard_2.0  
pip3 install -r PythonAPI/carla/requirements.txt
```
-Install the required Python dependencies for CARLA leaderboard
```bash
cd ..
cd leaderboard  
pip3 install -r requirements.txt
```
-Install the required Python dependencies for Scenario_Runner
```bash
cd ..
cd scenario_runner  
pip3 install -r requirements.txt
```


