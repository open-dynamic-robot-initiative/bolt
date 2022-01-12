# bolt

## What it is

Drivers of the bolt robot.

## Getting started:

dependance on pip packages 

```
python3 -m pip install xacro m2r sphinxcontrib.moderncmakedomain sphinx breathe pybullet
```
dependance on sources

```
cd $HOME
mkdir devel
cd devel
git clone git@github.com:machines-in-motion/treep_machines_in_motion.git
treep --clone BOLT
```

After these command you should have a workspace folder containing a src folder containing the bolt package and all it's dependencies.

You can use `colcon build` to compile all.

## Authors

- Maximilien Naveau
- Julian Viereck
- Majid Khadiv
- Elham Daneshmand 

## Copyrights

Copyright(c) 2018-2019 Max Planck Gesellschaft, New York University

## License

BSD 3-Clause License

## Max-Planck Institute: Test defective master_board

- master board V1
  - SPI 0: ok
  - SPI 1: ok
  - SPI 2: ok
  - SPI 3: ok
  - SPI 4: ok
  - SPI 5: ok

- master board V2
  - SPI 0: not ok
  - SPI 1: not ok
  - SPI 2: not ok
  - SPI 3: not ok
  - SPI 4: not ok
  - SPI 5: not ok

