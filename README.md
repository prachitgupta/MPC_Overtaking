# Project Title

Brief description of the project.

## Table of Contents

- [Project Title](#Infusing HJI Reachability based safety within MPC for overtaking)
  - [Table of Contents](#table-of-contents)
  - [About](#about)
  - [Getting Started](#getting-started)
    - [Prerequisites](#plotting)
    - [Installation](#installation)
  - [Usage](#usage)
  - [Contributing](#Todo)
  - [License](#Implementation issues)
  - [Acknowledgements](#comments)

## About

- status = completely avoid target state but not a practical maneveur (halt still for lead vehicle to proceed)
- works only for certain velocities and initial conditions not robust
- mostly due to small region of computed values
## Getting Started

- to run a basic simulation
- clone this repo
- python3 MPC_HJI.casadi.py


### edit animation and plotting

Class defined in draw.py


### Installation
pip3 install casadi
pip3 install pickle

## Usage
- MPC_KBM = simple 4d kbm implemented in casadi to reach goal
- MPC_HJI_KBM = slip angle KBM integrated with HJI using precomputed value function
- get_lookup  =  generate lookup table as a bspline function of relative state and value
- get_grid = simple grid mimicing grid used for value function computation
- human.py test out constant v human motion
- eval_u matlab data.mat conversion to required format

## ToDO
- generate lookup table for grid more than [13,13,9,9,9] for better results
- compute valufunction for larger horizons [-50,50]
- try using higher and more practical values of speed for human and robot
- improving penality term for value in cost
- setting value function tolerance
- determine best prediction horizon

- determining most practical final state xs and required accuracy level for computations

## Implementation issue
while computation both xrel and value is a function of casadi symbols, hence not possible to evaluate value of expresssin if(xrel > -20 and xrel < 20) where [-20,20] is computation horizon for value function , must find some other way to ensure the same (currently final state itself within computation horizon) - causing state to not stop at x_final as constraint value > 1 might be false at xs

## comments
- commented MPC_HJI_casadi explaining purpose of every line, refer that to make required changes
- ignore MPC_KBM_rel for now

current data13.mat
- grid_size = [13,13,9,9,9]
- grid_min = [-20,-5,-pi,0,0]
- grid_max = [20,5,pi,20,20]
- generate valuefunction using my_brs.m and save("data13.mat) which is fedd in get_lookup 
- current implementation (entire state has to be confined withing horizon computed by helperOc)
- can't constraint xrel as of now

