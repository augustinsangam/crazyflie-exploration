# SGBA

This repo implements the internal state machine of the drone. It also implements the Swarm Gradient Bug Algorithm as described [here](https://robotics.sciencemag.org/content/4/35/eaaw9710).

This repo is intended to be used with an abstraction layer called `porting`. This layer implements common functionnalities to all drone types (either ARGoS or Crazyflie).

The code is written in modern C++ and follows strict guidelines.

This implementation is heavily based on the reference implementation done by the authors of the paper and can be found [here](https://github.com/tudelft/SGBA_CF2_App_layer).

## Usage

To use this repo, you have to follow this CMake scheme from your own repo:

```cmake
set(EXPLORATION_DEP ON)
set(EXPLORATION_METHOD 1)
set(EXPLORATION_DRONE_INITIALISATION_DELAY 3000)
add_directory("./exploration")
```

The above example will use the first exploration method with a starting delay of 3s.

You can either choose to copy the content to your repo or use it as a submodule.
