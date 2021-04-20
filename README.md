# SGBA

This repo implements the internal state machine of the drone. It also implements the Swarm Gradient Bug Algorithm as described [here](https://robotics.sciencemag.org/content/4/35/eaaw9710).

This repo is intended to be used with an abstraction layer called `porting`. This layer implements common functionnalities to all drone types (either ARGoS or Crazyflie).

The code is written in modern C++ and follows strict guidelines.
