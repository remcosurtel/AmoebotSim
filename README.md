# AmoebotSim

[![Documentation Status](https://readthedocs.org/projects/amoebotsim/badge/?version=latest)](https://amoebotsim.readthedocs.io/en/latest/?badge=latest)

Welcome to AmoebotSim, a visual simulator for the [amoebot model](link-todo) developed by the [Self-Organizing Particle Systems (SOPS) Lab](https://sops.engineering.asu.edu/) at Arizona State University and the University of Paderborn. The amoebot model is an abstraction of _programmable matter_, a material that can dynamically change its physical properties (e.g., shape, size, color, etc.) based on user input or stimuli from its environment. This simulator is designed for researchers who want to:

- Visualize and learn about current distributed algorithms for the amoebot model
- Experiment with new ideas for amoebot model algorithms
- Perform validation and runtime testing for new or existing amoebot model algorithms

This README covers some basic information, but you'll find much more in [our documentation](https://amoebotsim.readthedocs.io/).


## Getting Started

You're an *explorer* 🔎 if you don't necessarily have a lot of programming experience but would love to see some particle systems in action. If this describes you, download the latest release of AmoebotSim [here](https://github.com/SOPSLab/AmoebotSim/releases). Then, read our [Usage Guide](https://amoebotsim.readthedocs.io/en/latest/usage/usage.html) for instructions on navigating the AmoebotSim environment.

You're a *researcher* 🧪 if you want to implement your own algorithm simulations in AmoebotSim and use it to capture performance statistics. If this describes you, read our [Installation Guide](https://amoebotsim.readthedocs.io/en/latest/install/install.html#for-researchers-and-developers) for instructions on cloning the most recent stable codebase, installing the development environment, and running the project. Then, read the [Tutorials](https://amoebotsim.readthedocs.io/en/latest/tutorials/tutorials.html) for a walkthrough of AmoebotSim algorithms and examples.

You're a *developer* 💻 if you're a researcher (as above) who wants to add your algorithms to the public, canonical version of AmoebotSim, or if you're interested in contributing to and maintaining AmoebotSim as an open source project. If this describes you, read our [Installation Guide](https://amoebotsim.readthedocs.io/en/latest/install/install.html#for-researchers-and-developers) for instructions on cloning the most recent stable codebase, installing the development environment, and running the project. If you're new to AmoebotSim development, read the [Tutorials](https://amoebotsim.readthedocs.io/en/latest/tutorials/tutorials.html) for a walkthrough of AmoebotSim algorithms and examples. All development best practices and guidelines for contributions are discussed in our [Development Guide](https://amoebotsim.readthedocs.io/en/latest/development/development.html).


## Acknowledgements

AmoebotSim was originally created by [Robert Gmyr](https://gmyr.net/) during his PhD studies at the University of Paderborn, and is now actively maintained by [Joshua J. Daymude](https://github.com/jdaymude) and [Kristian Hinnenthal](link-todo>), current PhD students in the SOPS Lab. Many other hands have helped (and are currently helping) build AmoebotSim. You can find a list of past and present contributors [here](https://amoebotsim.readthedocs.io/en/latest/index.html#acknowledgements) and a directory of our lab members [here](https://sops.engineering.asu.edu/sops/).


## Licensing

AmoebotSim is licensed under the [GNU General Public License v3.0](https://choosealicense.com/licenses/gpl-3.0/) in our attempts to keep research open.
You're welcome to do pretty much anything you'd like with our code, but you cannot distribute a closed source version commercially and you must keep all copyright and license notices intact.

> AmoebotSim: a visual simulator for the amoebot model of programmable matter.
> Copyright (C) 2020 Joshua J. Daymude, Robert Gmyr, and Kristian Hinnenthal.
> Please direct all questions and communications to sopslab@asu.edu.
>
> This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or (at your option) any later version.
>
> This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
>
> You should have received a copy of the GNU General Public License along with this program. If not, see <https://www.gnu.org/licenses/>.
>
> AmoebotSim is developed using Open Source Qt.


## Contact Us

If you can't find the answers you need in the [documentation](https://amoebotsim.readthedocs.io/en/latest/index.html#), feel free to open a [new issue](https://github.com/SOPSLab/AmoebotSim/issues/new/choose>) using the question template or email us at sopslab@asu.edu (whichever is more appropriate for your question).


## Changes

Here changes in [this forked repository](https://github.com/remcosurtel/AmoebotSim) compared to [the original](https://github.com/SOPSLab/AmoebotSim) are listed as completely as possible. They are as follows:

- [alg/leaderelection_erosion.h](https://github.com/remcosurtel/AmoebotSim/blob/dev/alg/leaderelection_erosion.h) and [alg/leaderelection_erosion.cpp](https://github.com/remcosurtel/AmoebotSim/blob/dev/alg/leaderelection_erosion.cpp) implement the [erosion algorithm](https://arxiv.org/abs/1705.03538) for leader election.
- [alg/leaderelection_stationary_deterministic.h](https://github.com/remcosurtel/AmoebotSim/blob/dev/alg/leaderelection_stationary_deterministic.h) and [alg/leaderelection_stationary_deterministic.cpp](https://github.com/remcosurtel/AmoebotSim/blob/dev/alg/leaderelection_stationary_deterministic.cpp) implement the [stationary deterministic algorithm](https://link.springer.com/chapter/10.1007/978-3-030-34992-9_3) for leader election. For this algorithm, [core/particle.h](https://github.com/remcosurtel/AmoebotSim/blob/dev/core/particle.h), [core/particle.cpp](https://github.com/remcosurtel/AmoebotSim/blob/dev/core/particle.cpp), [ui/visitem.cpp](https://github.com/remcosurtel/AmoebotSim/blob/dev/ui/visitem.cpp), and [res/textures/particle.png](https://github.com/remcosurtel/AmoebotSim/blob/dev/res/textures/particle.png) have been updated to support the drawing of virtual nodes between particles.
- [alg/leaderelection_scontraction.h](https://github.com/remcosurtel/AmoebotSim/blob/dev/alg/leaderelection_scontraction.h) and [alg/leaderelection_scontraction.cpp](https://github.com/remcosurtel/AmoebotSim/blob/dev/alg/leaderelection_scontraction.cpp) implement the [S-contraction algorithm](https://arxiv.org/abs/1807.10461) for leader election.
- [alg/leaderelection_deterministic.h](https://github.com/remcosurtel/AmoebotSim/blob/dev/alg/leaderelection_deterministic.h) and [alg/leaderelection_deterministic.cpp](https://github.com/remcosurtel/AmoebotSim/blob/dev/alg/leaderelection_deterministic.cpp) contain an unfinished implementation of the [deterministic algorithm](https://arxiv.org/abs/1905.00580) for leader election.
- [alg/leaderelection.h](https://github.com/remcosurtel/AmoebotSim/blob/dev/alg/leaderelection.h) and [alg/leaderelection.cpp](https://github.com/remcosurtel/AmoebotSim/blob/dev/alg/leaderelection.cpp) are updated to fix an error which could cause the final two candidates on a boundary to revoke their candidacy simultaneously, leading to a simulation that would never terminate and fail to elect a leader.
- [core/simulator.h](https://github.com/remcosurtel/AmoebotSim/blob/dev/core/simulator.h), [core/simulator.cpp](https://github.com/remcosurtel/AmoebotSim/blob/dev/core/simulator.cpp), [core/system.cpp](https://github.com/remcosurtel/AmoebotSim/blob/dev/core/system.cpp), [main/application.cpp](https://github.com/remcosurtel/AmoebotSim/blob/dev/main/application.cpp), [ui/algorithm.h](https://github.com/remcosurtel/AmoebotSim/blob/dev/ui/algorithm.h), [ui/algorithm.cpp](https://github.com/remcosurtel/AmoebotSim/blob/dev/ui/algorithm.cpp), [ui/parameterlistmodel.h](https://github.com/remcosurtel/AmoebotSim/blob/dev/ui/parameterlistmodel.h), and [ui/parameterlistmodel.cpp](https://github.com/remcosurtel/AmoebotSim/blob/dev/ui/parameterlistmodel.cpp), as well as all aforementioned leader election algorithms are updated to support the saving and loading of particle systems to and from text files.
- [core/amoebotsystem.h](https://github.com/remcosurtel/AmoebotSim/blob/dev/core/amoebotsystem.h) and [core/amoebotsystem.cpp](https://github.com/remcosurtel/AmoebotSim/blob/dev/core/amoebotsystem.cpp) are updated with a random permutation scheduler, which activates all particles exactly once per round in a random order. This is an optional alternative to the pre-existing random scheduler, which simply activates a randomly chosen particle at each iteration.