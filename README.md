# Bratmobile V4

This is work in progress towards true mapless navigation using insights from neuroscience and psychology:

- Elizabeth S. Spelke and Katherine D. Kinzler. Core knowledge. Developmental Science,
10:89–96, 1 2007. ISSN 1363755X. doi: 10.1111/J.1467-7687.2007.00569.X.
- [Retrosplenial cortex (Auger & Maguire, 2018](https://www.jneurosci.org/content/jneuro/38/6/1472.full.pdf)

The navigation problem is broken down into attraction and avoidance Braitenberg behaviours, called Tasks. A supervising module, called the Configurator, can simulate sequences of Tasks to build up a plan and represents the robot's 

## Features:
* Hybrid state-space representation

* Mapless navigation

* Plans are made up of multiple Tasks, not trajectories

* Pure event driven processing where the tasks receive sensor events and emit both motor actions and termination events to hand over to the next task!

## Hardware

[Zetabot](https://github.com/berndporr/zetabot))

## Software
### Development packages

```
sudo apt install g++ cmake libopencv-dev libboost-all-dev xorg-dev libglu1-mesa-dev libgtest-dev xauth x11-apps xfonts-base
```

### Install powersave service

The rpi5 draws too much current under load to run off the battery so we need to enable powersave:

sudo cp powersave.service /etc/systemd/system
sudo systemctl enable --now powersave.service


## Build

```
git checkout brat4target
cd brat4target
cmake .
make
```

## Credits
 - Bernd Porr
 - Giulia Lafratta
