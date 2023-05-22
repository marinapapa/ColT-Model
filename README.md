# *ColT Model - Collective turning in bird flocks*  

A two-dimensional agent-based model of flocking under predation, developped to investigate the dynamics of collective turning. First presented in the paper: 

*Papadopoulou M, Hildenbrandt H, Hemelrijk CK (2023)  Diffusion during collective turns in bird flocks under predation.*

## Prerequisites

### Windows
* Operating system: Windows 10.
* Graphic card with support for OpenGL 4.4 or later.

### Linux 

To build the software under Linux (Debian packet system):
```bash
~$ sudo apt install libtbb-dev
~$ sudo apt install libglm-dev
~$ cd ColT
~/ColT$ make
~/ColT$ make install   # creates the excecutable ./bin/Release/starlings
```

# The model

![alt text](https://github.com/marinapapa/HoPE-model/blob/main/simulation_screenshot.PNG)

## _Framework_ 

This model is based on self-organization and includes bird-like and predator-like agents. Agents interact with their surrounding neighbors based on the rules of attraction, alignment and avoidance. Prey-agents flock together and avoid the predator. Predator-agents chase and attack prey-agents. Catches of prey are not modelled. Each individual behaves according to the 'state' it is in, for instance normal flocking or escaping. Each state consists of a collection of specific actions that an agent follows, for instance alignment and attraction. 

The model consists of 3 timelines: action, update, and integration. At each action time step, prey agents may switch to a different state for a specific duration. While being in a state, an agent collects and updates its information about its environment at each reaction time step. At the beginning of a reaction step, an agent calculates an aimed position and heading. Between 2 reaction steps, individuals move from their current positions and headings towards their aimed ones. This happens gradually across several [dt] integration time steps. Thus, an action step may include several reaction steps, and a reaction step includes several integration steps. Some states, such as normal flocking, are transient: an action step includes one reaction step; the agent might switch state at each reaction time step. Other states, such as an escape maneuver, are persistent: an action step includes several reaction steps. Their duration is controlled through the parameter file. 

The switch between states depends on a transition matrix that gives a probability of each agent to switch state given its current state and its distance to predator (reflected on a [stress] value, unique for each prey agent). The closer the predator is, the highest the value of stress.

## _Parameters_
All user-defined parameters are parsed by combining a series of .json files: *config.json* (simulation parameters),  *starling.json* (prey parameters, starling used as an example) and *predator.json* (predator parameters). Distance is measured in meters [m], time in seconds [s] and angles in degrees [deg].

## _Individual Actions_

Actions are the basic elements controlling the movement of each agent in the simulations. Each action represents a steering vector so that the weighted sum of all actions controls the agent's motion. Each action has each own user-defined parameters. Multiple actions are combined to create *states*. The majority of actions control the interactions between agents (coordination between prey-agents, escape actions of prey-agents from the predator-agents, and hunting actions of the predator-agents towards prey-agents). The model is based on **topological** interactions.

Prey-agents actions:
* Avoid actions: 
    * __avoid_n_position__: the individual turns away from the position of its _topo_ closest neighbors if they are in distance smaller than _minsep_. Parameters: _topo_ (number of neighbors to take interact with), _fov_ (field of view), _maxdist_, _minsep_, _w_.
  
* Align actions:
    * __align_n__: the individual turns towards the average heading of its _topo_ closest neighbors. Parameters: _topo_ (number of neighbors to take interact with), _fov_ (field of view), _maxdist_, _w_.

* Cohere actions:
    * __cohere_centroid_distance__: the individual turns towards the center of the positions of its _topo_ closest neighbors. The strength (w) of this action is scaled on the distance between the focal individual and the center of its neighbors. Parameters: _topo_ (number of neighbors to take interact with), _fov_ (field of view), _maxdist_, _w_.

* Non-interacting actions:
    * __wiggle__: the individuals turn by a random angle controlled by the weight of this steering force that is perpendicular to the agent's heading, sampled from the range [-w,w]_. Parameters: _w_.

* Turning/Escape actions:
    * __relative_roosting_persistant__: individuals are attracted towards a point defined by a distance and a direction relative to their position when they enter the state that includes this action.  Parameters: _home_dist_, _home_direction_, _w_.
    * __random_t_turn_gamma_pred__: individuals perform a turn with angular velocity defined by a random angle and duration, both sampled by a gamma distribution. Parameters: _turn_mean_, _turn_sd_, _time_mean_, _time_sd_.
    * __copy_escape__: if one of the _topo_ closest neighbours of an agent is in an escape state, the agent enters the same state at the next reaction step. Parameters: _topo_, _fov_, _max_dist_.

Predator-agents actions:
* Avoid actions: 
    * __avoid_closest_prey__: the predator turns away from the position of its closest prey. Used in states where the predator should not hunt the prey. Parameters: _w_.
    * __set_retreat__: the predator is repositioned at a given distance away from the flock and given a new speed. Parameters: _distAway_, _speed_.
   
* Non-interacting actions:
    * __wiggle__: the individuals turn by a random angle controlled by the weight of this perpendicular to the agent' heading steering force, sampled from the range [-w,w]_. Parameters: _w_.
    * __hold_current__: the agents tries to hold a constant position. Parameters: _w_.
    
* Hunting actions:
    * __select_flock__: the predator chooses a flock as its target. Selection can be made based on the flock's size or proximity. Parameters: _selection_.
    * __shadowing__: the predator follows (or tries to follow) its target flock from a given angle and distance, keeping a constant speed that scales from the speed of its target. Parameters: _bearing_ (angle starting from the flock's heading), _distance_, _placement_ (whether to automatically reposition the predator to the given shadowing position), _prey_speed_scale_, _w_.
    *  __chase_closest_prey__: the predator turns towards the closest starling-agent at every time point (target) and moves with a speed that scales from this agent's speed. Parameters: _prey_speed_scale_, _w_. 
   
    
_Note_: The model is currently set to simulate roosting turns (referred to as circular evasion in the paper). 

## _Individual States_

States in the model are defined as combinations of actions. *Persistent* states have a user-defined duration, whereas *Transient* states can change after a time-step. The transition between states is controlled by the user-defined transition matrix. Prey-agents start from a normal flocking state, and as their stress increases, they switch to an escape state. After that, they get into a persistant flocking state that works as escape penalty. 

The hunting strategy of the predator is built on a chain of persistent states that makes the predator behave deterministically: follow the flock from a distance, attack, retreat and start over.

## _Initialization_

The initial conditions of the agents are controlled by the user. Prey agents are initiated in a flock formation, within a circle and with similar headings.

### __Application keys:__

1. PgUp: speed-up simulation
2. PgDown: slow-down simulation
3. Space: pause/continue simulation
4. Right Arrow: run 1 simulation step
5. A: darkens background
6. T: shows/hides the position trail of each prey-agent
7. Shift+T: shows/hides the position trail of each predator-agent
8. K: kills or revives predator-agents
9. 1: applies colormap of id of prey-agents
10. 2: applies colormap of speed of prey-agents
11. 4: applies colormap of state of prey-agents
12. 5: applies colormap of flock id of prey-agents (which flock each individual belongs to)
13. 6: colors the prey-agent that is the target of a predator
14. Shift+1: applies colormap of id of predator-agents
15. Shift+2: applies colormap of state of predator-agents

## _Data Collection_

The model exports data in _.csv_ format. It creates a unique folder within the user-defined data_folder (in the config.json), in which it saves a single .csv file for each Observer, as defined in the config file. Sampling frequency and output name of each files are also controled by the config. The config is also copied to the saving directory. 

In its current state, the model exports (1) timeseries of positions, heading, speed etc for each agent, (2) diffusion-related metrics. 

## Authors
* **Dr. Marina Papadopoulou** - Contact at: <m.papadopoulou.rug@gmail.com>
* **Dr. Hanno Hildenbrandt** 