# crash-modeling

This repository contians the code for all of our efforts to simulate and analyze the data within the TIMS database. We start with basic prgression of mnually adding lanelets and using 
commonorad tolls to pprogrssing to a nmore and more automated level where we can eventually tranform the entire dataabse with ease. - Pranav

Explanation of the code structure and files: 



collisions_w_RP - Folder that contains the simulations with RP (reactive/route planner)
                - As of right now contians the file that takes in a input scenrrio finds planning problems out puts the set of routes. 
                - Will contain the implemented Reactive planner code also.
collisions_wo_RP - Folder that contains the simulations
                 - Collision1: 2 parties, 2 cars, Kittredge shattuck
                 - Collision2: 2 parties, 2 cars, Kittredge shattuck (With splicing and OSM map)
                 - Collision3: 3 parties 1 loryy 2 cars, intersection of b88 and Univeristy avenue (With splicing and OSM map)
                 - Collision4: 2 Parties a bike and car, Kittredge and Mlivia (With splicing and OSM map)
                 
osm Files - This is where our files we downlaod or epxort form OSM are saved.
scenario_files - Where our maps of lanenelts are saved after they have been converted using the OSMtoCR tool
Full Pipeline.py - The file i am integrating the whole process (Work in progress)
collauto.py - This file extracts osm map and saves the commonroad conversion as an xml file
shifter.py - This is the file that shifts the XML file to be centered around our vertex of collision (has some bugs)

IMPORTANT: Due to a slightly different file structure i may need to change certian defentions to acocunt for this 
          - There is a problem int he reactive planner defneition that tries to import non-existent libaries some errors fixed and will update in form and add the fixed 
           version of RP to the repo also.


Current Dependancies: 

commonroad-io
commonroad-vehicle-models
commonroad-visualization
commonroad-scenario_designer
commonroad-route_planner
commonroad-reactive_planner
numpy
scipy
matplotlib
pandas
osmnx
shapely
pybullet
networkx
transformers
torch
pillow
flake8
black

___________
Code review:
- Full pipeline seems to be work in progess code? only header so far
- I did not look at collisions_wo_RP in detail since this seems to be the initial code that is fairly handcrafted and we will only use to validate the more automated pipeline build at the moment
- shifter.py -- this functionality could have been achieved with the (https://cps.pages.gitlab.lrz.de/commonroad/commonroad-io/api/scenario.html#commonroad.scenario.scenario.Scenario.translate_rotate)[scenario.translate_rotate method]
- collauto.py
  * The main function should take the geo location as input and not street names - you could define a enum for default locations like Shattuck/Kittredge
  * You could fill [all fields of the scenario](https://github.com/ucbtrans/crash-modeling/blob/main/common_git/collauto.py#L86) meaningfully - Name etc is missing
- RoutePlan.py
  * Initial state and goal should be determined more automatically based on the crash report
  * Steps after getting route:

    
(1) It should be irrelevant if the reference (center) points are equidistant for reactive planner - if not there should be a functionality in the dirvability-checker package that smooths them - ?For sure that makes things a litle easier?

(2) To ensure a collision, I would calculate the distance from initial state to goal: use the [curvilinear coordinate system](https://commonroad.in.tum.de/tutorials/drivability-checker-curvilinear-coordinate-system)for that - 1. convert points to curvilinear coordinate systems - 2. convert initial and goal state to curvilinear coordinate system to get (s,d) coordinates for both 3. s_diff = s_goal - s_init is the distance ?For sure?

(3) Then, set the desired and initial velocity based on pre-defined time to collision, e.g., v = s_diff / t_predef ?For sure, what is T_predef in this situatlion?


(4) Use the reactive planner to generate the trajectories similar to this (taken from [this tutorial](https://commonroad.in.tum.de/tutorials/commonroad-reactive-planner-getting-started)): ?In progress geeting Rp to work on our routes from Route planner?


```python
from commonroad_rp.reactive_planner import ReactivePlanner
planner = ReactivePlanner(config)
planner.set_reference_path(route.reference_path)
planner.record_state_and_input(planner.x_0)
planner.set_desired_velocity(current_speed=planner.x_0.velocity)
optimal = planner.plan()
planned_traj_cart = optimal[0]

```
