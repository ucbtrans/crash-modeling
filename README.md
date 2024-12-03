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

