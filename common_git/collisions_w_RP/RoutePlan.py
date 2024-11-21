from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.state import InitialState
from commonroad.geometry.shape import Circle, Rectangle
from commonroad.common.util import Interval
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad.visualization.mp_renderer import MPRenderer
import numpy as np
import matplotlib.pyplot as plt
import numpy as np
from scipy.interpolate import splprep, splev





vehicle_shape = Rectangle(length=4.5, width=2.0)  
collisionvertex = {'x': -24.3, 'y': -2.7}
A = {'x': -27.1, 'y': 30}
B = {'x': -47.7, 'y': -7}
steps = 40
speed = steps/4



# First we'll pqLoad Lanelet Network from XML
scenario_file = "Commonroad/USA_a1-1_1_T-1.xml"
scenario,_= CommonRoadFileReader(scenario_file).open()


#Define Two Vehicles with Starting Points and Goal Regions
planning_problem_set = PlanningProblemSet()

# Vehicle 1
start_state_1 = InitialState(
    time_step=0,
    position=np.array([-27.1, 30.0]),
    velocity=5.0,
    orientation=0.0,  # Heading angle in radians
    yaw_rate=0.0,  # Rotational velocity
    slip_angle=0.0  # Slip angle
)
goal_state_1 = InitialState(
    time_step=Interval(50, 50),
    position=Circle(radius=1.0, center=np.array([-24.3, -2.7])),
    velocity=Interval(4.0, 6.0)
)
goal_region_1 = GoalRegion([goal_state_1])
planning_problem_1 = PlanningProblem(1, initial_state=start_state_1, goal_region=goal_region_1)

# Vehicle 2
start_state_2 = InitialState(
    time_step=0,
    position=np.array([-47.7, -7.0]),
    velocity=7.0,
    orientation=0.0,  
    yaw_rate=0.0,  
    slip_angle=0.0 
)
goal_state_2 = InitialState(
    time_step=Interval(50, 50),
    position=Circle(radius=1.0, center=np.array([-24.3, -2.7])),
    velocity=Interval(6.0, 8.0)
)
goal_region_2 = GoalRegion([goal_state_2])
planning_problem_2 = PlanningProblem(2, initial_state=start_state_2, goal_region=goal_region_2)


planning_problem_set.add_planning_problem(planning_problem_1)
planning_problem_set.add_planning_problem(planning_problem_2)

#then use thois to plan the routes
routes = []
trajectories = []
for planning_problem in planning_problem_set.planning_problem_dict.values():
    route_planner = RoutePlanner(scenario.lanelet_network, planning_problem)
    candidate_holder = route_planner.plan_routes()
    print(len(candidate_holder))
    route = candidate_holder.pop()
    routes.append(route)



#then lets analyze one case 
lanelets_vehicle_1 = routes[0].lanelet_ids
lanelets_vehicle_2 = routes[1].lanelet_ids

print(lanelets_vehicle_1)

centerline_points1 = []
centerline_points2 = []

# Iterate through each lanelet ID
for lanelet_id in lanelets_vehicle_1:
    
    lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
    if lanelet is not None:
        # append all centerpoints of the lanelet to the list
        centerline_points1.extend(lanelet.center_vertices)
    else:
        print(f"Lanelet with ID {lanelet_id} not found in the scenario.")

# Convert the list to a NumPy array for further processing

for lanelet_id in lanelets_vehicle_2:

    lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
    if lanelet is not None:

        centerline_points2.extend(lanelet.center_vertices)
    else:
        print(f"Lanelet with ID {lanelet_id} not found in the scenario.")

centerline_points1 = np.array(centerline_points1)
centerline_points2 = np.array(centerline_points2)

print(centerline_points1,centerline_points2)


# Some problems here: The centerpoints dont give us equal steps wwe will have to use linspace
#  to add npoints based on our step size and sitace which is another layer of code. 
# We also have no way to add these objects as of yet, for this we will need to use the rective palnner which will be our next step

fig, ax = plt.subplots(figsize=(10, 10))
rnd = MPRenderer()
scenario.draw(rnd)
rnd.render()
ax.set_xlim([-50, 50])
ax.set_ylim([-50, 50])

plt.show()
