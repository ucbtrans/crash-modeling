import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad_route_planner.route_planner import RoutePlanner
from commonroad.scenario.state import KSState
from commonroad.scenario.obstacle import DynamicObstacle, ObstacleType
from commonroad.scenario.trajectory import Trajectory
from commonroad.visualization.mp_renderer import MPRenderer
from commonroad_rp.reactive_planner import ReactivePlanner
from commonroad_rp.utility.config import ReactivePlannerConfiguration
from commonroad.geometry.shape import Rectangle
from commonroad.common.file_reader import CommonRoadFileReader
from commonroad.planning.planning_problem import PlanningProblem, PlanningProblemSet
from commonroad.planning.goal import GoalRegion
from commonroad.scenario.state import InitialState
from commonroad.geometry.shape import Circle
from commonroad.common.util import Interval, AngleInterval
from commonroad_rp.utility.utils_coordinate_system import CurvilinearCoordinateSystem
import commonroad_route_planner.fast_api.fast_api as fast_api
from commonroad_route_planner.reference_path_planner import ReferencePathPlanner
from commonroad_route_planner.reference_path import ReferencePath

vehicle_shape = Rectangle(length=4.5, width=2.0)  # Typical passenger car dimensions
collisionvertex = {'x': -24.3, 'y': -2.7}
A = {'x': -27.1, 'y': 30}
B = {'x': -47.7, 'y': -7}
steps = 40
speed = steps/4

# Step 1: Load the scenario
scenario_path =  "Commonroad/USA_a1-1_1_T-1.xml"  # Replace with your scenario file path
scenario, planning_problem_set = CommonRoadFileReader(scenario_path).open()



# 2. Define Two Vehicles with Starting Points and Goal Regions
planning_problem_set = PlanningProblemSet()

# Vehicle 1
start_state_1 = InitialState(
    time_step=0,
    position=np.array([-27.1, 30.0]),
    velocity=5.0,
    orientation=0.0,  # Heading angle in radians
    yaw_rate=0.0,  # Rotational velocity
    slip_angle=1.0  # Slip angle
)
goal_state_1 = InitialState(
    time_step=Interval(50, 50),
    position=Circle(radius=1.0, center=np.array([-24.3, -2.7])),
    velocity=Interval(4.0, 6.0),
    orientation=AngleInterval(0.0,2.0),  # Heading angle in radians
    # yaw_rate=Interval(0.0,2.0),  # Rotational velocity
    # slip_angle=Interval(0.0,2.0)

)
goal_region_1 = GoalRegion([goal_state_1])
planning_problem_1 = PlanningProblem(1, initial_state=start_state_1, goal_region=goal_region_1)

# Vehicle 2
start_state_2 = InitialState(
    time_step=0,
    position=np.array([-47.7, -7.0]),
    velocity=7.0,
    orientation=0.0,  # Heading angle in radians
    yaw_rate=0.0,  # Rotational velocity
    slip_angle=1.0  # Slip angle
)
goal_state_2 = InitialState(
    time_step=Interval(50, 50),
    position=Circle(radius=1.0, center=np.array([-24.3, -2.7])),
    velocity=Interval(6.0, 8.0),
    orientation=AngleInterval(0.0,2.0),  # Heading angle in radians
    # yaw_rate=Interval(0.0,2.0),  # Rotational velocity
    # slip_angle=Interval(0.0,2.0)  # Slip angle
)
goal_region_2 = GoalRegion([goal_state_2])
planning_problem_2 = PlanningProblem(2, initial_state=start_state_2, goal_region=goal_region_2)

# Add to Planning Problem Set
planning_problem_set.add_planning_problem(planning_problem_1)
planning_problem_set.add_planning_problem(planning_problem_2)

routes = []
for planning_problem in planning_problem_set.planning_problem_dict.values():
    route_planner = RoutePlanner(scenario.lanelet_network, planning_problem,scenario)
    candidate_holder = route_planner.plan_routes()
    print(candidate_holder)
    route = candidate_holder.pop()
    
    routes.append(route)



lanelets_vehicle_1 = routes[0].lanelet_ids
lanelets_vehicle_2 = routes[1].lanelet_ids
ref1 = routes[0]
ref2 = routes[0]
# ----------------------------------------------------


centerline_points1 = []
centerline_points2 = []

# Iterate through each lanelet ID
for lanelet_id in lanelets_vehicle_1:
    # Retrieve the lanelet object by its ID
    lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
    if lanelet is not None:
        # Append the centerline vertices to the list
        centerline_points1.extend(lanelet.center_vertices)
    else:
        print(f"Lanelet with ID {lanelet_id} not found in the scenario.")

# Convert the list to a NumPy array for further processing

for lanelet_id in lanelets_vehicle_2:
    # Retrieve the lanelet object by its ID
    lanelet = scenario.lanelet_network.find_lanelet_by_id(lanelet_id)
    if lanelet is not None:
        # Append the centerline vertices to the list
        centerline_points2.extend(lanelet.center_vertices)
    else:
        print(f"Lanelet with ID {lanelet_id} not found in the scenario.")

centerline_points1 = np.array(centerline_points1)
centerline_points2 = np.array(centerline_points2)
# reference_path = centerline_points1

reference_path = centerline_points1

config_1 = ReactivePlannerConfiguration()
config_2 = ReactivePlannerConfiguration()

# Update configurations with the scenario and planning problems
config_1.update(scenario=scenario, planning_problem=planning_problem_1)
config_2.update(scenario=scenario, planning_problem=planning_problem_2)

# Initialize reactive planners
reactive_planner_1 = ReactivePlanner(config=config_1)
reactive_planner_2 = ReactivePlanner(config=config_2)

# Set the reference path for the planners using the centerlines
reactive_planner_1.set_reference_path(reference_path=centerline_points1)
# Set desired velocity and plan trajectory
reactive_planner_1.set_desired_velocity(current_speed=planning_problem.initial_state.velocity)
optimal_trajectory = reactive_planner_1.plan()




planned_trajectory = optimal_trajectory[0]

# Step 5: Create Dynamic Obstacles from planned trajectory
dynamic_obstacle = DynamicObstacle(
    obstacle_id=100,
    obstacle_type=ObstacleType.CAR,
    initial_state=planned_trajectory.state_list[0],
    prediction=Trajectory(
        initial_time_step=planned_trajectory.state_list[0].time_step,
        state_list=planned_trajectory.state_list
    )
)

# Add the dynamic obstacle to the scenario
scenario.add_objects(dynamic_obstacle)

# Step 6: Animate the vehicles along the trajectories
def animate_trajectories(scenario, trajectory):
    fig, ax = plt.subplots(figsize=(10, 10))

    # Draw the scenario
    renderer = MPRenderer()
    scenario.draw(renderer)
    planning_problem_set.draw(renderer)
    renderer.render(ax)

    # Extract positions from trajectory
    positions = np.array([state.position for state in trajectory.state_list])

    # Initialize vehicle marker
    vehicle_marker, = ax.plot([], [], 'ro', markersize=5, label='Vehicle')

    # Update function for the animation
    def update(frame):
        if frame < len(positions):
            vehicle_marker.set_data(positions[frame, 0], positions[frame, 1])
        return vehicle_marker,

    # Create animation
    num_frames = len(positions)
    ani = FuncAnimation(fig, update, frames=num_frames, interval=100, blit=True)

    # Display the animation
    plt.legend()
    plt.show()

# Call animation function
animate_trajectories(scenario, planned_trajectory)
