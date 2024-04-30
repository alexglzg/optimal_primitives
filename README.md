# Motion primitive generator based on optimal control

Python library for generating Motion primitives. The main script generates a set of motion primitives based on optimal control. The trajectory cost  is time-based.

## Install  

The required python packages can be installed by running the following command: 

pip install -r requirements.txt 

It has been tested with Python versions 3.8 and 3.10.

## Getting Started 

Before running the main code, you can modify/edit the settings (i.e, maximum turning radius, maximum velocity, etc) in the file (config.json).  

As an example, you can run directly the file “generate_optimal_primitives.py”. The resulted motion primitives are saved in the file “output.json” and some pictures are stored in the ‘visualizations’ folder. 

## Parameters 

The parameters are not given by default and should be specified in the file “config.json” 


**motion_model (string)** 

Note. The "motion_model" name is not used inside the code but is required if we test the motion primitives with “Nav2 smac planner “.    

The type of motion model used. Accepted values: 

diff: Forward moving trajectories + rotation in place by a single angular bin 

### TODO: Incorporate the following models: 

ackermann: Only forward and reversing trajectories 
omni: Forward moving trajectories + rotation in place by a single angular bin + sideways sliding motions 

 
**turning_radius (float)**

The minimum turning radius of the robot (in meters).  

**grid_resolution (float)**

The resolution of the grid (in meters) used to create the lattice. If the grid resolution is too large proportionally to the turning radius then the generator will not return a good result. This should be the same as your costmap resolution. 
 

**stopping_threshold (float)**

Number of consecutive iterations without a valid trajectory before stopping the search. A value too low may mean that you stop searching too early. A value too high will cause the search to take longer. Stopping after 5 iterations can produce the best results based on our experience. 

**num_of_headings (int)**

Number of discrete angular headings used. Due to the way discretization is done this number should be restricted to multiples of 8. Angles are not generated uniformly but instead generated in a way to facilitate straight lines. See angle discretization (below) for more details. We believe 16 headings is a good number for most use cases. 
 
**max_vel (float)**

Maximum Longitudinal speed   

**min_vel (float)**

Minimium Longitudinal speed. Set to 0 to avoid backward driving.   

**max_ang_vel (float)**

Maximum Angular speed   

**max_lin_acc (float)**

Maximum Longitudinal acceleration   

**max_ang_acc (float)**

Maximum Angular acceleration   


## Output file structure
The motion primitives are stored in the JSON output file as required by Nav2 smac planner and contains the following fields: 

**version** 

The version number of the lattice generator that created the output file 

**date_generated**

The date the output file was generated. Format: YYYY-MM-DD 

## Lattice_metadata 

A dictionary that contains information about the generated lattice. Most of this data comes from the config file used when generating the primitives. More information on each field is given in the Parameters section. Includes the following fields: 

- motion_model 

- turning_radius (meters) 

- grid_resolution (meters) 

- stopping_threshold 

- num_of_headings 

- heading_angles : A list of the heading angles (in radians) that are used in the primitives 

- number_of_trajectories : The total number of trajectories contained in the output file 

- max_vel

- min_vel

- max_ang_vel

- max_lin_acc

- max_ang_acc

## Primitives 

A list of dictionaries where each dictionary represents an individual motion primitive. Each motion primitive contains the following fields: 

- trajectory_id : The id associated with the primitive 

- start_angle_index : The start angle of the primitive represented as an index for the heading_angle list given in the lattice_metadata 

- end_angle_index : The end angle of the primitive represented as an index for the heading_angle list given in the lattice_metadata 

- left_turn  : A boolean value that is true if the path curves to the left. Straight paths default to true.   

- trajectory_radius (meters) : The radius of the circle that was used to create the arc portion of a primitive. trajectory_radius is 0 if the primitive is purely a straight line 

- trajectory_length (meters) : The length of the primitive 

- arc_length (meters) : Arc_length is 0 if the primitive is a straight line or turn on the spot

- straight_length (meters) : The length of the straight line portion of a primitive. straight_length is 0 if the primitive is a pure arc. 
poses. *TODO*: this value is not currently used.

- A list where each entry is a list containing three values: x, y, and yaw (radians) 

## Angle Discretization and Trajectory Generator 

More details in https://github.com/ros-planning/navigation2/tree/main/nav2_smac_planner/lattice_primitives 

## Contact
Any questions can be directed to:
- Alejandro Gonzalez Garcia: alex.gonzalezgarcia@kuleuven.be / alexglzg97@gmail.com
