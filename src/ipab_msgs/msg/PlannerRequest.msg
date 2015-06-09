# A planning request message
# A translation of the data from the exotica_planner_request_t lcm message
Header header
string poses         # Registered poses - JSON string
string constraints   # Set of constraints - JSON string
string affordances   # Set of collision affordances - JSON string
string seed_pose     # Seed pose name (initial configuration)
string nominal_pose  # Nominal pose name (for null-space resolution)
string end_pose      # End pose name (for biasing - if available)
string joint_names   # Ordered list of joint names
string options       # Planner options (specified in the GUI)
