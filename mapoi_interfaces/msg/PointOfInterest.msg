# A point of interest (POI) with position, radius, and classification tags
int32 id                    # Unique identifier for this POI
string name                 # Human-readable name
geometry_msgs/Pose pose     # 6-DOF pose in the map frame
float64 radius              # Proximity detection radius [m]
string[] tags               # Classification tags (e.g. "goal", "event")
string description          # Free-text description
