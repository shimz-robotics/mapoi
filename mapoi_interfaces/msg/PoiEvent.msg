# Published when the robot enters / exits / stops at / resumes from a POI's tolerance.xy radius
uint8 EVENT_ENTER=1     # Robot entered the POI tolerance.xy radius
uint8 EVENT_EXIT=2      # Robot exited the POI tolerance.xy radius
uint8 EVENT_STOPPED=3   # Robot stopped while inside the POI radius (Nav2 succeeded or sustained zero velocity)
uint8 EVENT_RESUMED=4   # Robot resumed motion after STOPPED inside the POI radius

uint8 event_type                    # EVENT_ENTER / EVENT_EXIT / EVENT_STOPPED / EVENT_RESUMED
mapoi_interfaces/PointOfInterest poi  # The POI that triggered the event
builtin_interfaces/Time stamp       # Timestamp when the event occurred
