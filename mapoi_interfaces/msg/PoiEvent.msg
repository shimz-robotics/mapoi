# Published when the robot enters or exits a POI's radius
uint8 EVENT_ENTER=1   # Robot entered the POI radius
uint8 EVENT_EXIT=2    # Robot exited the POI radius

uint8 event_type                    # EVENT_ENTER or EVENT_EXIT
mapoi_interfaces/PointOfInterest poi  # The POI that triggered the event
builtin_interfaces/Time stamp       # Timestamp when the event occurred
