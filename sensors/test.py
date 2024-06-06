import math

def haversine(lat1, lon1, lat2, lon2):
    R = 6371*39370.1 # Earth radius in kilometers

    # Convert latitude and longitude from degrees to radians
    lat1 = math.radians(lat1)
    lon1 = math.radians(lon1)
    lat2 = math.radians(lat2)
    lon2 = math.radians(lon2)

    # Calculate differences
    dlat = lat2 - lat1
    dlon = lon2 - lon1

    # Haversine formula
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    distance = R * c

    # Calculate bearing
    angle = math.atan2(math.sin(lon2 - lon1) * math.cos(lat2),
                       math.cos(lat1) * math.sin(lat2) - math.sin(lat1) * math.cos(lat2) * math.cos(lon2 - lon1))
    angle = math.degrees(angle)
    angle = ((angle + 360) % 360) - 90  # Convert bearing to a compass angle

    return distance, angle

# Example coordinates
lat2, lon2 = 34.64959, -117.881182  # GPS1
lat1, lon1 = 34.649592, -117.881182  # GPS2

distance, angle = haversine(lat1, lon1, lat2, lon2)
print("Distance:", distance, "inches")
print("Angle:", angle, "degrees")
