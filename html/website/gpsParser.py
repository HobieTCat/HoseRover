latitudes = []
longitudes = []
def getPositionData(data):
    if len(data) < 6:
    # Not enough data to parse latitude and longitude
        return None, None

    lat = data[2]
    lat_dir = data[3]
    lon = data[4]
    lon_dir = data[5]

    if not lat or not lon:
    # Latitude or longitude data is missing
        return None, None


    # Split the data into lines
    lines = data.split('$')

    # Iterate through each line
    for line in lines:
        # Check if the line starts with 'GP'
        if line.startswith('GP'):
            # Split the line by comma
            data = line.split(',')
            if data[0] == 'GPGGA':
                # Get latitude and longitude data
                lat = data[2]
                lat_dir = data[3]
                lon = data[4]
                lon_dir = data[5]   
                 
                 #Create arrays for lat, lon that are fifo
                latitudes.append(lat)
                longitudes.append(lon)
                if len(latitudes) > 5:
                    latitudes.pop(0)
                    longitudes.pop(0)
        
                avg_latitude = sum(float(lats) for lats in latitudes) / len(latitudes)      
                avg_latitude = round(avg_latitude,6)          
                # avg_longitude = sum(float(lons) for lons in longitudes) / len(longitudes) 
                avg_longitude = round(avg_longitude,6)  

                # Return latitude and longitude data as a tuple
                # print("RawLat= ", avg_latitude)
                # print("RawLong= ", lon)
                # return lat_dec, lon_dec
                return str(avg_latitude), str(avg_longitude)

    # Return None if the data does not contain valid GPS data
    return None, None

