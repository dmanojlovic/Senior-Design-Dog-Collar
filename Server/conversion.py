import json


#Purdue Location
data = {
    "life" : 85,
    "location" : {
        "lng" : -86.929,
        "lat" : 40.424
    }
}

def jsonSave(data):
    with open('Website 2.0/public/data.json', 'w') as file:
        json.dump(data, file)

def NMEAConv(lng :str, lat :str):
    degLng = lng[0:3]
    degLat = lat[0:2]
    decLng = float(lng[3:10]) 
    decLat = float(lat[2:9])

    lngDir = lng[11]
    latDir = lat[10]

    decLng /= 60
    decLat /= 60

    outLng = float(degLng) + decLng
    outLat = float(degLat) + decLat

    if(lngDir == 'W'):
        outLng *= -1
    if(latDir == 'S'):
        outLat *= -1

    return outLng, outLat

lng, lat = NMEAConv("08654.8235,W", "4025.7222,N")

data['location']['lng'] = lng
data['location']['lat'] = lat

jsonSave(data)

