import json


#Purdue Location
'''
data = {
    "life" : 85,
    "location" : {
        "lng" : -86.929,
        "lat" : 40.424
    }
}
'''

def NMEAConv(lng :str, lat :str):
    degLng = lng[0:3]
    degLat = lat[0:2]
    decLng = float(lng[3:10]) 
    decLat = float(lat[2:9])

    lngDir = lng[-1]
    latDir = lat[-1]

    decLng /= 60
    decLat /= 60

    outLng = float(degLng) + decLng
    outLat = float(degLat) + decLat

    if(lngDir == 'W'):
        outLng *= -1
    if(latDir == 'S'):
        outLat *= -1

    return outLng, outLat

