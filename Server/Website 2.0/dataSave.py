import json
from time import strftime

def jsonSave(data):
    with open('public/data.json', 'w') as file:
        json.dump(data, file)

def conv(lng, lat, signalStrength):
    data = {}
    data["location"] = {}
    data["location"]["lng"] = lng
    data["location"]["lat"] = lat
    data["time"] = strftime("%I:%M%p on %m/%d/%y")
    data["strength"] = signalStrength
    print(data)
    jsonSave(data)
