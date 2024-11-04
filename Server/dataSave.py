import json
from time import strftime

def jsonSave(data):
    with open('Website 2.0/public/data.json', 'w') as file:
        json.dump(data, file)

def conv(lng:str, lat:str, signalStrength:str):
    data = {}
    data["lng"] = lng
    data["lat"] = lat
    data["time"] = strftime("%I:%M%p on %d/%m/%y")
    data["strength"] = signalStrength