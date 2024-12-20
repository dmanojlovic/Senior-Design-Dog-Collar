var initialized = 0;
var map;
var posArea;

async function initMap(latlng){
    map = new google.maps.Map(document.getElementById("map"), {
        zoom: 18,
        center: latlng,
        mapTypeId: "terrain"
    });

    const res = await fetch('geofence.json');

    const fenceData = await res.json();

    const fenceCoords = {
        north: fenceData.north,
        south: fenceData.south,
        east:  fenceData.east,
        west:  fenceData.west
    }

    const geofence = new google.maps.Rectangle({
        bounds: fenceCoords,
        strokeColor: "#FF0000",
        strokeOpacity: 0.8,
        strokeWeight: 2,
        fillColor: "#FF0000",
        fillOpacity: 0.35,
        editable: true,
        draggable: true
      });

    posArea = new google.maps.Circle({
        strokeColor: "#00FF00",
        strokeOpacity: 0.8,
        strokeWeight: 2,
        fillColor: "#00FF00",
        fillOpacity: 0.35,
        map,
        center: latlng,
        radius: Math.sqrt(.01) * 100,
    });

    posArea.setMap(map);
    geofence.setMap(map);
    ["bounds_changed", "dragstart", "drag", "dragend"].forEach((eventName) => {
        geofence.addListener(eventName, () => {
          fetch("/geofence", {
        method: "POST",
        headers: {
            'Content-Type': 'application/json'       
        },
        body: JSON.stringify(geofence.getBounds())
    }).catch(err => {
        console.log(err);
    })
        });
    });
}

function jsonLoad() {
    fetch('data.json').then(response => response.json()).then(data => {
    const time = document.getElementById("time");
    const strength = document.getElementById("strength");
    time.textContent = "Last Recorded: " + data.time;
    strength.textContent = "Communication Signal Strength: " + data.strength;
    if(initialized == 0){
        initMap(data.location);
        initialized = 1
    }
    else{
        posArea.setMap(null);
        posArea = new google.maps.Circle({
            strokeColor: "#00FF00",
            strokeOpacity: 0.8,
            strokeWeight: 2,
            fillColor: "#00FF00",
            fillOpacity: 0.35,
            map,
            center: data.location,
            radius: Math.sqrt(.01) * 100,
        });
        posArea.setMap(map);
        map.setCenter(data.location);
    }
    }).catch(error => console.error("Error fetching JSON data:", error));
}


