async function initMap(latlng){
    const map = new google.maps.Map(document.getElementById("map"), {
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

    const posArea = new google.maps.Circle({
        strokeColor: "#00FF00",
        strokeOpacity: 0.8,
        strokeWeight: 2,
        fillColor: "#00FF00",
        fillOpacity: 0.35,
        map,
        center: latlng,
        radius: Math.sqrt(.05) * 100,
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
    const battery = document.getElementById("battery");
    battery.textContent = "Battery Life: " + data.life + "%";
    const time = document.getElementById("time");
    time.textContent = "Last Recorded: " + data.time;


    initMap(data.location);

    }).catch(error => console.error("Error fetching JSON data:", error));
}

function load(){}

document.addEventListener("DOMContentLoaded", jsonLoad());