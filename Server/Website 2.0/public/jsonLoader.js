function initMap(latlng){
    const map = new google.maps.Map(document.getElementById("map"), {
        zoom: 18,
        center: latlng,
        mapTypeId: "terrain"
    });

    const posArea = new google.maps.Circle({
        strokeColor: "#FF0000",
        strokeOpacity: 0.8,
        strokeWeight: 2,
        fillColor: "#FF0000",
        fillOpacity: 0.35,
        map,
        center: latlng,
        radius: Math.sqrt(.05) * 100,
    });

    posArea.setMap(map);
}

function jsonLoad() {
    fetch('data.json').then(response => response.json()).then(data => {
    const dataDisplay = document.getElementById("dataDisplay");
    const battery = document.createElement("h3");
    battery.textContent = "Battery Life: " + data.life + "%";

    dataDisplay.append(battery);

    initMap(data.location);

    }).catch(error => console.error("Error fetching JSON data:", error));
}

function load(){}

document.addEventListener("DOMContentLoaded", jsonLoad());