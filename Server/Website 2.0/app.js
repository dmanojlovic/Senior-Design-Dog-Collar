const express = require('express');
const app = express();
const port = 3000;
const path = require('path');
var fs = require('fs');

require('dotenv').config();

app.use(express.static(__dirname + '/public'));
app.use(express.json());

app.post("/geofence", (req, res) => {
	fs.writeFile('public/geofence.json', JSON.stringify(req.body), 'utf8', err => { 
        if (err) throw err;
    });
	res.send("recieved");
});

app.get('/', (req, res) => {
	res.sendFile(path.join(__dirname, '/public/index.html'));
});

app.listen(port, () => {
	console.log(`App listening on port ${port}`);
});
