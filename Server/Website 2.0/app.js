const express = require('express');
const app = express();
const port = 80;
const path = require('path');
var fs = require('fs');
const spawn = require("child_process").spawn;
var audioPlaying = 0;

require('dotenv').config();

app.use(express.static(__dirname + '/public'));
app.use(express.json());

app.post("/geofence", (req, res) => {
	fs.writeFile('public/geofence.json', JSON.stringify(req.body), 'utf8', err => { 
        if (err) throw err;
    });
	res.send("recieved");
});

app.post("/alarm", (req, res) => {
	if(audioPlaying == 0){
		audioPlaying = 1;
		console.log(req.body["button"])
		var pythonProcess = spawn('python', ['test.py', req.body["button"]])
		pythonProcess.stdout.on('data', (data) => {
			console.log(data.toString());
		});
		audioPlaying = 0;
	}
	res.sendStatus(200)
})

app.get('/', (req, res) => {
	res.sendFile(path.join(__dirname, '/public/index.html'));
});

app.listen(port, () => {
	console.log(`App listening on port ${port}`);
	var pythonProcess = spawn('python',['master.py'])
	pythonProcess.stdout.on('data', (data) => {
			console.log(data.toString());
	});
});
