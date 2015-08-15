var express = require('express');
var app = express();
var server = require('http').createServer(app);
var io = require('socket.io').listen(server);
var dgram = require('dgram');

server.listen(8080);
app.use("/js", express.static(__dirname + '/js'));
app.use("/style", express.static(__dirname + '/style'));
app.use("/images", express.static(__dirname + '/images'));
app.get('/', function (req, res) {
  res.sendfile(__dirname + '/index.html');
});

var server = dgram.createSocket("udp4");

server.on("message", function (msg, rinfo) {
        var jsonObj = JSON.parse(msg.toString('utf8'));
	io.sockets.emit('update', jsonObj);
        
    });

server.on("listening", function () {
	var address = server.address();
	console.log("server listening " +
		    address.address + ":" + address.port);
    });

server.bind(41234, 'localhost');

io.sockets.on('connection', function (socket) {
    });
