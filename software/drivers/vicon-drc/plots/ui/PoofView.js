var dgram = require("dgram");
var express = require('express');

var app = express.createServer()
    , io = require('socket.io').listen(app)
    , fs = require('fs')

app.listen(8080);
app.use("/js", express.static(__dirname + '/js'));
app.use("/style", express.static(__dirname + '/style'));
app.use("/images", express.static(__dirname + '/images'));
app.get('/', function (req, res) {
  res.sendfile(__dirname + '/PoofView.html');
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

server.bind(1180);

io.sockets.on('connection', function (socket) {
    });
