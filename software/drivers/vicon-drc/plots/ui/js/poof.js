$(function () {
$("#shooterslider").slider({
    orientation: "vertical",
    range: true,
    min: 0,
    max: 100,
    values: [0,100],
    disabled: true
});

   var socket = io.connect('http://localhost');
    socket.on('update', function (dataIn) {
       console.log(dataIn);
       var s = dataIn["S"];
       var v = dataIn["V"];
       var c = dataIn["C"];
       goal = s;
       curr = s * .8;
       $("#shooterval").text(goal);
       $("#shootergoal").text(curr);
       $("#shooterslider").slider("values",1,goal);
       $("#shooterslider").slider("values",0,curr);
    });
});

