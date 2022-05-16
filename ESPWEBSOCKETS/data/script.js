var gateway = `ws://${window.location.hostname}/ws`;
var websocket;
window.addEventListener('load', onload);


function onload(event) {
    initWebSocket();
}

function initWebSocket() {
    console.log('Trying to open a WebSocket connection…');
    websocket = new WebSocket(gateway);
    websocket.onopen = onOpen;
    websocket.onclose = onClose;
    websocket.onmessage = onMessage;
}

function onOpen(event) {
    console.log('Connection opened');
}

function onClose(event) {
    console.log('Connection closed');
    //document.getElementById("robot-state").innerHTML = "uncalibrated"
    setTimeout(initWebSocket, 2000);
}

function submitForm(){


    document.getElementById("robot-state").style.color = "blue";
    var Deliveries = document.getElementById("Deliveries").value;
    var Routes = document.getElementById("Routes").value;
    var Heights = document.getElementById("Heights").value;
    websocket.send(Deliveries+"@"+Routes+"&"+Heights);
}

function onMessage(event) {
    console.log(event.data);
    var state;
    if (event.data == "0") {
      state = "UNCALIBRATED";
    } else if (event.data == "1") {
      state = "CALIBRATED";
    } else if (event.data == "2") {
      state = "Ready for Orders";
    } else if (event.data == "3") {
      state = "On its Way";
    } else if (event.data == "4") {
      state = "Turning";
    } else if (event.data == "5") {
      state = "Picking up Package";
    } else if (event.data == "6") {
      state = "Delivering";
    }

    document.getElementById("robot-state").innerHTML = state;
}