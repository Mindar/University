// Set the default values for various parameters
var a = 0.0;
var tempHeater = 20.0;
var tempRoom = 20.0;
var tempEnvironment = 15.0;

// the k-factors
var k1 = 1.0;
var k2 = 0.4;
var k3 = 0.4;
var k4 = 0.1;

// Historical temperature data
var historyHeater = [];
var historyRoom = [];
var historyEnv = [];
var historyAlpha = [];

// Global variables for drawing etc
var canvas, ctx, height, width;
var color = "#000";

// Once the window is loaded, call the init function
window.onload = init;

// Initialize important stuff such as the canvas
function init(){
    // Initialize the canvas
    canvas = document.getElementById("diagrams");
    ctx = canvas.getContext("2d");
    
    //doScenario1();
}

// Does the simulation for scenario 1
function doScenario1(){
    reset();
    
    // Setting the starting condidtions for the scenario
    k2 = 0.4;
    a = 0.0;
    tempHeater = 20.0;
    tempRoom = 20.0;
    tempEnvironment = 15.0;
    historyHeater.push(tempHeater);
    historyRoom.push(tempRoom);
    historyEnv.push(tempEnvironment);
    historyAlpha.push(a);
    
    // Do 150 minutes of simulation in 1 minute steps
    for(var t = 0; t < 150; t++){
        //Calculate the temperature difference for the room and heater
        var deltaTH = k1 * a - k2 * (tempHeater - tempRoom);
		var deltaTR = k3 * (tempHeater - tempRoom) - k4 * (tempRoom - tempEnvironment);
        
        // Apply the temperature changes to the temperatures
        tempHeater = tempHeater + deltaTH;
		tempRoom = tempRoom + deltaTR;
        
        // Add the new temperatures to the array of historical temperature data
        historyHeater.push(tempHeater);
        historyRoom.push(tempRoom);
        historyEnv.push(tempEnvironment);
        historyAlpha.push(a);
	    }
        
    //Draw the diagram
    drawDiagram();
    printValues();
}

// Does the simulation for scenario 2
function doScenario2() {
    reset();
    
    // Setting the starting condidtions for the scenario
    k2 = 0.4;
    a = 1.0;
    tempHeater = 20.0;
    tempRoom = 20.0;
    tempEnvironment = 15.0;
    historyHeater.push(tempHeater);
    historyRoom.push(tempRoom);
    historyEnv.push(tempEnvironment);
    historyAlpha.push(a);
    
    for(var t = 0; t < 150; t++){
        //Calculate the temperature difference for the room and heater
        var deltaTH = k1 * a - k2 * (tempHeater - tempRoom);
		var deltaTR = k3 * (tempHeater - tempRoom) - k4 * (tempRoom - tempEnvironment);
        
        // Apply the temperature changes to the temperatures
        tempHeater = tempHeater + deltaTH;
		tempRoom = tempRoom + deltaTR;
        
        // Add the new temperatures to the array of historical temperature data
        historyHeater.push(tempHeater);
        historyRoom.push(tempRoom);
        historyEnv.push(tempEnvironment);
        historyAlpha.push(a);
    }
    
    // Draw the diagram
    drawDiagram();
    printValues();
}

// Does the simulation for scenario 3
function doScenario3(){
    reset();
    k2 = 0.4;
    a = 0.0;
    tempHeater = 20.0;
    tempRoom = 20.0;
    tempEnvironment = 15.0;
    historyHeater.push(tempHeater);
    historyRoom.push(tempRoom);
    historyEnv.push(tempEnvironment);
    historyAlpha.push(a);
    
    for(var t = 0; t < 150; t++){
        // Set environment temperature to 10°C if t is 60
        if(t == 60)
            tempEnvironment = 10.0;
        
        //Calculate the temperature difference for the room and heater
        var deltaTH = k1 * a - k2 * (tempHeater - tempRoom);
		var deltaTR = k3 * (tempHeater - tempRoom) - k4 * (tempRoom - tempEnvironment);
        
        // Apply the temperature changes to the temperatures
        tempHeater = tempHeater + deltaTH;
		tempRoom = tempRoom + deltaTR;
        
        // Calculate the new heater-valve value
        a = calculateNewAlpha(tempRoom);
        
        // Add the new temperatures to the array of historical temperature data
        historyHeater.push(tempHeater);
        historyRoom.push(tempRoom);
        historyEnv.push(tempEnvironment);
        historyAlpha.push(a);
    }
    // Draw the diagram
    drawDiagram();
    printValues();
}

// Does the simulation for scenario 3
function doScenario4(){
    reset();
    k2 = 0.2;
    a = 0.0;
    tempHeater = 20.0;
    tempRoom = 20.0;
    tempEnvironment = 15.0;
    historyHeater.push(tempHeater);
    historyRoom.push(tempRoom);
    historyEnv.push(tempEnvironment);
    historyAlpha.push(a);
    
    for(var t = 0; t < 150; t++){
        // Set environment temperature to 10°C if t is 60
        if(t == 60)
            tempEnvironment = 10.0;
        
        //Calculate the temperature difference for the room and heater
        var deltaTH = k1 * a - k2 * (tempHeater - tempRoom);
		var deltaTR = k3 * (tempHeater - tempRoom) - k4 * (tempRoom - tempEnvironment);
        
        // Apply the temperature changes to the temperatures
        tempHeater = tempHeater + deltaTH;
		tempRoom = tempRoom + deltaTR;
        
        // Calculate the new heater-valve value
        a = calculateNewAlpha(tempRoom);
        
        // Add the new temperatures to the array of historical temperature data
        historyHeater.push(tempHeater);
        historyRoom.push(tempRoom);
        historyEnv.push(tempEnvironment);
        historyAlpha.push(a);
    }
    // Draw the diagram
    drawDiagram();
    printValues();
}

// Calculate the new alpha for the next simulation step
function calculateNewAlpha(temp){
    var error = 25.0 - temp;
    //return error;
    
    if(error > 1.0) {
        return 1.0;
    } else if(error < -1.0) {
        return -1.0;
    } else {
        return error;
    }
}

// Draws the diagram
function drawDiagram(){
    height = canvas.height;
    width = canvas.width;
    color = "#000";
    
    drawGrid();
    drawXAxis();
    drawYAxis();
    drawData(historyEnv, "#f00");
    drawData(historyHeater, "#0f0");
    drawData(historyRoom, "#00f");
}

// Plots the data
function drawData(data, lineColor){
    var xPosStart = width * 0.05;
    var xPosEnd = width * 0.95
    var xSpacing = (xPosEnd - xPosStart) / 150;
    
    
    var yPosStart = height * 0.95;
    var yPosEnd = height * 0.05;
    var yRange = yPosStart - yPosEnd;
    var yRel;
    
    ctx.beginPath();
    
    yRel = data[0] / 30;
	ctx.moveTo(xPosStart, yPosStart - yRel * yRange);
    
    
	for(var i = 1; i <= 150; i++){
        yRel = data[i] / 30;
        ctx.lineTo(xPosStart + xSpacing * i, yPosStart - yRel * yRange);
	}
	ctx.lineWidth = 1;
	ctx.strokeStyle = lineColor;
	ctx.stroke();
}

// Draws the grid on the canvas
function drawGrid(){
    var xPosStart = width * 0.05;
    var xPosEnd = width * 0.95
    var xLines = 150 / 5;
    var xLineSpacing = (width * 0.9) / xLines;
    
    
    var yPosStart = height * 0.95;
    var yPosEnd = height * 0.05;
    var yLines = 30;
    var yLineSpacing = (height * 0.9) / yLines;
    
    
    for(var i = 0; i <= xLines; i++){
        var lineXPos = Math.round(xPosStart + (i * xLineSpacing)) + 0.5;
        
        ctx.beginPath();
        ctx.moveTo(lineXPos, yPosStart);
        ctx.lineTo(lineXPos, yPosEnd);
        ctx.lineWidth = 1;
	    ctx.strokeStyle = "#ddd";
	    ctx.stroke();
    }
    
    for(var i = 0; i <= xLines; i++){
        var lineYPos = Math.round(yPosEnd + (i * yLineSpacing)) + 0.5;
        
        ctx.beginPath();
        ctx.moveTo(xPosStart, lineYPos);
        ctx.lineTo(xPosEnd, lineYPos);
        ctx.lineWidth = 1;
	    ctx.strokeStyle = "#ddd";
	    ctx.stroke();
    }
}

// Draws the X Axis
function drawXAxis(){
    var xPosStart = width * 0.05;
    var xPosEnd = width * 0.96
    var xLines = 150 / 5;
    var xLineSpacing = (width * 0.9) / xLines;
    
    var yPosStart = height * 0.95;
    var yPosEnd = height * 0.05;
    var yLines = 30;
    var yLineSpacing = (height * 0.9) / yLines;
    
    var arrowWidth = ((height + width) / 2) * 0.005;
    var arrowLength = ((height + width) / 2) * 0.01;
    
    ctx.beginPath();
    ctx.moveTo(xPosStart, yPosStart);
    ctx.lineTo(xPosEnd, yPosStart);
    ctx.lineWidth = 2;
    ctx.strokeStyle = color;
    ctx.stroke();
    
    ctx.beginPath();
    ctx.moveTo(xPosEnd + arrowLength, yPosStart);
    ctx.lineTo(xPosEnd, yPosStart + arrowWidth);
    ctx.lineTo(xPosEnd, yPosStart - arrowWidth);
    ctx.fill();
    
    ctx.font = "14px serif";
    ctx.fillText("t / min", xPosEnd - 5, yPosStart - 20);
    
    for(var i = 0; i <= xLines; i++){
        ctx.beginPath();
        ctx.moveTo(Math.round(xPosStart + xLineSpacing * i) + 0.5, yPosStart);
        ctx.lineTo(Math.round(xPosStart + xLineSpacing * i) + 0.5, yPosStart + arrowLength);
        ctx.lineWidth = 1;
        ctx.strokeStyle = color;
        ctx.stroke();
        
        ctx.fillText(i * 5, Math.round(xPosStart + xLineSpacing * i) - 5, yPosStart + height * 0.03);
    }
}

// Draws the Y Axis
function drawYAxis(){
    var xPosStart = width * 0.05;
    var xPosEnd = width * 0.96
    var xLines = 150 / 5;
    var xLineSpacing = (width * 0.9) / xLines;
    
    var yPosStart = height * 0.95;
    var yPosEnd = height * 0.05;
    var yLines = 30;
    var yLineSpacing = (height * 0.9) / yLines;
    
    var arrowWidth = ((height + width) / 2) * 0.005;
    var arrowLength = ((height + width) / 2) * 0.01;
    
    ctx.beginPath();
    ctx.moveTo(xPosStart, yPosStart);
    ctx.lineTo(xPosStart, yPosEnd);
    ctx.lineWidth = 2;
    ctx.strokeStyle = color;
    ctx.stroke();
    
    ctx.beginPath();
    ctx.moveTo(xPosStart, yPosEnd - arrowLength);
    ctx.lineTo(xPosStart + arrowWidth, yPosEnd);
    ctx.lineTo(xPosStart - arrowWidth, yPosEnd);
    ctx.fill();
    
    ctx.font = "14px serif";
    ctx.fillText("T / \u00B0C", xPosStart + 10, yPosEnd);
    
    for(var i = 0; i <= yLines; i++){
        ctx.beginPath();
        ctx.moveTo(xPosStart, yPosStart - i * yLineSpacing + 0.5);
        ctx.lineTo(xPosStart - arrowLength, yPosStart - i * yLineSpacing + 0.5);
        ctx.lineWidth = 1;
        ctx.strokeStyle = color;
        ctx.stroke();
        
        ctx.fillText(i * 1, xPosStart - width * 0.03, yPosStart - i * yLineSpacing + 5);
    }
}



// Resets the simulation
function reset(){
    // Clear the canvas
    ctx.clearRect(0, 0, width, height);
    
    // Clear old historical data from the arrays
    historyHeater = [];
    historyRoom = [];
    historyEnv = [];
    historyAlpha = [];
    
    // Clear debug output
    document.getElementById("debug").innerText = "";
}

// Prints out the values of historyHeater and historyRoom
function printValues(){
    var dbg = document.getElementById("debug");
    for(var i = 0; i < 150; i++){
        dbg.innerText += Math.round(historyHeater[i] * 100) / 100 + ", " + Math.round(historyRoom[i] * 100) / 100 + ", " + Math.round(historyEnv[i] * 100) / 100 + ", " + Math.round(historyAlpha[i] * 100) / 100 + "\n";
    }
}