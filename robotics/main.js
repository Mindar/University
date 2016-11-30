window.onload = init;

const scale = 1;

let xzCanv, xyCanv;
let xzCtx, xyCtx;
let TMatrices = [];
let jointCoords = [];
let jointAngles = [5];
let toolCoordinates = [6];
let outtxt = "";
let simulationUnchanged = false;

function init(){
	xzCanv = document.getElementById("xzCanvas");
	xyCanv = document.getElementById("xyCanvas");

	xzCtx = xzCanv.getContext("2d");
	xyCtx = xyCanv.getContext("2d");

	initDefaults();

	setInterval(loop, 100);
}

function initDefaults(){
	//sets default values for all numeric inputs and default robot position
}

function speed_dist(currentT, totalTime, rampTime){
	if(currentT < rampTime){
		//return linearly increasing speed
	} else if ((currentT > rampTime) && (currentT < totalTime - rampTime)){
		return 1;
	} else if (currentT > totalTime - rampTime){
		//return decreasing speed
	} else if (currentT > totalTime){
		return 1;
	} else {
		return 0;
	}
}

/*
changes the transform matrices according to the given joint space angle input
*/
function changedJoint(){
	simulationUnchanged = false;

	jointAngles[0] = document.getElementById("q1").value;
	jointAngles[1] = document.getElementById("q2").value;
	jointAngles[2] = document.getElementById("q3").value;
	jointAngles[3] = document.getElementById("q4").value;
	jointAngles[4] = document.getElementById("q5").value;

	TMatrices[0] = calcT(jointAngles[0], 26.04, 0    , -Math.PI / 2);
	TMatrices[1] = calcT(jointAngles[1], 0    , 22.86, 0           );
	TMatrices[2] = calcT(jointAngles[2], 0    , 22.86, 0           );
	TMatrices[3] = calcT(jointAngles[3], 0    , 0.95 , -Math.PI / 2);
	TMatrices[4] = calcT(jointAngles[4], 16.83, 0    , 0           );
}

function loop(){
	update();
	draw();
}

/*
Transforms the given point from TransformMatrix[frameNumber] into the base coordinate system
*/
function transformToTn(frameNumber, point){
	if(frameNumber == 0){
		return math.multiply(TMatrices[0], point);
	} else if ((frameNumber > 0) && (frameNumber < TMatrices.length)) {
		return transformToTn(frameNumber - 1, math.multiply(TMatrices[frameNumber], point));
	}
}

function getXYCoords(vector){

}

function getXZCoords(vector){

}

function update(){
	
	var basePoint = math.matrix([[0],[0], [0], [1]]);
	var projectedPoint = basePoint;

	if(simulationUnchanged){
		return;
	}

	console.log("updating");

	jointsCoords = [];

	for(var i = 0; i < TMatrices.length; i++){
		jointsCoords.push(transformToTn(i, basePoint));
	}
	/*
		calc arm matrix from variables,
		calc inverse kinematics
	*/
	simulationUnchanged = true;
}

function draw(){
	xzCtx.clearRect(0, 0, xzCanvas.width, xzCanvas.height);
	xzCtx.beginPath();
	xzCtx.moveTo(0, 0);
	xzCtx.lineTo(xzCanvas.width, xzCanvas.height);
	xzCtx.stroke();

	xyCtx.clearRect(0, 0, xyCanvas.width, xyCanvas.height);
	xyCtx.beginPath();
	xyCtx.moveTo(0, 0);
	xyCtx.lineTo(xyCanvas.width, xyCanvas.height);
	xyCtx.stroke();
}





/*

Robot stuff below this point.

*/




function calcT(theta, d, a, alpha){
	var result = math.matrix([
		[Math.cos(theta), -Math.cos(alpha) * Math.sin(theta),  Math.sin(alpha) * Math.sin(theta), a * Math.cos(theta)],
		[Math.sin(theta),  Math.cos(alpha) * Math.cos(theta), -Math.sin(alpha) * Math.cos(theta), a * Math.sin(theta)],
		[0,                Math.sin(alpha),                    Math.cos(alpha),	                  d                  ],
		[0,                0,                                  0,                                 1                  ]
	]);

	return result;
}

/*
function calcArmMatrix(q1, q2, q3, q4, q5){
	var T_One_Base   = calcT(q1, 26.04,	 0,		 -Math.PI / 2);
	var T_Two_One    = calcT(q2, 0,		 22.86,	 0		);
	var T_Three_Two  = calcT(q3, 0,		 22.86,	 0		);
	var T_Four_Three = calcT(q4, 0,		 0.95,	 -Math.PI / 2);
	var T_Tool_Four  = calcT(q5, 16.83,	 0,		 0		);

	var result = math.multiply(T_One_Base, math.multiply(T_Two_One, math.multiply(T_Three_Two, math.multiply(T_Four_Three, T_Tool_Four))));

	return result;
}*/