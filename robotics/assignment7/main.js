window.onload = init;

let ctx, canv;
let txt = "";
let TMatrices = [];

let targetPos = [];
let targetPosJoints = [];
let currentPos = [];
let currentPosJoints = [];

const scale = 5;

function init(){
	canv = document.getElementById("mycanv");
	ctx = canv.getContext("2d");
	simulate();
}



async function simulate(){
	var w1 = document.getElementById("w1").value;
	var w2 = document.getElementById("w2").value;
	var w3 = document.getElementById("w3").value;
	var w4 = document.getElementById("w4").value;
	var w5 = document.getElementById("w5").value;
	var w6 = document.getElementById("w6").value;
	var time = document.getElementById("time").value;

	targetPos = [w1, w2, w3, w4, w5, w6];

	var wDirLen = Math.sqrt(w4 * w4 + w5 * w5 + w6 * w6);
	w4 /= wDirLen;
	w5 /= wDirLen;
	w6 /= wDirLen;

	var jVariables =[w1, w2, w3, w4, w5 ,w6];
	targetPosJoints = calcInverseKinematics(jVariables);

	if(time == 0){
		calcAll(targetPosJoints);
		currentPos = targetPos;
		currentPosJoints = targetPosJoints;
	} else {
		for(var i = 0; i < Math.round(time * 10); i++){
			
			let toolSpeed = calcToolSpeed(i / 10, time);
			let newPos = math.add(currentPos, toolSpeed);
			let jointVel = calcJointSpeeds(newPos);
			let newJoints = math.add(jointVel, currentPosJoints);

			console.log(newPos + " " + toolSpeed);
			calcAll(newJoints);

			currentPos = newPos;
			currentPosJoints = newJoints;

			await sleep(100);
		}
	}
}

function sleep(ms) {
  return new Promise(resolve => setTimeout(resolve, ms));
}


function calcLength(vec){
	return Math.sqrt(vec[0] * vec[0] + vec[1] * vec[1] + vec[2] * vec[2]);
}


function calcDirectKinematics(jointSpaceCoords){
	var TMatrix = calcArmMatrix(jointSpaceCoords[0], jointSpaceCoords[1], jointSpaceCoords[2], jointSpaceCoords[3], jointSpaceCoords[4]);
	var pos = math.multiply(TMatrix, [0,0,0,1]);
	
	var result = [6];

	result[0] = math.subset(pos, math.index(0));
	result[1] = math.subset(pos, math.index(1));
	result[2] = math.subset(pos, math.index(2));
	result[3] = -Math.exp(jointSpaceCoords[4] / Math.PI) * Math.cos(jointSpaceCoords[0]) * Math.sin(jointSpaceCoords[1]) * Math.sin(jointSpaceCoords[2]) * Math.sin(jointSpaceCoords[3]);
	result[4] = -Math.exp(jointSpaceCoords[4] / Math.PI) * Math.sin(jointSpaceCoords[0]) * Math.sin(jointSpaceCoords[1]) * Math.sin(jointSpaceCoords[2]) * Math.sin(jointSpaceCoords[3]);
	result[5] = -Math.exp(jointSpaceCoords[4] / Math.PI) * Math.cos(jointSpaceCoords[1]) * Math.cos(jointSpaceCoords[2]) * Math.cos(jointSpaceCoords[3]);
	
	return result;
}

function calcInverseKinematics(toolConfig){
	var result = [5];

	result[0] = Math.atan2(toolConfig[1], toolConfig[0]);
	var tmpSum = Math.atan2(-(Math.cos(result[0]) * toolConfig[3] + Math.sin(result[0]) * toolConfig[5]), -toolConfig[5]);
	var b1 = Math.cos(result[0]) * toolConfig[0] + Math.sin(result[0]) * toolConfig[1] - 0.95 * Math.cos(tmpSum) + 16.83 * Math.sin(tmpSum);
	var b2 = 26.04 - 0.95 * Math.sin(tmpSum) - 16.83 * Math.cos(tmpSum) - toolConfig[2];
	var bNorm = Math.pow(b1, 2) + Math.pow(b2, 2);

	result[2] = Math.acos((bNorm - 2 * Math.pow(22.86, 2))/(2 * Math.pow(22.86, 2)));


	result[1] = Math.atan2((22.86 + 22.86 * Math.cos(result[2])) * b2 - 22.86 * b1 * Math.sin(result[2]), (22.86 + 22.86 * Math.cos(result[2])) * b1 + 22.86 * b2 * Math.sin(result[2]));
	result[3] = tmpSum - result[2] - result[1];
	result[4] = Math.PI * Math.log(Math.sqrt(Math.pow(toolConfig[3], 2) + Math.pow(toolConfig[4], 2) + Math.pow(toolConfig[5], 2)));

	return result;
}

function calcJointSpeeds(newpos){
	return math.subtract(calcInverseKinematics(newpos), calcInverseKinematics(currentPos));
}

function calcToolSpeed(curT, time){
	let speed = speed_dist(curT, time, time * 0.1);
	let targetVec = math.subtract(targetPos, currentPos);
	let distanceToTarget = calcLength(targetVec);
	let result = math.multiply(speed / distanceToTarget, targetVec);
	return result;
}

function speed_dist(currentTime, totalTime, rampTime){
	if(currentTime < 0){
		return 0;
	} else if (currentTime < rampTime){
		return 1/(totalTime - rampTime) * (currentTime / rampTime);
	} else if (currentTime < (totalTime - rampTime)){
		return 1/(totalTime - rampTime);
	} else if (currentTime < totalTime){
		return 1/(totalTime - rampTime) - 1/(totalTime - rampTime) * currentTime / rampTime;
	} else {
		return 1;
	}
}








// internal stuff below









function transformToTn(frameNumber, point){
	if(frameNumber == 0){
		return math.multiply(TMatrices[0], point);
	} else if ((frameNumber > 0) && (frameNumber < TMatrices.length)) {
		return transformToTn(frameNumber - 1, math.multiply(TMatrices[frameNumber], point));
	}
}


function calcT(theta, d, a, alpha){
	var result = math.matrix([
		[Math.cos(theta), -Math.cos(alpha) * Math.sin(theta),  Math.sin(alpha) * Math.sin(theta), a * Math.cos(theta)],
		[Math.sin(theta),  Math.cos(alpha) * Math.cos(theta), -Math.sin(alpha) * Math.cos(theta), a * Math.sin(theta)],
		[0,                Math.sin(alpha),                    Math.cos(alpha),	                  d                  ],
		[0,                0,                                  0,                                 1                  ]
	]);

	return result;
}

function calcArmMatrix(q1, q2, q3, q4, q5){
	var T_One_Base   = calcT(q1, 26.04,	 0,		 -Math.PI / 2);
	var T_Two_One    = calcT(q2, 0,		 22.86,	 0		);
	var T_Three_Two  = calcT(q3, 0,		 22.86,	 0		);
	var T_Four_Three = calcT(q4, 0,		 0.95,	 -Math.PI / 2);
	var T_Tool_Four  = calcT(q5, 16.83,	 0,		 0		);

	var result = math.multiply(T_One_Base, math.multiply(T_Two_One, math.multiply(T_Three_Two, math.multiply(T_Four_Three, T_Tool_Four))));

	return result;
}


function calcAll(inputVec){
	/*
	var q1 = document.getElementById("q1").value;
	var q2 = document.getElementById("q2").value;
	var q3 = document.getElementById("q3").value;
	var q4 = document.getElementById("q4").value;
	var q5 = document.getElementById("q5").value;*/
	var q1 = inputVec[0];
	var q2 = inputVec[1];
	var q3 = inputVec[2];
	var q4 = inputVec[3];
	var q5 = inputVec[4];

	var armMatrix = calcArmMatrix(q1, q2, q3, q4, q5);

	var vec = armMatrix.subset(math.index(math.range(0, 3), 3));

	var T1 = calcT(q1, 26.04, 0,    -Math.PI / 2);
	var T2 = calcT(q2, 0,     22.86, 0          );
	var T3 = calcT(q3, 0,     22.86, 0          );
	var T4 = calcT(q4, 0,     0.95, -Math.PI / 2);
	var T5 = calcT(q5, 16.83, 0,     0          );
	/*
	dbgLog(T1, "T 1 -> base");
	dbgLog(T2, "T 2 -> 1");
	dbgLog(T3, "T 3 -> 2");
	dbgLog(T4, "T 4 -> 3");
	dbgLog(T5, "T tool -> 4");
	dbgLog(armMatrix, "Arm Matrix");
	dbgDisplay();*/


	/*
		Calculate positions of all joints in the xz-plane
	*/
	var basePoint = math.matrix([[0],[0], [0], [1]]);
	var projectedPoint = basePoint;
	var jointCoords = [];



	ctx.clearRect(0, 0, canv.width, canv.height);

	var xVal = projectedPoint.subset(math.index(0, 0));
	var zVal = projectedPoint.subset(math.index(2, 0));
	jointCoords.push({x: xVal, z: zVal});

	projectedPoint = math.multiply(T1, basePoint);
	xVal = projectedPoint.subset(math.index(0, 0));
	zVal = projectedPoint.subset(math.index(2, 0));
	jointCoords.push({x: xVal, z: zVal});

	projectedPoint = math.multiply(math.multiply(T1, T2), basePoint);
	xVal = projectedPoint.subset(math.index(0, 0));
	zVal = projectedPoint.subset(math.index(2, 0));
	jointCoords.push({x: xVal, z: zVal});
	
	projectedPoint = math.multiply(math.multiply(T1, math.multiply(T2, T3)), basePoint);
	xVal = projectedPoint.subset(math.index(0, 0));
	zVal = projectedPoint.subset(math.index(2, 0));
	jointCoords.push({x: xVal, z: zVal});
	
	projectedPoint = math.multiply(math.multiply(T1, math.multiply(T2, math.multiply(T3, T4))), basePoint);
	xVal = projectedPoint.subset(math.index(0, 0));
	zVal = projectedPoint.subset(math.index(2, 0));
	jointCoords.push({x: xVal, z: zVal});
	
	projectedPoint = math.multiply(math.multiply(T1, math.multiply(T2, math.multiply(T3, math.multiply(T4, T5)))), basePoint);
	xVal = projectedPoint.subset(math.index(0, 0));
	zVal = projectedPoint.subset(math.index(2, 0));
	jointCoords.push({x: xVal, z: zVal});


	/*
		Calculate positions of all coordinate system origins
	*/
	var xAxis = math.matrix([[10],[0], [0], [1]]);
	var yAxis = math.matrix([[0],[10], [0], [1]]);
	var zAxis = math.matrix([[0],[0], [10], [1]]);
	var projectedX, projectedY, projectedZ;

	var coordSystems = [];

	projectedX = math.multiply(T1, xAxis);
	projectedY = math.multiply(T1, yAxis);
	projectedZ = math.multiply(T1, zAxis);
	coordSystems.push({x: projectedX, y: projectedY, z: projectedZ});
	
	projectedX = math.multiply(math.multiply(T1, T2), xAxis);
	projectedY = math.multiply(math.multiply(T1, T2), yAxis);
	projectedZ = math.multiply(math.multiply(T1, T2), zAxis);
	coordSystems.push({x: projectedX, y: projectedY, z: projectedZ});
	
	projectedX = math.multiply(math.multiply(T1, math.multiply(T2, T3)), xAxis);
	projectedY = math.multiply(math.multiply(T1, math.multiply(T2, T3)), yAxis);
	projectedZ = math.multiply(math.multiply(T1, math.multiply(T2, T3)), zAxis);
	coordSystems.push({x: projectedX, y: projectedY, z: projectedZ});
	
	projectedX = math.multiply(math.multiply(T1, math.multiply(T2, math.multiply(T3, T4))), xAxis);
	projectedY = math.multiply(math.multiply(T1, math.multiply(T2, math.multiply(T3, T4))), yAxis);
	projectedZ = math.multiply(math.multiply(T1, math.multiply(T2, math.multiply(T3, T4))), zAxis);
	coordSystems.push({x: projectedX, y: projectedY, z: projectedZ});

	projectedX = math.multiply(math.multiply(T1, math.multiply(T2, math.multiply(T3, math.multiply(T4, T5)))), xAxis);
	projectedY = math.multiply(math.multiply(T1, math.multiply(T2, math.multiply(T3, math.multiply(T4, T5)))), yAxis);
	projectedZ = math.multiply(math.multiply(T1, math.multiply(T2, math.multiply(T3, math.multiply(T4, T5)))), zAxis);
	coordSystems.push({x: projectedX, y: projectedY, z: projectedZ});

	drawCoords(coordSystems, jointCoords);
	drawJoints(jointCoords);
}

function drawJoints(points){
	ctx.beginPath();
	ctx.strokeStyle = "#000";
	ctx.moveTo(canv.width / 2, canv.height - 100);
	for(var i = 0; i < points.length; i++){
		ctx.lineTo(canv.width/2 + points[i].x * scale, canv.height - 100 - points[i].z * scale);
	}
	ctx.stroke();
}

function drawCoords(axes, bases){
	for(var i = 0; i < axes.length; i++){
		var xAxis = axes[i].x;
		var yAxis = axes[i].y;
		var zAxis = axes[i].z;

		var x1 = xAxis.subset(math.index(0, 0));
		var z1 = xAxis.subset(math.index(2, 0));
		var x2 = yAxis.subset(math.index(0, 0));
		var z2 = yAxis.subset(math.index(2, 0));
		var x3 = zAxis.subset(math.index(0, 0));
		var z3 = zAxis.subset(math.index(2, 0));

		ctx.beginPath();
		ctx.strokeStyle = "#f00";
		//console.log(x1 + " " +  bases[i+1].x);
		ctx.moveTo(canv.width / 2 + bases[i+1].x * scale, canv.height - 100 - bases[i+1].z * scale);
		ctx.lineTo(canv.width / 2 + x1 * scale, canv.height - 100 - z1 * scale);
		ctx.stroke();

		ctx.beginPath();
		ctx.strokeStyle = "#0f0";
		ctx.moveTo(canv.width / 2 + bases[i+1].x * scale, canv.height - 100 - bases[i+1].z * scale);
		ctx.lineTo(canv.width / 2 + x2 * scale, canv.height - 100 - z2 * scale);
		ctx.stroke();

		ctx.beginPath();
		ctx.strokeStyle = "#00f";
		ctx.moveTo(canv.width / 2 + bases[i+1].x * scale, canv.height - 100 - bases[i+1].z * scale);
		ctx.lineTo(canv.width / 2 + x3 * scale, canv.height - 100 - z3 * scale);
		ctx.stroke();
	}
}

function dbgLog(matrix, title){
	txt += title + "\n";
	
	var lines = matrix.size()[0];
	var cols = matrix.size()[1];

	for(var i = 0; i < lines; i++){
		for(var j = 0; j < cols; j++){
			txt += math.round(matrix.subset(math.index(i, j)), 4) + " ";
		}
		txt += "\n";
	}
	txt += "\n\n";
}

function dbgDisplay(){
	document.getElementById("debugOut").innerText = txt;
	txt = "";
}