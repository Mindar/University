window.onload = init;

let ctx, canv;
let txt = "";
const scale = 5;

function init(){
	canv = document.getElementById("mycanv");
	ctx = canv.getContext("2d");
	calcAll();
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

function calcAll(){
	var q1 = document.getElementById("q1").value;
	var q2 = document.getElementById("q2").value;
	var q3 = document.getElementById("q3").value;
	var q4 = document.getElementById("q4").value;
	var q5 = document.getElementById("q5").value;

	var armMatrix = calcArmMatrix(q1, q2, q3, q4, q5);

	var vec = armMatrix.subset(math.index(math.range(0, 3), 3));

	var T1 = calcT(q1, 26.04, 0,    -Math.PI / 2);
	var T2 = calcT(q2, 0,     22.86, 0          );
	var T3 = calcT(q3, 0,     22.86, 0          );
	var T4 = calcT(q4, 0,     0.95, -Math.PI / 2);
	var T5 = calcT(q5, 16.83, 0,     0          );

	dbgLog(T1, "T 1 -> base");
	dbgLog(T2, "T 2 -> 1");
	dbgLog(T3, "T 3 -> 2");
	dbgLog(T4, "T 4 -> 3");
	dbgLog(T5, "T tool -> 4");
	dbgLog(armMatrix, "Arm Matrix");
	dbgDisplay();


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
		console.log(x1 + " " +  bases[i+1].x);
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