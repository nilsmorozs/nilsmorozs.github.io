// Global variables	
var outputs = new Array(0, 0, 0, 0, -Math.PI, 0, 0, 0);	
var Ts = 10; // 100 Hz sampling
var SAMPLES_PER_FRAME = 4; // 25 frames per second
var Tframe = Ts * SAMPLES_PER_FRAME;
var prev_time = 0; // timer
var pendulum_length;

window.requestAnimFrame = (function(callback) {
        return window.requestAnimationFrame || window.webkitRequestAnimationFrame || window.mozRequestAnimationFrame || window.oRequestAnimationFrame || window.msRequestAnimationFrame ||
        function(callback) {
          window.setTimeout(callback, 1000 / 60);
        };
		})();

function runSfcExperiment()
{
	var ripCanvas = document.getElementById('rip_canvas');
    var ripContext = ripCanvas.getContext('2d');
	
	pendulum_length = ripCanvas.height / 2 - 5;
		
	drawRIP(ripCanvas, ripContext, outputs);
	prev_time = (new Date()).getTime();
	
	animate(ripCanvas, ripContext);
}

function drawRIP(canvas, context, outputs)
{
	var arm_length = pendulum_length;
	
	// Draw the ground level
	context.strokeStyle = "black";
	context.lineWidth = 1;
	context.moveTo(0, canvas.height/2);
	context.lineTo(canvas.width, canvas.height/2);
	context.stroke();
	
	context.beginPath();
	
	// Draw the pendulum
	context.strokeStyle = "#FF0000";
	context.lineWidth = 3;
	context.moveTo(canvas.width/2 - outputs[0]*arm_length - 10, canvas.height/2);
	context.lineTo(canvas.width/2 - outputs[0]*arm_length + 10, canvas.height/2);
	context.stroke();
	context.moveTo(canvas.width/2 - outputs[0]*arm_length, canvas.height/2);
	context.lineTo(canvas.width/2 - outputs[0]*arm_length + pendulum_length * Math.sin(outputs[4]), canvas.height/2 - pendulum_length * Math.cos(outputs[4]));
	context.stroke();
	
}

function animate(canvas, context)
{
	var Vm = 0;
	var n;
	var wrapped_alpha;
	var time;
	
	//// Update
	for (n=0; n < SAMPLES_PER_FRAME; n++)
	{
		// Wrap value of alpha
		wrapped_alpha = outputs[4];
		while (wrapped_alpha >= Math.PI)
		{
			wrapped_alpha = wrapped_alpha - 2*Math.PI;
		}
		while (wrapped_alpha < -Math.PI)
		{
			wrapped_alpha = wrapped_alpha + 2*Math.PI;
		}
		
		if (Math.abs(wrapped_alpha) <= Math.PI/6)
			// Calculate voltage, using the state feedback control law
			Vm = 0.0087*outputs[2] + 0.037*outputs[3] - 6*wrapped_alpha - 0.8*outputs[5];
		else
			// Energy-based swing-up control law
			if (((wrapped_alpha < -(5/6)*Math.PI) || (wrapped_alpha > (5/6)*Math.PI)) && (outputs[5] == 0) && (outputs[3] == 0))
				Vm = 6;
			else
				Vm = 0.17 * outputs[5] * Math.cos(wrapped_alpha);
			
		// Saturate Vm to +- 12V
		Vm = Math.min(Vm, 12);
		Vm = Math.max(Vm, -12);
				
		// Proceed to the next sample
		outputs = ripTransition(Vm, outputs, Ts, 0.12);
		//outputs[4] += 2*Math.PI * Ts/60000;
	}
	
	// polling loop for delay
	time = (new Date()).getTime();
	while(time < prev_time + Tframe)
	{
		time = (new Date()).getTime();
	}
	prev_time  = time;
	
	//// Clear canvas
	context.clearRect(0, 0, canvas.width, canvas.height);
	context.beginPath();
	
	//// Draw pendulum
	drawRIP(canvas, context, outputs);
	
	// request new frame
	requestAnimFrame(function() {
	  animate(canvas, context);
	});
}

/***
* Simulation model
***/
function ripTransition(Vm, prev_outputs, Tsamp, backlash)
{
	var GEAR_RATIO = 23.04;
	var STEPS_IN_SAMPLE = 100;
	var NUM_OUTPUTS = 8;
	var time_step;
	var theta_accel, alpha_accel, gamma_accel, gamma;
	var dz_function;
	var g1, g2, g5;
	var t, n;
	var sin_alpha, cos_alpha;
	var next_outputs = new Array(0, 0, 0, 0, 0, 0, 0, 0);
	
	// Define constant parameters
	var a1=0.0947, a2=0.0526, a4=0.0016; 
	var b=0.000425, c=0.00037, d=0.0199, e=0.0028, f=0.0088;
	var k=0.0000954, l=0.00000414, m=0.0055, n=0.003, p=0.000132;
	
	time_step = Tsamp / (1000 * STEPS_IN_SAMPLE);
	
	for (t=0; t<STEPS_IN_SAMPLE; t++)
	{
		// Calculate deadzone function value
		gamma = prev_outputs[6];
		if ((gamma >= backlash) || (gamma <= -backlash))
			dz_function = 1;
		else
			dz_function = 0;
			
		// Calculate dynamic parameters
		g1 = e + a4 * dz_function;
		g2 = f + dz_function * a2;
		g5 = dz_function * a1;

		sin_alpha = Math.sin(prev_outputs[4]);
		cos_alpha = Math.cos(prev_outputs[4]);
		
		// Calculate theta acceleration
		theta_accel = g5*Vm + (c*d/b)*sin_alpha*cos_alpha;
		theta_accel = theta_accel - c*prev_outputs[5]*prev_outputs[5]*sin_alpha - g2*prev_outputs[1];
		theta_accel = theta_accel / (g1 - (c*c/b)*cos_alpha*cos_alpha);

		// Calculate alpha acceleration
		alpha_accel = (c/b)*theta_accel*cos_alpha + (d/b)*sin_alpha;
		// Calculate gamma acceleration
		gamma_accel = (m*Vm - n*prev_outputs[1] - p*prev_outputs[7] - k*theta_accel) / l;
		
		// Calculate outputs
		next_outputs[1] = prev_outputs[1] + theta_accel*time_step; // theta dot
		next_outputs[0] = prev_outputs[0] + (next_outputs[1] + prev_outputs[1]) * time_step / 2; // theta
		next_outputs[5] = prev_outputs[5] + alpha_accel*time_step; // alpha dot
		next_outputs[4] = prev_outputs[4] + (next_outputs[5] + prev_outputs[5]) * time_step / 2; // alpha
		next_outputs[7] = prev_outputs[7] + gamma_accel*time_step; // gamma dot
		next_outputs[6] = gamma + (next_outputs[7] + prev_outputs[7]) * time_step / 2; // gamma
		
		// Saturate gamma
		if (next_outputs[6] >= backlash)
		{
			next_outputs[6] = backlash;
			next_outputs[7] = 0;
		}
		else if (next_outputs[6] <= -backlash)
		{
			next_outputs[6] = -backlash;
			next_outputs[7] = 0;
		}

		// Calculate remaining outputs
		next_outputs[2] = next_outputs[0]*GEAR_RATIO + next_outputs[6];
		next_outputs[3] = next_outputs[1]*GEAR_RATIO + next_outputs[7];
		
		// Store outputs as previous outputs
		prev_outputs = next_outputs;
	}
	
	return next_outputs;
}

runSfcExperiment();