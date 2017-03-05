function Scalar() {}

Scalar.toRadians = function(degrees) {
    return degrees * Math.PI / 180;
}

// Returns the ratio or the specified upper limit
Scalar.rationalMin = function(numerator, denominator, maxRatio) {
    var fn = maxRatio * numerator;
    var fd = maxRatio * denominator;
    return fn / Math.max(numerator, fd);
}

// Returns next state
Scalar.lag = function(state, target, responsiveness, stepsize) {
    var k = responsiveness * stepsize;
    return (k * target + state) / (k + 1);
}

// Returns rate for integration
Scalar.lagRate = function(state, target, responsiveness, stepsize) {
    return responsiveness * (target - state) / (responsiveness * stepsize + 1);
}

function Pair() {
    this.x = 0;
    this.y = 0;
}

Pair.prototype.load = function(p) {
    this.x = p.x;
    this.y = p.y;
    return this;
}

Pair.prototype.dot = function(p) {
    return this.x * p.x + this.y * p.y;
}

Pair.prototype.add = function(p) {
    this.x += p.x;
    this.y += p.y;
    return this;
}

Pair.prototype.subtract = function(p) {
    this.x -= p.x;
    this.y -= p.y;
    return this;
}

// Add scalar product
Pair.prototype.addProduct = function(f, p) {
    this.x += f * p.x;
    this.y += f * p.y;
    return this;
}

// Subtract the projection of this onto p
// leaving only the perpendicular component of this
Pair.prototype.subtractProjection = function(p) {
    var f = this.dot(p) / (p.x * p.x + p.y * p.y);
    return this.addProduct(-f, p);
}

function Platform() {}

// Copies values from src for corresponding properties in dst
// Returns dst
Platform.softCopy = function(dst, src) {
    for (var key in src) {
        // The key must be in both objects
        if (src.hasOwnProperty(key) && dst.hasOwnProperty(key)) {
            var srcValue = src[key];
            var dstValue = dst[key];
            var srcType = typeof srcValue;
            var dstType = typeof dstValue;
            // Both values must have the same type
            if (srcType == dstType) {
                if (srcType != "object") {
                    dst[key] = srcValue;
                } else if (srcValue && dstValue) {
                    Platform.softCopy(dstValue, srcValue);
                }
            }
        }
    }
    return dst;
}

function Executive(simulation, canvasId, mainWindow) {
    this.simulation = simulation;
    this.canvasId = canvasId;
    this.mainWindow = mainWindow;
    this.previousTimestamp = null;
    this.simulationLeadTime = 0;

    // Default values
    var preferences = {
        visualScale: 100,
        stepsize: 0.01,
        maxStepsPerFrame: 100
    };

    // Allow simulation to override
    Platform.softCopy(preferences, simulation.preferences());

    this.visualScale = preferences.visualScale;
    this.controls = {
        stepsize: preferences.stepsize,
        holdSteady: false
    };
    this.maxStepsPerFrame = preferences.maxStepsPerFrame;
}

// A convenience method with a dependency on the window object
Executive.start = function(simulation, canvasId) {
    var executive = new Executive(simulation, canvasId, window);
    executive.startSimulation();
}

Executive.prototype.startSimulation = function() {
    this.previousTimestamp = this.mainWindow.performance.now();
    this.nextFrame();
}

Executive.prototype.nextFrame = function() {
    var self = this;
    this.mainWindow.requestAnimationFrame(function(timestamp) {
        self.update(timestamp);
    });
}

Executive.prototype.update = function(timestamp) {
    var elapsedTime = (timestamp - this.previousTimestamp) / 1000;
    this.previousTimestamp = timestamp;

    var stepsize = this.controls.stepsize;

    // Adjust for simulation lead from previous frame
    var simulationTime = elapsedTime - this.simulationLeadTime;

    // Number of steps needed to meet or exceed the adjusted time
    // Impose an upper limit because the browser might pause the animation indefinitely
    var numSteps = Math.ceil(Scalar.rationalMin(simulationTime, stepsize, this.maxStepsPerFrame));

    // Calculate adjustment for the next frame
    this.simulationLeadTime = Math.max(0, numSteps * stepsize - simulationTime);

    // Update the simulation
    for (var i = 0; i < numSteps; i++) {
        this.simulation.update(this.controls);
    }

    // Simple visualization
    var canvas = this.mainWindow.document.getElementById(this.canvasId);
    var w = canvas.width;
    var h = canvas.height;
    var context = canvas.getContext("2d");
    context.clearRect(0, 0, w, h);
    context.save();
    context.translate(w / 2, h / 2);
    context.scale(this.visualScale, -this.visualScale);
    context.beginPath();
    this.simulation.visualize(context, elapsedTime);
    context.restore();
    context.stroke();

    this.nextFrame();
}
