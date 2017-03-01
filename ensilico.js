function Platform() {}

// Copies values from src for corresponding properties in dst.
// Returns dst.
Platform.softCopy(dst, src) {
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
                    copyCorresponding(dstValue, srcValue);
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
        stepsize = 0.01,
        visualScale = 100,
        minStepsize = 0.00001, //TODO: replace both of these with maxNumSteps
        maxElapsedTime = 0.05  //TODO: replace both of these with maxNumSteps
    };

    // Allow simulation to override (typically just stepsize and visualScale)
    Platform.softCopy(preferences, simulation.preferences());

    this.stepsize = preferences.stepsize;
    this.visualScale = preferences.visualScale;

    // Needed because the browser might pause the animation indefinitely
    this.minStepsize = preferences.minStepsize;
    this.maxElapsedTime = preferences.maxElapsedTime;
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

    // Avoid excessive catch-up after a long pause
    elapsedTime = Math.min(elapsedTime, this.maxElapsedTime);

    // Adjust for simulation lead in previous frame
    var simulationTime = elapsedTime - this.simulationLeadTime;

    // Number of steps needed to meet or exceed the adjusted time
    var numSteps = Math.ceil(simulationTime / Math.max(this.stepsize, this.minStepsize));

    // Calculate adjustment for the next frame
    this.simulationLeadTime += numSteps * this.stepsize - elapsedTime;

    // Update the simulation
    for (var i = 0; i < numSteps; i++) {
        this.simulation.update(this.stepsize);
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
