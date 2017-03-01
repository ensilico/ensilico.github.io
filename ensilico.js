function Platform() {}

// Copies values from src for corresponding properties in dst.
// Returns dst.
Platform.copyCorresponding(dst, src) {
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

    // Allow simulation to override the following default values
    var preferences = {
        stepsize = 0.01,
        nominalViewDimension = 4
    };
    Platform.copyCorresponding(preferences, simulation.preferences());

    // Avoid division by zero
    var minStepsize = 0.00001;
    var minViewDimension = 0.000001;

    this.stepsize = Math.max(preferences.stepsize, minStepsize);
    this.nominalViewDimension = Math.max(preferences.nominalViewDimension, minViewDimension);
}

Executive.maxElapsedTime = function() {
    return 0.05;
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

    // Avoid excessive catch-up
    elapsedTime = Math.min(elapsedTime, this.maxElapsedTime());

    // Adjust for simulation lead
    var simulationTime = elapsedTime - this.simulationLeadTime;

    // Steps needed to meet or exceed the adjusted time
    var numSteps = Math.ceil(simulationTime / this.stepsize);

    // Calculate adjustment for the next frame
    this.simulationLeadTime += numSteps * this.stepsize - elapsedTime;

    for (var i = 0; i < numSteps; i++) {
        this.simulation.update(this.stepsize);
    }

    // Simple visualization
    var canvas = this.mainWindow.document.getElementById(this.canvasId);
    var w = canvas.width;
    var h = canvas.height;
    var scale = Math.min(w, h) / this.nominalViewDimension;
    var context = canvas.getContext("2d");
    context.clearRect(0, 0, w, h);
    context.save();
    context.translate(w / 2, h / 2);
    context.scale(scale, -scale);
    context.beginPath();
    this.simulation.visualize(context, elapsedTime);
    context.restore();
    context.stroke();

    this.nextFrame();
}
