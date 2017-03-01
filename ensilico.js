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
    this.preferences = {
        stepsize = 0.01,
        nominalViewDimension = 4
    };
    Platform.copyCorresponding(this.preferences, simulation.preferences());
}

Executive.minStepsize = function() {
    return 0.00001;
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

    // Prevent division by zero
    var stepsize = Math.max(this.preferences.stepsize, this.minStepsize());

    // Prevent an excessive number of simulation steps
    elapsedTime = Math.min(elapsedTime, this.maxElapsedTime());

    // Calculate remaining simulation time
    var simulationTime = elapsedTime - this.simulationLeadTime;

    // Calculate how many simulation steps are needed
    var numSteps = Math.ceil(simulationTime / stepsize);

    // Update simulation lead time for the next pass
    this.simulationLeadTime = numSteps * stepsize - simulationTime; //.............................
                for (var i = 0; i < numSteps; i++) {
                    this.config.simulation.update(this.config.stepsize);
                }
                //
                // Render a simple visualization
                //
                var canvas = this.config.domWindow.document.getElementById(this.config.drawingCanvasId);
                var context = canvas.getContext("2d");
                context.clearRect(0, 0, canvas.width, canvas.height);
                context.save();
                context.translate(this.config.drawingOriginX, this.config.drawingOriginY);
                context.scale(this.config.drawingScale, -this.config.drawingScale);
                context.beginPath();
                this.config.simulation.visualize(context, frameTime);
                context.restore();
                context.stroke();
    this.nextFrame();
}
