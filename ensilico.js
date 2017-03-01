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

Executive.start = function(simulation, canvasId) {
    var executive = new Executive(simulation, canvasId, window);
    executive.startSimulation();
}

Executive.prototype.startSimulation = function() {
    this.previousTimestamp = this.mainWindow.performance.now();
    this.nextFrame(); // LEFT OFF HERE................................
}

Executive.prototype.nextFrame = function() {
    var self = this;
    this.mainWindow.requestAnimationFrame(function(timestamp) {
        self.update(timestamp);
    });
}
