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

function Executive(simulation, canvasId) {
    this.simulation = simulation;
    this.canvasId = canvasId;
}

Executive.start = function(simulation, canvasId) {
    var executive = new Executive(simulation, canvasId);
    executive.startSimulation();
}

Executive.prototype.startSimulation = function() {
    // Default values for simulation preferences
    var preferences = {
        stepsize = 0.01,
        minViewDimension = 4
    };
    
    // Override defaults
    Platform.copyCorresponding(preferences, this.simulation.preferences());
}
