function Scalar() {}

// A bit larger than sqrt(MIN_NORMAL)
Scalar.tiny = function() {
    return 1e-154;
}

Scalar.toRadians = function(degrees) {
    return degrees * Math.PI / 180;
}

// Returns the ratio or the specified upper limit
Scalar.rationalMin = function(numerator, denominator, maxRatio) {
    var fn = maxRatio * numerator;
    var fd = maxRatio * denominator;
    return fn / Math.max(numerator, fd);
}

Scalar.lerp = function(f1, f2, t) {
    return (1 - t) * f1 + t * f2;
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

Pair.ZERO = new Pair();

Pair.prototype.norm = function() {
    return Math.sqrt(this.dot(this));
}

Pair.prototype.dot = function(p) {
    return this.x * p.x + this.y * p.y;
}

Pair.prototype.load = function(p) {
    this.x = p.x;
    this.y = p.y;
    return this;
}

Pair.prototype.loadDelta = function(p1, p2) {
    this.x = p1.x - p2.x;
    this.y = p1.y - p2.y;
    return this;
}

Pair.prototype.loadPolar = function(r, theta) {
    this.x = r * Math.cos(theta);
    this.y = r * Math.sin(theta);
    return this;
}

// Load this with scalar product
Pair.prototype.loadProduct = function(f, p) {
    this.x = f * p.x;
    this.y = f * p.y;
    return this;
}

// Load this with the result of (0,0,z) X p
Pair.prototype.loadCrossProduct = function(z, p) {
    this.x = -z * p.y;
    this.y = z * p.x;
    return this;
}

Pair.prototype.loadMidpoint = function(p1, p2) {
    this.x = 0.5 * (p1.x + p2.x);
    this.y = 0.5 * (p1.y + p2.y);
    return this;
}

Pair.prototype.loadLerp = function(p1, p2, t) {
    this.x = (1 - t) * p1.x + t * p2.x;
    this.y = (1 - t) * p1.y + t * p2.y;
    return this;
}

Pair.prototype.multiplyBy = function(f) {
    this.x *= f;
    this.y *= f;
    return this;
}

// Caller is responsible for avoiding division by zero
Pair.prototype.divideBy = function(f) {
    var reciprocal = 1 / f;
    this.x *= reciprocal;
    this.y *= reciprocal;
    return this;
}

Pair.prototype.rotateBy = function(angle) {
    var c = Math.cos(angle);
    var s = Math.sin(angle);
    var x = this.x * c - this.y * s;
    var y = this.x * s + this.y * c;
    this.x = x;
    this.y = y;
    return this;
}

Pair.prototype.add = function(p) {
    this.x += p.x;
    this.y += p.y;
    return this;
}

// Add scalar product
Pair.prototype.addProduct = function(f, p) {
    this.x += f * p.x;
    this.y += f * p.y;
    return this;
}

Pair.prototype.subtract = function(p) {
    this.x -= p.x;
    this.y -= p.y;
    return this;
}

// Subtract scalar product
Pair.prototype.subtractProduct = function(f, p) {
    this.x -= f * p.x;
    this.y -= f * p.y;
    return this;
}

// Subtract the projection of this onto p
// leaving only the perpendicular component of this
Pair.prototype.subtractProjection = function(p) {
    var f = this.dot(p) / (p.x * p.x + p.y * p.y + Scalar.tiny());
    return this.subtractProduct(f, p);
}

Pair.prototype.normalize = function() {
    return this.divideBy(this.norm() + Scalar.tiny());
}

// Pivots about the origin
function Rod(properties) {
    // Default values
    this.rodLength = 3;
    this.pivotOffset = 0.2;
    this.flexMass = 0.1;
    this.flexSpring = 10;
    this.flexDamping = 1;
    this.flexDrag = 1;
    this.tipPosition = new Pair();
    this.alignTo(0);
    this.tipVelocity = new Pair();

    // Allow caller to override
    Platform.softCopy(this, properties);

    // Avoid object allocation in inner loop
    this.reusage = {
        force: new Pair(),
        velocity: new Pair()
    };
}

Rod.prototype.radius = function() {
    return this.rodLength - this.pivotOffset;
}

Rod.prototype.alignTo = function(angle) {
    this.tipPosition.loadPolar(this.radius(), angle);
    return this;
}

Rod.prototype.update = function(stepsize, gravity, externalForce, targetPosition, targetVelocity) {
    var massRate = this.flexMass / (stepsize + Scalar.tiny());

    var force = this.reusage.force;
    force
        .load(externalForce)
        .addProduct(this.flexMass, gravity)
        .addProduct(massRate, this.tipVelocity)
        .addProduct(this.flexSpring, targetPosition)
        .addProduct(this.flexDamping, targetVelocity);
    force.subtractProjection(this.tipPosition);

    var denominator =
        massRate +
        stepsize * this.flexSpring +
        this.flexDamping +
        this.flexDrag * this.tipVelocity.norm();
    var velocity = this.reusage.velocity;
    velocity.load(force).divideBy(denominator);

    var norm = this.tipPosition.norm();
    var corrector = (this.radius() - norm) / (stepsize * norm + Scalar.tiny());
    velocity.addProduct(corrector, this.tipPosition);

    this.tipVelocity.load(velocity);
    this.tipPosition.addProduct(stepsize, velocity);
}

function Wire(properties, bottomEndPosition) {
    // Default values
    this.numSegments = 20;
    this.spacing = 0.2;
    this.mass = 0.0004;
    this.spring = 10;
    this.damping = 1;
    this.drag = 0.001;

    // Allow caller to override
    Platform.softCopy(this, properties);

    this.position = [];
    this.velocity = [];
    for (var i = 0; i <= this.numSegments; i++) {
        var p = new Pair();
        p.load(bottomEndPosition).add({
            x: i * this.spacing,
            y: 0
        });
        this.position.push(p);

        this.velocity.push(new Pair());
    }

    this.axis = [];
    for (var i = 0; i < this.numSegments; i++) {
        this.axis.push(new Pair());
    }
    this.updateAxes();

    // Avoid object allocation in inner loop
    this.reusage = {
        deltaPosition: new Pair(),
        deltaVelocity: new Pair(),
        accumulator: new Pair(),
        upperForce: new Pair(),
        lowerForce: new Pair()
    };
}

Wire.prototype.updateAxes = function() {
    for (var i = 0; i < this.numSegments; i++) {
        this.axis[i]
            .loadDelta(this.position[i+1], this.position[i])
            .normalize();
    }
}

// Assume i to be greater than or equal to 0 and less than this.numSegments
Wire.prototype.scalarForce = function(i) {
    this.reusage.deltaPosition.loadDelta(this.position[i+1], this.position[i]);
    this.reusage.deltaVelocity.loadDelta(this.velocity[i+1], this.velocity[i]);
    this.reusage.accumulator
        .loadProduct(this.spring, this.reusage.deltaPosition)
        .addProduct(this.damping, this.reusage.deltaVelocity);
    return this.reusage.accumulator.dot(this.axis[i]) - this.spring * this.spacing;
}

Wire.prototype.storeBottomEndForce = function(gravity, bottomEndForce) {
    bottomEndForce.loadProduct(this.mass, gravity).addProduct(this.scalarForce(0), this.axis[0]);
}

Wire.prototype.storeTopEndForce = function(gravity, topEndForce) {
    var i = this.numSegments - 1;
    topEndForce.loadProduct(this.mass, gravity).subtractProduct(this.scalarForce(i), this.axis[i]);
}

Wire.prototype.update = function(stepsize, gravity, bottomEndPosition, bottomEndVelocity, topEndPosition, topEndVelocity) {
    this.position[0].load(bottomEndPosition);
    this.velocity[0].load(bottomEndVelocity);
    this.position[this.numSegments].load(topEndPosition);
    this.velocity[this.numSegments].load(topEndVelocity);
    this.updateAxes();

    var massRate = this.mass / (stepsize + Scalar.tiny());
    var lumped = this.spring * stepsize + this.damping;

    for (var i = 1; i < this.numSegments; i++) {
        this.reusage.upperForce
            .loadDelta(this.position[i+1], this.position[i])
            .multiplyBy(this.spring)
            .addProduct(this.damping, this.velocity[i+1]);
        this.reusage.lowerForce
            .loadProduct(massRate, this.velocity[i])
            .addProduct(this.mass, gravity)
            .subtractProduct(this.scalarForce(i-1), this.axis[i-1])
            .subtractProduct(this.spring * this.spacing, this.axis[i]);
        var fluid = this.drag * this.velocity[i].norm();
        this.reusage.accumulator
            .loadProduct(massRate + fluid, this.reusage.upperForce)
            .subtractProduct(lumped - fluid, this.reusage.lowerForce)
            .divideBy(massRate + lumped);
        this.velocity[i]
            .loadProduct(this.reusage.accumulator.dot(this.axis[i]), this.axis[i])
            .add(this.reusage.lowerForce)
            .divideBy(massRate + fluid);
        this.position[i].addProduct(stepsize, this.velocity[i]);
    }
}

Wire.prototype.visualize = function(context, elapsedTime) {
    context.moveTo(this.position[0].x, this.position[0].y);
    var len = this.numSegments + 1;
    for (var i = 0; i < len; i++) {
        this.reusage.accumulator.loadMidpoint(this.position[i], this.position[Math.min(i+1, len-1)]);
        context.quadraticCurveTo(
            this.position[i].x,
            this.position[i].y,
            this.reusage.accumulator.x,
            this.reusage.accumulator.y);
    }
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

Platform.getJson = function(url, success) {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", url, true);
    xhr.onreadystatechange = function() {
        if (xhr.readyState == 4 && xhr.status == 200) {
            var json = JSON.parse(xhr.responseText);
            success(json);
        }
    };
    xhr.send();
}

function Executive(simulation, context) {
    this.simulation = simulation;
    this.context = context;
    this.simulationLeadTime = 0;
    this.onBreakFrame = null;

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

Executive.previousTimestamp = null;

Executive.instances = {};

// An attempt to centralize the dependency
Executive.mainWindow = function() {
    return window;
}

Executive.breakFrame = function(simulationId, onBreakFrame) {
    Executive.instances[simulationId].onBreakFrame = onBreakFrame;
}

Executive.start = function(simulationId, simulation, canvasId) {
    var canvas = Executive.mainWindow().document.getElementById(canvasId);
    var context = canvas.getContext("2d");
    Executive.instances[simulationId] = new Executive(simulation, context);
    if (!Executive.previousTimestamp) {
        Executive.previousTimestamp = Executive.mainWindow().performance.now();
        Executive.nextFrame();
    }
}

Executive.startFromJson = function(simulationId, simulation, canvasId, url) {
    Platform.getJson(url, function(json) {
        Platform.softCopy(simulation, json);
        Executive.start(simulationId, simulation, canvasId);
    });
}

Executive.nextFrame = function() {
    Executive.mainWindow().requestAnimationFrame(Executive.onFrame);
}

Executive.onFrame = function(timestamp) {
    var elapsedTime = (timestamp - Executive.previousTimestamp) / 1000;
    Executive.previousTimestamp = timestamp;
    for (var simulationId in Executive.instances) {
        if (Executive.instances.hasOwnProperty(simulationId)) {
            Executive.instances[simulationId].onFrame(elapsedTime);
        }
    }
    Executive.nextFrame();
}

Executive.prototype.onFrame = function(elapsedTime) {
    var stepsize = this.controls.stepsize;

    // Adjust for simulation lead from previous frame
    var simulationTime = elapsedTime - this.simulationLeadTime;

    // Number of steps needed to meet or exceed the adjusted time
    // Impose an upper limit because the browser might pause the animation indefinitely
    var numSteps = Math.ceil(Scalar.rationalMin(simulationTime, stepsize, this.maxStepsPerFrame));

    // Calculate adjustment for the next frame
    this.simulationLeadTime = Math.max(0, numSteps * stepsize - simulationTime);

    // Update the simulation
    this.updateSimulation(numSteps);

    // Simple visualization
    this.visualizeSimulation(elapsedTime);
}

Executive.prototype.updateSimulation = function(numSteps) {
    if (this.onBreakFrame) {
        this.onBreakFrame(this.simulation);
        this.onBreakFrame = null;
        debugger;
    }

    for (var i = 0; i < numSteps; i++) {
        this.simulation.update(this.controls);
    }
}

Executive.prototype.visualizeSimulation = function(elapsedTime) {
    var context = this.context;
    var w = context.canvas.width;
    var h = context.canvas.height;
    context.clearRect(0, 0, w, h);
    context.save();
    context.translate(w / 2, h / 2);
    context.scale(this.visualScale, -this.visualScale);
    context.beginPath();
    this.simulation.visualize(context, elapsedTime);
    context.restore();
    context.stroke();
}
