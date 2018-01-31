var Scalar = {};

// A bit larger than sqrt(MIN_NORMAL)
Scalar.tiny = function() {
    return 1e-154;
};

Scalar.toRadians = function(degrees) {
    return degrees * Math.PI / 180;
};

// Returns the ratio or the specified upper limit
Scalar.rationalMin = function(numerator, denominator, maxRatio) {
    var fn = maxRatio * numerator;
    var fd = maxRatio * denominator;
    return fn / Math.max(numerator, fd);
};

Scalar.lerp = function(f1, f2, t) {
    return (1 - t) * f1 + t * f2;
};

// Returns next state
Scalar.lag = function(state, target, responsiveness, stepsize) {
    var k = responsiveness * stepsize;
    return (k * target + state) / (k + 1);
};

// Returns rate for integration
Scalar.lagRate = function(state, target, responsiveness, stepsize) {
    return responsiveness * (target - state) / (responsiveness * stepsize + 1);
};

Scalar.integrate = function(state, rate, stepsize) {
    return stepsize * rate + state;
};

function Pair() {
    this.x = 0;
    this.y = 0;
}

Pair.ZERO = new Pair();

Pair.prototype.norm = function() {
    return Math.sqrt(this.dot(this));
};

Pair.prototype.dot = function(p) {
    return this.x * p.x + this.y * p.y;
};

Pair.prototype.cross = function(p) {
    return this.x * p.y - this.y * p.x;
}

Pair.prototype.load = function(p) {
    this.x = p.x;
    this.y = p.y;
    return this;
};

Pair.prototype.loadDelta = function(p1, p2) {
    this.x = p1.x - p2.x;
    this.y = p1.y - p2.y;
    return this;
};

Pair.prototype.loadPolar = function(r, theta) {
    this.x = r * Math.cos(theta);
    this.y = r * Math.sin(theta);
    return this;
};

// Load this with scalar product
Pair.prototype.loadProduct = function(f, p) {
    this.x = f * p.x;
    this.y = f * p.y;
    return this;
};

// Load this with the result of (0,0,z) X p
Pair.prototype.loadCrossProduct = function(z, p) {
    this.x = -z * p.y;
    this.y = z * p.x;
    return this;
};

Pair.prototype.loadMidpoint = function(p1, p2) {
    this.x = 0.5 * (p1.x + p2.x);
    this.y = 0.5 * (p1.y + p2.y);
    return this;
};

Pair.prototype.loadLerp = function(p1, p2, t) {
    this.x = (1 - t) * p1.x + t * p2.x;
    this.y = (1 - t) * p1.y + t * p2.y;
    return this;
};

Pair.prototype.negate = function() {
    this.x = -this.x;
    this.y = -this.y;
    return this;
};

Pair.prototype.multiplyBy = function(f) {
    this.x *= f;
    this.y *= f;
    return this;
};

// Caller is responsible for avoiding division by zero
Pair.prototype.divideBy = function(f) {
    var reciprocal = 1 / f;
    this.x *= reciprocal;
    this.y *= reciprocal;
    return this;
};

Pair.prototype.rotateBy = function(angle) {
    var c = Math.cos(angle);
    var s = Math.sin(angle);
    var x = this.x * c - this.y * s;
    var y = this.x * s + this.y * c;
    this.x = x;
    this.y = y;
    return this;
};

Pair.prototype.add = function(p) {
    this.x += p.x;
    this.y += p.y;
    return this;
};

// Add scalar product
Pair.prototype.addProduct = function(f, p) {
    this.x += f * p.x;
    this.y += f * p.y;
    return this;
};

Pair.prototype.subtract = function(p) {
    this.x -= p.x;
    this.y -= p.y;
    return this;
};

// Subtract scalar product
Pair.prototype.subtractProduct = function(f, p) {
    this.x -= f * p.x;
    this.y -= f * p.y;
    return this;
};

// Subtract the projection of this onto p
// leaving only the perpendicular component of this
Pair.prototype.subtractProjection = function(p) {
    var f = this.dot(p) / (p.x * p.x + p.y * p.y + Scalar.tiny());
    return this.subtractProduct(f, p);
};

Pair.prototype.normalize = function() {
    return this.divideBy(this.norm() + Scalar.tiny());
};

Pair.prototype.integrate = function(rate, stepsize) {
    this.x += stepsize * rate.x;
    this.y += stepsize * rate.y;
    return this;
};

function Particle() {
    // Default values
    this.mass = 0.005;
    this.drag = 0.001;
    this.buoyancy = 0.0005; // buoyancy to weight ratio in air
    this.position = new Pair();
    this.velocity = new Pair();

    // Use closure to avoid object allocation in inner loop
    var reusage = {
        force: new Pair()
    };
    this.reusage = function() {
        return reusage;
    };
}

Particle.prototype.update = function(stepsize, geosphere, externalForce) {
    var massRate = this.mass / (stepsize + Scalar.tiny());
    var densityRatio = geosphere.density(this.position);
    var drag = this.drag * densityRatio;
    var force = this.reusage().force;

    force
        .load(externalForce)
        .addProduct(this.mass * (1 - densityRatio * this.buoyancy), geosphere.gravity)
        .addProduct(massRate, this.velocity);
    var denominator = massRate + drag * this.velocity.norm();
    this.velocity.load(force).divideBy(denominator);
    this.position.integrate(this.velocity, stepsize);
};

// Pivots about the origin
function Flexor() {
    // Default values
    this.overallLength = 3;
    this.pivotOffset = 0.2;
    this.mass = 0.1;
    this.spring = 10;
    this.damping = 0.1;
    this.drag = 0.1;
    this.tipPosition = new Pair();
    this.alignTo(0);
    this.tipVelocity = new Pair();

    // Use closure to avoid object allocation in inner loop
    var reusage = {
        force: new Pair(),
        velocity: new Pair()
    };
    this.reusage = function() {
        return reusage;
    };
}

Flexor.prototype.radius = function() {
    return this.overallLength - this.pivotOffset;
};

Flexor.prototype.alignTo = function(angle) {
    this.tipPosition.loadPolar(this.radius(), angle);
    return this;
};

Flexor.prototype.update = function(stepsize, geosphere, externalForce, targetPosition, targetVelocity) {
    var massRate = this.mass / (stepsize + Scalar.tiny());
    var reusage = this.reusage();
    var force = reusage.force;
    var velocity = reusage.velocity;

    force
        .load(externalForce)
        .addProduct(this.mass, geosphere.gravity)
        .addProduct(massRate, this.tipVelocity)
        .addProduct(this.spring, targetPosition)
        .addProduct(this.damping, targetVelocity);
    force.subtractProjection(this.tipPosition);

    var denominator =
        massRate +
        stepsize * this.spring +
        this.damping +
        this.drag * this.tipVelocity.norm();
    velocity.load(force).divideBy(denominator);

    var norm = this.tipPosition.norm();
    var corrector = (this.radius() - norm) / (stepsize * norm + Scalar.tiny());
    velocity.addProduct(corrector, this.tipPosition);

    this.tipVelocity.load(velocity);
    this.tipPosition.integrate(velocity, stepsize);
};

function Filament(headPosition, tailPosition) {
    // Default values
    this.spacing = 0.2;
    this.mass = 0.0005;
    this.spring = 10;
    this.damping = 0.1;
    this.drag = 0.0001;
    this.buoyancy = 0.0005; // buoyancy to weight ratio in air

    var n = Filament.numSegments();
    this.position = [];
    this.velocity = [];
    for (var i = 0; i <= n; i++) {
        var p = new Pair();
        p.loadLerp(headPosition, tailPosition, i / n);
        this.position.push(p);
        this.velocity.push(new Pair());
    }

    this.upAxis = [];
    this.downAxis = [new Pair()]; // unused first element
    for (var i = 0; i < n; i++) {
        this.upAxis.push(new Pair());
        this.downAxis.push(new Pair());
    }
    this.updateAxes();

    // Use closure to avoid object allocation in inner loop
    var reusage = {
        deltaPosition: new Pair(),
        deltaVelocity: new Pair(),
        accumulator: new Pair(),
        leadForce: new Pair(),
        lagForce: new Pair()
    };
    this.reusage = function() {
        return reusage;
    };
}

Filament.numSegments = function() {
    return 20;
};

Filament.prototype.updateAxes = function() {
    var n = Filament.numSegments();
    for (var i = 0; i < n; i++) {
        this.upAxis[i]
            .loadDelta(this.position[i+1], this.position[i])
            .normalize();
        this.downAxis[i+1]
            .load(this.upAxis[i])
            .negate();
    }
};

Filament.prototype.scalarForce = function(i, sense, axis) {
    var reusage = this.reusage();
    var deltaPosition = reusage.deltaPosition;
    var deltaVelocity = reusage.deltaVelocity;
    var accumulator = reusage.accumulator;

    deltaPosition.loadDelta(this.position[i+sense], this.position[i]);
    deltaVelocity.loadDelta(this.velocity[i+sense], this.velocity[i]);
    accumulator
        .loadProduct(this.spring, deltaPosition)
        .addProduct(this.damping, deltaVelocity);
    return accumulator.dot(axis[i]) - this.spring * this.spacing;
};

Filament.prototype.storeHeadForce = function(geosphere, headForce) {
    var densityRatio = geosphere.density(this.position);
    headForce
        .loadProduct(this.mass * (1 - densityRatio * this.buoyancy), geosphere.gravity)
        .addProduct(this.scalarForce(0, 1, this.upAxis), this.upAxis[0]);
};

Filament.prototype.storeTailForce = function(geosphere, tailForce) {
    var densityRatio = geosphere.density(this.position);
    var n = Filament.numSegments();
    tailForce
        .loadProduct(this.mass * (1 - densityRatio * this.buoyancy), geosphere.gravity)
        .addProduct(this.scalarForce(n, -1, this.downAxis), this.downAxis[n]);
};

Filament.prototype.update = function(stepsize, geosphere, headPosition, headVelocity, tailPosition, tailVelocity) {
    var n = Filament.numSegments();
    this.position[0].load(headPosition);
    this.velocity[0].load(headVelocity);
    this.position[n].load(tailPosition);
    this.velocity[n].load(tailVelocity);
    this.updateAxes();

    var halfstep = 0.5 * stepsize;
    var massRate = this.mass / (halfstep + Scalar.tiny());
    var lumped = this.spring * halfstep + this.damping;
    this.updateCore(halfstep, geosphere, massRate, lumped, 0, n, 1, this.upAxis);
    this.updateCore(halfstep, geosphere, massRate, lumped, n, 0, -1, this.downAxis);
};

Filament.prototype.updateCore = function(stepsize, geosphere, massRate, lumped, start, end, sense, axis) {
    var reusage = this.reusage();
    var leadForce = reusage.leadForce;
    var lagForce = reusage.lagForce;
    var accumulator = reusage.accumulator;

    for (var i = start + sense; i != end; i += sense) {
        var densityRatio = geosphere.density(this.position[i]);
        leadForce
            .loadDelta(this.position[i+sense], this.position[i])
            .multiplyBy(this.spring)
            .addProduct(this.damping, this.velocity[i+sense]);
        lagForce
            .loadProduct(massRate, this.velocity[i])
            .addProduct(this.mass * (1 - densityRatio * this.buoyancy), geosphere.gravity)
            .subtractProduct(this.scalarForce(i-sense, sense, axis), axis[i-sense])
            .subtractProduct(this.spring * this.spacing, axis[i]);
        var fluid = this.drag * densityRatio * this.velocity[i].norm();
        accumulator
            .loadProduct(massRate + fluid, leadForce)
            .subtractProduct(lumped - fluid, lagForce)
            .divideBy(massRate + lumped);
        this.velocity[i]
            .loadProduct(accumulator.dot(axis[i]), axis[i])
            .add(lagForce)
            .divideBy(massRate + fluid);
        this.position[i].integrate(this.velocity[i], stepsize);
    }
};

Filament.prototype.visualize = function(context, elapsedTime) {
    var accumulator = this.reusage().accumulator;

    context.moveTo(this.position[0].x, this.position[0].y);
    var len = Filament.numSegments() + 1;
    for (var i = 0; i < len; i++) {
        accumulator.loadMidpoint(this.position[i], this.position[Math.min(i+1, len-1)]);
        context.quadraticCurveTo(
            this.position[i].x,
            this.position[i].y,
            accumulator.x,
            accumulator.y);
    }
};

var Platform = {};

// Copies values from src for corresponding properties in dst
// Returns dst
// Optional log handler
Platform.softCopy = function(dst, src, log) {
    for (var key in src) {
        var srcOwn = src.hasOwnProperty(key);
        var dstOwn = dst.hasOwnProperty(key);
        // The key must be in both objects
        if (srcOwn && dstOwn) {
            var srcValue = src[key];
            var dstValue = dst[key];
            var srcType = typeof srcValue;
            var dstType = typeof dstValue;
            // Both values must have the same type
            if (srcType == dstType) {
                if (srcType != "object") {
                    dst[key] = srcValue;
                } else if (srcValue && dstValue) {
                    Platform.softCopy(dstValue, srcValue, log);
                } else if (log) {
                    if (!srcValue) {
                        log("src error at " + key);
                    }
                    if (!dstValue) {
                        log("dst error at " + key);
                    }
                }
            } else if (log) {
                log("type mismatch at " + key);
            }
        } else if (log) {
            if (!srcOwn) {
                log("skip src key " + key);
            }
            if (!dstOwn) {
                log("no dst key " + key);
            }
        }
    }
    return dst;
};

Platform.getJson = function(url, success) {
    if (!url) {
        success({});
    } else {
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
};

Platform.load = function(dst, url, log, onLoad, getJsonArg) {
    var getJson = getJsonArg || Platform.getJson;
    getJson(url, function(json) {
        Platform.softCopy(dst, json, log);
        onLoad(dst);
    });
}

var FrameTimer = (function() {
    var listeners = [];
    var mainWindow = null;
    var previousTimestamp = 0;

    function nextFrame() {
        mainWindow.requestAnimationFrame(onFrame);
    }

    function onFrame(timestamp) {
        var elapsedTime = (timestamp - previousTimestamp) / 1000;
        previousTimestamp = timestamp;
        var len = listeners.length;
        for (var i = 0; i < len; i++) {
            listeners[i](elapsedTime);
        }
        nextFrame();
    }

    return {
        addListener: function(windowArg, listener) {
            listeners.push(listener);
            if (!mainWindow) {
                mainWindow = windowArg;
                previousTimestamp = windowArg.performance.now();
                nextFrame();
            }
        }
    };
})();

var Executive = (function() {
    var agents = {};

    function Agent(simulation, canvas, log) {
        this.simulation = simulation;
        this.context = canvas.getContext("2d");
        this.simulationLeadTime = 0;
        this.onBreakFrame = null;
        this.clutch = new Clutch();

        // Default values
        var preferences = {
            visualScale: 100,
            stepsize: 0.01,
            maxStepsPerFrame: 100
        };

        // Allow simulation to override
        Platform.softCopy(preferences, simulation.preferences, log);

        this.visualScale = preferences.visualScale;
        this.controls = {
            stepsize: preferences.stepsize,
            pointer: {
                contact: 0,
                velocity: new Pair()
            }
        };
        this.maxStepsPerFrame = preferences.maxStepsPerFrame;

        this.registerListeners(canvas);
    }

    Agent.prototype.registerListeners = function(canvas) {
        var clutch = this.clutch;

        canvas.addEventListener("mousedown", function(event) {
            clutch.onEngage(event.clientX, event.clientY);
        }, false);
        canvas.addEventListener("mousemove", function(event) {
            clutch.onMove(event.clientX, event.clientY);
        }, false);
        canvas.addEventListener("mouseup", function() {
            clutch.onDisengage();
        }, false);

        canvas.addEventListener("touchstart", function(event) {
            var touch = event.touches[0];
            clutch.onEngage(touch.clientX, touch.clientY);
        }, false);
        canvas.addEventListener("touchmove", function(event) {
            var touch = event.touches[0];
            clutch.onMove(touch.clientX, touch.clientY);
        }, false);
        canvas.addEventListener("touchend", function() {
            clutch.onDisengage();
        }, false);
    };

    Agent.prototype.onFrame = function(elapsedTime) {
        var stepsize = this.controls.stepsize;

        // Adjust for simulation lead from previous frame
        var simulationTime = elapsedTime - this.simulationLeadTime;

        // Number of steps needed to meet or exceed the adjusted time
        // Impose an upper limit because the browser might pause the animation indefinitely
        var numSteps = Math.ceil(Scalar.rationalMin(simulationTime, stepsize, this.maxStepsPerFrame));

        // Calculate adjustment for the next frame
        var period = numSteps * stepsize;
        this.simulationLeadTime = Math.max(0, period - simulationTime);

        // Calculate pointer velocity
        var contact = this.clutch.engaged ? 1 : 0;
        this.controls.pointer.contact = contact;
        this.controls.pointer.velocity
            .loadDelta(this.clutch.currentPosition, this.clutch.basePosition)
            .multiplyBy(contact / (period + Scalar.tiny()));
        this.clutch.rebase();

        // Update the simulation
        this.updateSimulation(numSteps);

        // Simple visualization
        this.visualizeSimulation(elapsedTime);
    };

    Agent.prototype.updateSimulation = function(numSteps) {
        if (this.onBreakFrame) {
            this.onBreakFrame(this.simulation);
            this.onBreakFrame = null;
        }

        for (var i = 0; i < numSteps; i++) {
            this.simulation.update(this.controls);
        }
    };

    Agent.prototype.visualizeSimulation = function(elapsedTime) {
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
    };

    return {
        // Simulation to canvas is assumed to be 1:1
        // Starting an id already started is neither supported nor defined
        // Optional windowArg is for dependency injection
        start: function(id, simulation, url, windowArg) {
            var mainWindow = windowArg || window;
            var log = function(msg) {
                mainWindow.console.log(msg);
            };
            var canvas = mainWindow.document.getElementById(id);
            var agent = new Agent(simulation, canvas, log);
            agents[id] = agent;
            Platform.load(simulation, url, log, function() {
                FrameTimer.addListener(mainWindow, function(elapsedTime) {
                    agent.onFrame(elapsedTime);
                });
            });
        },
        breakFrame: function(id, onBreakFrame, debugArg) {
            var debug = debugArg || function() {
                debugger;
            };
            agents[id].onBreakFrame = function(sim) {
                onBreakFrame(sim);
                debug();
            };
        }
    };
})();

function Clutch() {
    this.engaged = false;
    this.currentPosition = new Pair();
    this.basePosition = new Pair();
}

Clutch.prototype.onEngage = function(x, y) {
    this.engaged = true;
    this.currentPosition.x = x;
    this.currentPosition.y = y;
    this.rebase();
};

Clutch.prototype.onMove = function(x, y) {
    this.currentPosition.x = x;
    this.currentPosition.y = y;
};

Clutch.prototype.onDisengage = function() {
    this.engaged = false;
};

Clutch.prototype.rebase = function() {
    this.basePosition.load(this.currentPosition);
};
