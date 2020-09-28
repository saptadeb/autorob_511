function update_pendulum_state(numerical_integrator, pendulum, dt, gravity) {
    // integrate pendulum state forward in time by dt
    // please use names 'pendulum.angle', 'pendulum.angle_previous', etc. in else codeblock between line 28-30

    if (typeof numerical_integrator === "undefined")
        numerical_integrator = "none";

    if (numerical_integrator === "euler") {

    // STENCIL: a correct Euler integrator is REQUIRED for assignment
        pendulum.angle_previous = pendulum.angle;
        var temp_acc = pendulum_acceleration(pendulum, gravity)

        pendulum.angle = pendulum.angle + pendulum.angle_dot * dt;
        pendulum.angle_dot = pendulum.angle_dot + temp_acc * dt;

    }
    else if (numerical_integrator === "verlet") {

    // STENCIL: basic Verlet integration
        var temp_acc = pendulum_acceleration(pendulum, gravity)
        var temp_angle = pendulum.angle

        pendulum.angle = 2 * pendulum.angle - pendulum.angle_previous + temp_acc * Math.pow(dt, 2)
        pendulum.angle_dot = (pendulum.angle - pendulum.angle_previous)/(2 * dt)

        pendulum.angle_previous = temp_angle

    }
    else if (numerical_integrator === "velocity verlet") {

    // STENCIL: a correct velocity Verlet integrator is REQUIRED for assignment
        pendulum.angle_previous = pendulum.angle;
        var temp_acc = pendulum_acceleration(pendulum, gravity)

        pendulum.angle = pendulum.angle + pendulum.angle_dot * dt + ((temp_acc * Math.pow(dt, 2)) / 2)
        pendulum.angle_dot = pendulum.angle_dot + (((temp_acc + pendulum_acceleration(pendulum, gravity)) * dt) / 2)

    }
    else if (numerical_integrator === "runge-kutta") {

    // STENCIL: Runge-Kutta 4 integrator
        var b1 = 1/6
        var b2 = 1/3
        var b3 = 1/3
        var b4 = 1/6
        var a21 = 1/2
        var a32 = 1/2
        var a43 = 1

        kx1 = pendulum.angle
        kv1 = pendulum.angle_dot
        a_kx1 = pendulum_acceleration(pendulum, gravity)
        kx2 = kx1 + (a21 * kv1 * dt)
        kv2 = kv1 + (a21 * a_kx1 * dt)
        pendulum.angle = kx2
        a_kx2 = pendulum_acceleration(pendulum, gravity)
        kx3 = kx1 + (a32 * kv2 * dt)
        kv3 = kv1 + (a32 * a_kx2 * dt)
        pendulum.angle = kx3
        a_kx3 = pendulum_acceleration(pendulum, gravity)
        kx4 = kx1 + (a43 * kv3 * dt)
        kv4 = kv1 + (a43 * a_kx3 * dt)
        pendulum.angle = kx4
        a_kx4 = pendulum_acceleration(pendulum, gravity)

        pendulum.angle = kx1 + dt * (b1 * kv1 + b2 * kv2 + b3 * kv3 + b4 * kv4)
        pendulum.angle_dot = kv1 + dt * (b1 * a_kx1 + b2 * a_kx2 + b3 * a_kx3 + b4 * a_kx4)
        pendulum.angle_previous = kx1

    } 
    else {
        pendulum.angle_previous = pendulum.angle;
        pendulum.angle = (pendulum.angle+Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot = (pendulum.angle-pendulum.angle_previous)/dt;
        numerical_integrator = "none";
    }

    return pendulum;
}

function pendulum_acceleration(pendulum, gravity) {
    // STENCIL: return acceleration(s) system equation(s) of motion 
    var dot_dot = -(gravity/pendulum.length)*Math.sin(pendulum.angle) + pendulum.control/(pendulum.mass * Math.pow(pendulum.length,2))
    return dot_dot
}

function init_verlet_integrator(pendulum, t, gravity) {
    // STENCIL: for verlet integration, a first step in time is needed
    // return: updated pendulum state and time

    pendulum.angle_previous = pendulum.angle
    pendulum.angle = pendulum.angle + dt * pendulum.angle_dot + Math.pow(dt, 2) * pendulum_acceleration(pendulum, gravity) * 0.5
    t = t + dt
    return [pendulum, t];
}

function set_PID_parameters(pendulum) {
    // STENCIL: change pid parameters
    // pendulum.servo = {kp:0, kd:0, ki:0};  // no control
    // pendulum.servo = {kp:40, kd:6, ki:0.2};  
    pendulum.servo = {kp:500, kd:200, ki:5};  

    return pendulum;
}

function PID(pendulum, accumulated_error, dt) {
    // STENCIL: implement PID controller
    // return: updated output in pendulum.control and accumulated_error

    // var gains = pendulum.servo
    var temp_error_desired = pendulum.desired - pendulum.angle
    var temp_error_current = pendulum.angle_previous - pendulum.angle 

    accumulated_error = accumulated_error + temp_error_desired
    pendulum.control = pendulum.servo.kp * temp_error_desired + pendulum.servo.ki * accumulated_error + pendulum.servo.kd * (temp_error_current/dt)

    return [pendulum, accumulated_error];
}