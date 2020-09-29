function update_pendulum_state(numerical_integrator, pendulum, dt, gravity) {
    // integrate pendulum state forward in time by dt
    

    if (typeof numerical_integrator === "undefined")
        numerical_integrator = "none";

    if (numerical_integrator === "euler") {

    // STENCIL: a correct Euler integrator is REQUIRED for assignment
        pendulum.angle_previous = pendulum.angle
        //pendulum.angle_previous[1] = pendulum.angle[1];
        // pendulum.angle_dot_previous = pendulum.angle_dot
        var temp_acc = pendulum_acceleration(pendulum, gravity)  
        pendulum.angle[0] = pendulum.angle_previous[0] + pendulum.angle_dot[0] * dt
        pendulum.angle[1] = pendulum.angle_previous[1] + pendulum.angle_dot[1] * dt
        pendulum.angle_dot[0] = pendulum.angle_dot[0] + dt * temp_acc[0]
        pendulum.angle_dot[1] = pendulum.angle_dot[1] + dt * temp_acc[1]

    }
    else if (numerical_integrator === "verlet") {

    // STENCIL: basic Verlet integration
        var temp_angle = pendulum.angle
        var temp_acc =  pendulum_acceleration(pendulum, gravity)
        pendulum.angle[0] = 2* pendulum.angle[0] - pendulum.angle_previous[0] + dt * dt * temp_acc[0];
        pendulum.angle[1] = 2* pendulum.angle[1] - pendulum.angle_previous[1] + dt * dt * temp_acc[1];

        pendulum.angle_dot[0] = pendulum.angle_dot[0] + dt * temp_acc[0]
        pendulum.angle_dot[1] = pendulum.angle_dot[1] + dt * temp_acc[1]

        
        // pendulum.angle[0] = 2 * pendulum.angle[0] - pendulum.angle_previous[0] + temp_acc[0] * Math.pow(dt, 2)
        // pendulum.angle[1] = 2 * pendulum.angle[1] - pendulum.angle_previous[1] + temp_acc[1] * Math.pow(dt, 2)

        // pendulum.angle_dot[0] = (pendulum.angle[0] - pendulum.angle_previous[0])/(2 * dt)
        // pendulum.angle_dot[1] = (pendulum.angle[1] - pendulum.angle_previous[1])/(2 * dt)

        pendulum.angle_previous = temp_angle

    }
    else if (numerical_integrator === "velocity verlet") {

    // STENCIL: a correct velocity Verlet integrator is REQUIRED for assignment

    }
    else if (numerical_integrator === "runge-kutta") {

    // STENCIL: Runge-Kutta 4 integrator

       
        a21 = 1.0 / 2;
        a32 = 1.0 / 2;
        a43 = 1.0;
        c1 = 0;
        c2 = 1.0 / 2;
        c3 = 1.0 / 2;
        c4 = 1.0;
        b1 = 1.0 / 6;
        b2 = 1.0 / 3;
        b3 = 1.0 / 3;
        b4 = 1.0 / 6;

        pendulum.angle_previous = pendulum.angle
        var temp_acc = pendulum_acceleration(pendulum, gravity)

        k1_angle_dot = pendulum.angle_dot
        k1_angle = pendulum.angle_previous
        k1_angle_dot_dot = temp_acc

        k2_angle_dot = [k1_angle_dot[0] + a21 * dt * k1_angle_dot_dot[0], k1_angle_dot[1] + a21 * dt * k1_angle_dot_dot[1] ];
        k2_angle = [k1_angle[0] + a21 * dt * k1_angle_dot[0], k1_angle[1] + a21 * dt * k1_angle_dot[1] ];
        pendulum.angle = [k2_angle[0], k2_angle[1]];
        k2_angle_dot_dot = pendulum_acceleration(pendulum, gravity);

        k3_angle_dot = [ k1_angle_dot[0] + a32 * dt * k2_angle_dot_dot[0], k1_angle_dot[1] + a32 * dt * k2_angle_dot_dot[1] ];
        k3_angle = [ k1_angle[0] + a32 * dt * k2_angle_dot[0], k1_angle[1] + a32 * dt * k2_angle_dot[1] ];
        pendulum.angle = [ k3_angle[0], k3_angle[1] ];
        k3_angle_dot_dot = pendulum_acceleration ( pendulum, gravity );

        k4_angle_dot = [ k1_angle_dot[0] + a43 * dt * k3_angle_dot_dot[0], k1_angle_dot[1] + a43 * dt * k3_angle_dot_dot[1] ];
        k4_angle = [ k1_angle[0] + a43 * dt * k3_angle_dot[0], k1_angle[1] + a43 * dt * k3_angle_dot[1] ];
        pendulum.angle = [ k4_angle[0], k4_angle[1] ];
        k4_angle_dot_dot = pendulum_acceleration ( pendulum, gravity );
        sum_angle_dot_dot = [ 0.0, 0.0 ];
        sum_angle_dot = [ 0.0, 0.0 ];
        sum_angle_dot_dot[0] = b1 * k1_angle_dot_dot[0] + b2 * k2_angle_dot_dot[0] + b3 * k3_angle_dot_dot[0] + b4 * k4_angle_dot_dot[0];
        sum_angle_dot_dot[1] = b1 * k1_angle_dot_dot[1] + b2 * k2_angle_dot_dot[1] + b3 * k3_angle_dot_dot[1] + b4 * k4_angle_dot_dot[1];
        sum_angle_dot[0] = b1 * k1_angle_dot[0] + b2 * k2_angle_dot[0] + b3 * k3_angle_dot[0] + b4 * k4_angle_dot[0];
        sum_angle_dot[1] = b1 * k1_angle_dot[1] + b2 * k2_angle_dot[1] + b3 * k3_angle_dot[1] + b4 * k4_angle_dot[1];
        pendulum.angle_dot = [ pendulum.angle_dot[0] + dt * sum_angle_dot_dot[0], pendulum.angle_dot[1] + dt * sum_angle_dot_dot[1] ];
        pendulum.angle = [ pendulum.angle_previous[0] +  dt * sum_angle_dot[0], pendulum.angle_previous[1] +  dt * sum_angle_dot[1] ];
        pendulum.angle_dot_dot = pendulum_acceleration  ( pendulum, gravity );

    } 
    else {
        pendulum.angle_previous[0] = pendulum.angle[0];
        pendulum.angle[0] = (pendulum.angle[0]+Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot[0] = (pendulum.angle[0]-pendulum.angle_previous[0])/dt;
        pendulum.angle_previous[1] = pendulum.angle[1];
        pendulum.angle[1] = (pendulum.angle[1]-Math.PI/180)%(2*Math.PI);
        pendulum.angle_dot[1] = (pendulum.angle[1]-pendulum.angle_previous[1])/dt;
        numerical_integrator = "none";
    }

    return pendulum;
}

function pendulum_acceleration(pendulum, gravity) {
    // STENCIL: return acceleration(s) system equation(s) of motion 
    // var dot_dot = -(gravity/pendulum.length)*Math.sin(pendulum.angle) + pendulum.control/(pendulum.mass * Math.pow(pendulum.length,2))
    // return dot_dot

    var m1 = pendulum.mass[0];
    var m2 = pendulum.mass[1];
    var l1 = pendulum.length[0];
    var l2 = pendulum.length[1];
    var x1 = pendulum.angle[0];
    var x2 = pendulum.angle[1];
    var v1 = pendulum.angle_dot[0];
    var v2 = pendulum.angle_dot[1];
    var c1 = pendulum.control[0];
    var c2 = pendulum.control[1];

    var A1 = (m1+m2) * l1;
    var B1 = m2 * l2 * Math.cos(x1-x2);
    var C1 = m2*l2*v2*v2*Math.sin(x1-x2) + (m1+m2)*gravity*Math.sin(x1) - c1/l1;
    var A2 = l1*Math.cos(x1-x2);
    var B2 = l2;
    var C2 = -l1*v1*v1*Math.sin(x1-x2) + gravity*Math.sin(x2) - c2/m2/l2;
    var a1 = (B1*C2-B2*C1)/(A1*B2-B1*A2);
    var a2 = (C1*A2-C2*A1)/(A1*B2-B1*A2);

    return [a1,a2];
}

function init_verlet_integrator(pendulum, t, gravity) {
    // STENCIL: for verlet integration, a first step in time is needed
    // return: updated pendulum state and time

    var temp_acc = pendulum_acceleration(pendulum, gravity)
    pendulum.angle_previous = pendulum.angle
    pendulum.angle[0] = pendulum.angle[0] + dt * pendulum.angle_dot[0] + Math.pow(dt, 2) * temp_acc[0] * 0.5
    pendulum.angle[1] = pendulum.angle[1] + dt * pendulum.angle_dot[1] + Math.pow(dt, 2) * temp_acc[1] * 0.5
    pendulum.angle_dot[0] = pendulum.angle_dot[0] + dt * temp_acc[0]
    pendulum.angle_dot[1] = pendulum.angle_dot[1] + dt * temp_acc[1]
   
    t = t + dt

    return [pendulum, t];
}

function set_PID_parameters(pendulum) {
    // STENCIL: change pid parameters
    pendulum.servo = {kp:[1,1], kd:[0,0],ki:[0, 0]};  // no control
    return pendulum;
}

function PID(pendulum, accumulated_error, dt) {
    // STENCIL: implement PID controller
    // return: updated output in pendulum.control and accumulated_error

    // var temp_error_desired = pendulum.desired - pendulum.angle
    var d1 = pendulum.desired[0] - pendulum.angle[0]
    var d2 = pendulum.desired[1] - pendulum.angle[1]
    var temp_error_desired = [d1,d2]

    var current1 = pendulum.angle_previous[0] - pendulum.angle[0] 
    var current2 = pendulum.angle_previous[1] - pendulum.angle[1] 
    var temp_error_current = [current1,current2]



    accumulated_error[0] = accumulated_error[0] + temp_error_desired[0]
    accumulated_error[1] = accumulated_error[1] + temp_error_desired[1]
    var c1 = temp_error_desired[0] * pendulum.servo.kp[0] + accumulated_error[0] * pendulum.servo.ki[0] + (temp_error_current[0]/dt) * pendulum.servo.kd[0];
    var c2 = temp_error_desired[1] * pendulum.servo.kp[1] + accumulated_error[1] * pendulum.servo.ki[1] + (temp_error_current[1]/dt) * pendulum.servo.kd[1];
    pendulum.control= [c1 - c2 , c2];

    // if (pendulum.control != ){
    console.log(temp_error_desired[0] * pendulum.servo.kp[0], temp_error_desired[1] * pendulum.servo.kp[1])
    // }
    return [pendulum, accumulated_error];
}