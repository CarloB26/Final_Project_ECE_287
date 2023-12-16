inputValue = 512;
Kp = 5;
Ki = 1;
Kd = 1;
transferFunctionSimulationWithPID(inputValue, Kp, Ki, Kd);


function transferFunctionSimulationWithPID(inputValue, Kp, Ki, Kd)
    % Build time vector
    dt = 8;
    t = 0:dt:10000;

    % Build step input
    u = ones(size(t)) * inputValue;
    u(1) = 0;

    % Initial conditions
    y0 = 0;
    yd0 = 0;
    ud0 = 0;
    ydd0 = 0;
    % Predefine the size of the input/output vectors
    ud = zeros(size(t));
    y = zeros(size(t));
    yd = zeros(size(t));
    ydd = zeros(size(t));
    feedBack = zeros(size(t));

    % Integrated output variable
    y_integrated = zeros(size(t));

    
    % PID controller variables
    integral = 0;
    prev_error = 0;

    % Step through time and solve the differential equation
    for i = 1:length(t)
        if i == 1
            ud(i) = ud0;
            yd(i) = yd0;
            y(i) = y0;
            ydd(i) = ydd0;
        else
            y_integrated(i) = y_integrated(i-1) + ((1 * ((y(i) + y(i-1))))/ dt);

            % PID controller
            error = inputValue - y_integrated(i);
            integral = integral + error / dt;
            derivative = (error - prev_error) * dt;

            pid_output = Kp * error + Ki * integral + Kd * derivative;

            ud(i) = pid_output;
            yd(i) = yd(i-1) + ydd(i-1)/dt;
            y(i) = y(i-1) + yd(i-1)/dt;
            ydd(i) = (ud(i) + 1 * error - 6 * yd(i) - 10 * y(i)) / 4;

            % Update previous error for the next iteration
            prev_error = error;
        end
    end

    % Plot the output and its integral
    figure;

    subplot(6, 1, 1);
    scatter(t, y, 3);
    legend('Differential equation approach');
    title('Output');

    subplot(6, 1, 2);
    scatter(t, y_integrated, 3);
    legend('Integrated Output');
    title('Integrated Output');

    subplot(6, 1, 3);
    scatter(t, ydd, 3);
    title('ydd');

    subplot(6, 1, 4);
    scatter(t, y, 3);
    title('y');    

    subplot(6, 1, 5);
    scatter(t, yd, 3);
    title('yd');

    subplot(6, 1, 6);
    scatter(t, ud, 3);
    title('ud');

    xlabel('Time');
    ylabel('Values');
end