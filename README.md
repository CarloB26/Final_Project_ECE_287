# PID-CONTROLLER PROJECT
Final Project for ECE 287
Matthew Back & Carlo Buelvas
Fall 2023


Overview
Our research builds a PID controller that can control the output of a transfer function in order to simulate a motor model. The PID controller solves for an output position. not simply velocity, hence the controller first takes the output derivative before obtaining feedback. FPGAs are used in this project because of their great performance and adaptability to create other systems.


Background 
An input and output relationship in a system are represented mathematically by a transfer function. This is an input of the desired location for our system and an output of the actual position. With the use of this transfer function, a motor simulation is accurate. 
A popular type of feedback control system that modifies a system's input according to its current output is the PID controller. To achieve the desired output within limits, the PID controller uses feedback from the system along with a set of proportional integral and derivative gains. 


Proportional (P) Term
The current mistake is proportionate to this phrase. It responds instantly to the present error with the intention of minimizing it. The proportionate contribution rises with a greater kp value, which causes the system to react to the current error more forcefully.
Integral (I) Term 
The total of all prior errors is taken into consideration by this phrase. It addresses extended departures from the setpoint, which aids in the elimination of steady-state error. When the system is constantly being affected by a bias or disturbance, the integral term becomes especially helpful. Growing increases the impact of cumulative mistakes.




Derivative (D) Term 
This term takes the error's rate of change into account.It reduces oscillations and overshooting by projecting future errors depending on the rate of change that is occurring right now.The derivative term enhances the system's transient reactivity.The dampening effect is increased with a larger value. 
The PID controller fine-tunes the motor control system to create a balance between a quick reaction, little overshooting, and precision at steady state by varying the gains. Due to its ease of use and efficiency in a range of control applications, including motor control, PID controllers are extensively utilized.


Design
This project was divided into several sections in order to make it more manageable. 
PID and Transfer Function design in Matlab
Rather than using the built-in transfer functions in Matlab, an algorithm loop was created by starting with a baseline motor model second order transfer function that was synthesized and programmed using numerical methods. An implementation of a PID controller was made using this transfer function approach. We were able to comprehend the steps the program would need to follow thanks to the way the method was designed in Matlab. An excerpt of our Matlab algorithm can be found here.

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
    End


Using Verilog for code design
A verilog program and testbench were created from this Matlab code by adhering to its algorithmic structure. Many of the items from earlier labs or assignments are implemented in this code. Initially, we began by configuring an FSM capable of running the algorithm step by step. The next step was to set up each equation and variable. Among the things this proof taught us is that the verilog "/" division operator is incredibly unreliable and inefficient. We therefore switched to dividing more effectively and maintaining the accuracy of our signed 2's complement variables by utilizing a right shift signed ">>>" operator.


The test bench's design in Verilog
What you would do in the real world is replicated on the test bench. putting some variables in order, like turning off the reset and turning on the start. Future program users can additionally set their desired output and the Kp, Ki, and Kd variables in the testbench. To aid in debugging, a monitor section is also included. It outputs the values for each variable. 


How a new user should utilize the system
You can set the input variables and Kp Ki Kd to calculated or desired values in the testbench. This allows you to simulate the code and compare, via graphs, its outputs with the desired outcome. As Figure 1 illustrates


![image](https://github.com/CarloB26/Final_Project_ECE_287/assets/153966309/7bae0745-3dc6-4e5e-9ec9-ffb8c75b4573)

Figure 1: Output approach with the target value of 500 displayed
You can then modify your Kp Ki and Kd variables to achieve your desired outcomes. For example, a reduced overshoot percentage, a faster rise time, and a decreased steady state error. 


Changing the transfer function to represent various systems
It is possible to change the program's actual transfer function to represent various motor types. Case A of the FSM allows for the setting of a, b, and c values. These values match the transfer function values shown in the following illustration.

![image](https://github.com/CarloB26/Final_Project_ECE_287/assets/153966309/97136ad5-c380-454b-a1a5-f32b4d6cf97f)

Figure 2: Transfer function formatting

Future Objectives of the Program
On a user's PC, this program should ideally not be mimicked. Operating on the actual FPGA board would be preferable. To avoid costly errors while loading the program on the real machine, this could be helpful for emulating motor performance and PID controller values. An ALU with registers to rapidly modify input and transfer function values would be this program's future version. This software wouldn't need to use up PC processing resources in order to execute rapidly and effectively on an FPGA. 


Outcomes and potential enhancements
The results of our transfer function simulation using the PID controller were good and closely matched the output of the Matlab software. One area for improvement would be to tweak the fixed point values and use better fractional bits to increase the program's accuracy. Still, the existing presentation of the results is satisfactory. 





![image](https://github.com/CarloB26/Final_Project_ECE_287/assets/153966309/04357c7f-bc43-4516-a39f-d059be33cd59)

Figure 3: the results of testing different Kp Ki & Kd values 
This figure showcases the changing of Kp Ki and Kd values which have a goal output of 50.

![image](https://github.com/CarloB26/Final_Project_ECE_287/assets/153966309/58e70e5f-4bff-4a93-a834-566442897429)

Figure 4: the results of testing different Kp Ki & Kd values 
This figure showcases the changing of Kp Ki and Kd values which have a goal output of 500. 


Exporting data
After compiling the program and running the Modelsim altera software a waveform diagram will be made. Once this is made the output data can be selected and put into a list form by using the list section of Modelsim. This can then be opened in excel and graphed. 


Conclusion
The implementation of the PID controller and transfer function simulation were a success. However they are only stepping stones to future projects. A PID controller and transfer function when applied together can simulate many different systems other than just motors as shown in this project. They could be used to simulate physical systems, other devices, or any feasible design that has an input or output. These are the building blocks of controls and with simple adjustments it can be made to fit many possible systems and use cases.


Appendix
![image](https://github.com/CarloB26/Final_Project_ECE_287/assets/153966309/93f4c3fa-eb5a-4143-bdc1-217e6c09bb66)
![image](https://github.com/CarloB26/Final_Project_ECE_287/assets/153966309/34ed3ac2-3dc3-4617-80cd-da0d73acf06e)
![image](https://github.com/CarloB26/Final_Project_ECE_287/assets/153966309/7855ad58-f0d7-406c-a424-0987fcc549aa)

![image](https://github.com/CarloB26/Final_Project_ECE_287/assets/153966309/7eb9bf6a-54cc-49c8-9f8c-d1654a10658d)

Image showing where kp ki and kd values can be edited

