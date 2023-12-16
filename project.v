module project(clk,start,rst,in,kp,ki,kd,o);
input clk, start, rst;
input [31:0] in,kp,ki,kd;
output reg [31:0] o;

//reg variables
reg signed[31:0] dt;
reg signed[15:0] i;
reg signed[31:0] ud;
reg signed[31:0] yd;
reg signed[31:0] yd_1;
reg signed[31:0] y;
reg signed[31:0] y_1;
reg signed[31:0] ydd;
reg signed[31:0] ydd_1;
reg signed[31:0] y_int;
reg signed[31:0] y_int_1;
reg signed[31:0] error;
reg signed[31:0] prev_error;
reg signed[31:0] yddtemp;
reg signed[31:0] yd6;
reg signed[31:0] y10;
reg signed[31:0] a;
reg signed[31:0] b;
reg signed[31:0] c;
reg signed[31:0] kp1;
reg signed[31:0] ki1;
reg signed[31:0] kd1;
reg signed[31:0] integral;
reg signed[31:0] derivative;
reg signed[31:0] pid;


reg [4:0]s;
reg [4:0]ns;

//set cases
parameter A = 5'b00000,
			 B = 5'b00001,
			 C = 5'b00010,
			 D = 5'b00011,
			 E = 5'b00100,
			 F = 5'b00101,
			 G = 5'b00110,
			 H = 5'b00111,
			 I = 5'b01000,
			 J = 5'b01001,
			 K = 5'b01010,
			 L = 5'b01011,
			 M = 5'b01100,
			 N = 5'b01101,
			 O = 5'b01110,
			 P = 5'b01111,
			 Q = 5'b10000,
			 R = 5'b10001,
			 S = 5'b10010,
			 T = 5'b10011,
			 U = 5'b10100,
			 V = 5'b10101;
			 
//program reset switch		 
always @(posedge clk or negedge rst)
begin
	if (rst == 1'b0)
		s <= A;
		else 
		s <= ns;
end

//set case flow for fsm
always @(*)
begin
	case(s)
		A: if (start==1)
			begin
			ns = A;
			end
			else 
			begin
			ns = B;
			end
			
		B: ns = C;
		
		C: if (i==16'b1111111111111111) //run until i equals this then end if you would like a shorter runtime just make this value lower
			begin
			ns = V;
			end
			else 
			begin
			ns = D;
			end
		
		D: ns = E;
		E: ns = F;
		F: ns = G;
		G: ns = H;
		H: ns = I;
		I: ns = J;
		J: ns = K;
		K: ns = L;
		L: ns = M;
		M: ns = N;
		N: ns = O;
		O: ns = P;
		P: ns = Q;
		Q: ns = R;
		R: ns = S;
		S: ns = T;
		T: ns = U;
		U: ns = B;
		V: ns = V;
		
		
		
	endcase
end

always @(posedge clk or negedge rst)
begin
	if (rst == 1'b0)
		begin
			i <= 16'b1111111111; //set i for how many times you would like the fsm to loop
		end
	else
		begin
			case (s)
				A: 
				begin
				i <= 16'b0; //initalize values 
				ud <= 32'b0;
				yd <= 32'b0;
				yd_1 <= 32'b0;
				y <= 32'b0;
				y_1 <= 32'b0;
				ydd <= 32'b0;
				ydd_1 <= 32'b0;
				y_int <= 32'b0;
				y_int_1 <= 32'b0;
				dt <= 32'b0;
				prev_error <= 32'b0;
				error <= 32'b0;
				yddtemp <= 32'b0;
				yd6 <= 32'b0;
				y10 <= 32'b0;
				a <= 32'b1;
				b <= 32'b110;
				c <= 32'b1010;
				kp1 <= 32'b0;
				ki1 <= 32'b0;
				kd1 <= 32'b0;
				integral <= 32'b0;
				derivative <= 32'b0;
				pid <= 32'b0;
				end
				
				B: 
				begin
				y_int <= y_int_1 + ((a * (y + y_1) ) >>>3); //y_int is the output
				end
				
				C:
				begin
				error <= (in - (y_int)); //calculate the error from input to output
				end
				
				D:
				begin
				o <= y_int; //set the output value
				end
				
				E: 
				begin
				yd <= yd_1 + (ydd_1 >>>3); //solve for yd value which is the value of the derivative of y
				end
				
				F:
				begin
				y <= y_1 + (yd_1 >>>3); //solve for the y value
				end
				
				G: 
				begin
				yd6 <= b * yd; //multiply yd by your transfer function value b
				end
				
				H:
				begin
				
				y10 <= c * y; //multiply y by your transfer function value c
				end
				
				I:
				begin
				integral <= integral + error >>>2; //solve for the integral value of PID
				derivative <= (error - prev_error); //solve for the derivative value of PID
				end 

				J: 
				begin
				kp1 <= kp * error; //solve for kp gain * integral
				end
								
				K: 
				begin
				ki1 <= ki * integral; //Solve for ki gain * integral
				end
				
				L: 
				begin
				kd1 <= kd * derivative; //solve for kd gain * integral
				end
				
				M: 
				begin
				pid <= kp1+ki1+kd1; //solve for the total pid value
				end
				
				N: 
				begin
				end
				
				O: 
				begin
				ydd <= pid + error - yd6 -y10; //solve for ydd the double deravitive of y still needs divided however
				yddtemp <= ydd>>>2; //divide to finsh solving for ydd
				end
				
				P: 
				begin
				end
				
				Q: 
				begin
				end
				
				R: 
				begin
				end
				
				S: 
				begin
				end
				
				T: 
				begin
				yd_1 <= yd; //set all your values to become previous values
				y_1 <= y;
				ydd_1 <= yddtemp;
				y_int_1 <= y_int;
				prev_error <= error;
				end
				
				U: 
				begin
				
				i <= i+16'b1; //cout up 1 for i
				end
				
				V: 
				begin
				end
				
			endcase
	end
end


endmodule
