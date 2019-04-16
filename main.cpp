#include <math.h>
#include <uWS/uWS.h>
#include <iostream>
#include <string>
#include "json.hpp"
#include "PID.h"

// for convenience
using nlohmann::json;
using std::string;
using namespace std;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.find_last_of("]");
  if (found_null != string::npos) {
    return "";
  }
  else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 1);
  }
  return "";
}

int main() {
  uWS::Hub h;

  PID pid;
  /**
   * TODO: Initialize the pid variable.
   */
  bool twiddle = true;

  //double Kp = 0.05, Ki = 0.0001, Kd = 1.5;
  /* get the initial parameter by manual traning each p_index */

  double p[3] = { 0.05,0.0001,1.5 };
  double dp[3] = { 0.01,0.0001,0.1 };
  //pid.Init(Kp, Ki, Kd);
  double best_p[3] = { p[0],p[1],p[2] };
  double best_error = 1000000.0;
  double tolerance = 0.001;
  /*define the itration counter*/
  int n_iteration = 0;
  int max_iteration = 1000;
  double sum_cte = 0;

  bool flag_plusdp = true, flag_minusdp = false;

  int p_index = 0;

  pid.Init(p[0], p[1], p[2]);
  if (twiddle == true) {
	  /*initial the pid parameter*/
	  pid.Init(p[0], p[1], p[2]);
  }
  else {
	  /*the parameter trained by twiddle*/
	  pid.Init(0.05,0.0001,1.5);

  }
  //int iterate

  h.onMessage([&pid,&twiddle,&p,&dp,&tolerance,&n_iteration,&max_iteration,&sum_cte,&best_error,&best_p,&flag_minusdp,&flag_plusdp, &p_index]\
	  (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {
      auto s = hasData(string(data).substr(0, length));

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

		if (event == "telemetry") {
			// j[1] is the data JSON object
			double cte = std::stod(j[1]["cte"].get<string>());
			double speed = std::stod(j[1]["speed"].get<string>());
			double angle = std::stod(j[1]["steering_angle"].get<string>());
			double steer_value;
			/**
			 * TODO: Calculate steering value here, remember the steering value is
			 *   [-1, 1].
			 * NOTE: Feel free to play around with the throttle and speed.
			 *   Maybe use another PID controller to control the speed!
			 */
			json msgJson;

			/* twiddle */
			/*initialization*/
			/* Can not initailization for every onmessage event */

			//double p[3] = { Kp, Ki, Kd };
			//double dp[3] = { 1,1,1 };


			if (twiddle == true) {
				/*inital the pid for each epoch */
				/*the parameter is updated by each epoch */
				/*n_iteration == 0 is the flag of the start of an epoch */
				if (n_iteration == 0) {
					pid.Init(p[0], p[1], p[2]);
				}

				/*sum the second half of the cte error*/
				if (n_iteration > max_iteration / 2) {
					sum_cte += pow(cte, 2);
				}
				
				/*normal operation*/
				{
					pid.UpdateError(cte);
					steer_value = pid.TotalError();
					n_iteration++; 
				}



				/*??????????????????????????????????*/
				/*the first p=p+dp;*/
				/*?????????????????????????????????*/



				/*check if the n_iteration reach the max*/
				/*-->The iteriation is done */
				/*if yes, check the error with best_error*/
				if (n_iteration > max_iteration) {
					/*motion is done */
					/*reset the count n_iteration */
					n_iteration = 0;
					/*need to judge the best_error and change the dp */

					/* main twiddle function */
					/* compesate the sum_cte with *2 to get the error */
					double error = sum_cte / max_iteration * 2;
					/* reset the sum_ste */
					sum_cte = 0.0;

					/*because the main process is triggerred by the event of new cte message from the simular*/
					/*Need to distinguish the plusdp,minusdp(increase dp),or need to decrease dp*/					
					if (error < best_error) {
						best_error = error;
						best_p[0] = p[0];
						best_p[1] = p[1];
						best_p[2] = p[2];

						dp[p_index] *= (flag_plusdp||flag_minusdp ? 1.1 : 0.9);
						

						/*if the loop is done(get the best_error), next p_index */
						p_index += 1;
						p_index = p_index % 2;

						/* prepare for the next p_index */
						/*?????????????????????????????????*/
						p[p_index] += dp[p_index];
					}
					else {
						/* didnt get a new best_error */
						if (flag_plusdp == true) {
							flag_plusdp = false;
							flag_minusdp = true;
							p[p_index] -= 2*dp[p_index];

						}
						else if (flag_minusdp == true) {
							flag_minusdp = false;
							p[p_index] += dp[p_index];

						}
						else
						{	/* next p_index */
							flag_plusdp = true;

							/*if the loop is done(get the best_error), next p_index */
							p_index += 1;
							p_index = p_index % 2;

							/* prepare for the next p_index */
							/*?????????????????????????????????*/
							p[p_index] += dp[p_index];
						}

					}

					/*until the p_index reach the 2th, reset the p_index*/

					/*check if the sum(dp) is satisfied the tolerance.*/
					if (dp[0] + dp[1] + dp[2] < tolerance) {
						/*get the best p[]*/
						cout << "##### twiddle completation ! #####" << endl;
						cout << "best parameter Kp: " << p[0] << " Ki: " << p[1] << " Kd: " << p[2] << endl;
						ws.close();
					}
					else {
						string reset_msg = "42[\"reset\",{}]";
						ws.send(reset_msg.data(), reset_msg.length(), uWS::OpCode::TEXT);
					}
				}
				/*under the iteration max */
				/* go on */
				else {


					//json msgJson;
					msgJson["steering_angle"] = steer_value;
					msgJson["throttle"] = 0.3;
					auto msg = "42[\"steer\"," + msgJson.dump() + "]";
					std::cout << msg << std::endl;
					ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
				}
			}

			/* normal simulation instead of twiddle */
			else {

				/*the parameter trained by twiddle*/
				//pid.Init(0.05,0.0001,1.5);
				//pid.Init(0.06, 0.00031, 1.29);
			   /*initialization need only once*/

				pid.UpdateError(cte);
				steer_value = pid.TotalError();
			
			// DEBUG
			cout << "CTE: " << cte << " Steering Value: " << steer_value << std::endl;
			
			msgJson["steering_angle"] = steer_value;
			msgJson["throttle"] = 0.3;
			auto msg = "42[\"steer\"," + msgJson.dump() + "]";
			std::cout << msg << std::endl;
			ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
			}
        }  // end "telemetry" if
      } else {
        // Manual driving
        string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket message if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code, 
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}