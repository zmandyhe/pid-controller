#include "PID.h"
#include <cmath>
#include <iostream>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
    this->Kp = Kp_;
    this->Ki = Ki_;
    this->Kd = Kd_;
    this->p_error = 0.0;
    this->i_error = 0.0;
    this->d_error = 0.0;

    dp = {0.1*Kp,0.1*Kd,0.1*Ki};
    step = 1;
    p_index = 0; 
    n_settle_steps = 100;
    n_eval_steps = 1000;
    total_error = 0;
    best_error =  100000.0;
}

void PID::UpdateError(double cte) {
    if (step == 1) {
        // to get correct initial d_error
        p_error = cte;
    }
    d_error = cte - p_error;
    p_error = cte;
    i_error += cte;

    // update total error only if we're past number of settle steps
    if (step % (n_settle_steps + n_eval_steps) > n_settle_steps){
        total_error += pow(cte,2);
    }

    if (step % (n_settle_steps + n_eval_steps) == 0){
        std::cout << "step" << step << std::endl;
        std::cout<<"total error" << total_error << std::endl;
        std::cout<< "best error" << best_error << std::endl;

        if (total_error < best_error) {
            cout << "improvement!" << endl;
            best_error = total_error;
            if (step !=  n_settle_steps + n_eval_steps) {
                // don't do this if it's the first time through
                dp[p_index] *= 1.1;            
            }
 
            p_index = (p_index + 1) % 3;
            second = first = false;
        }
        else if (!second && !first) {
            // try adding dp[i] to params[i]
            Cal_Parameter(p_index, dp[p_index]);
            second = true;
        }
        else if (second && !first) {
            // try subtracting dp[i] from params[i]
            Cal_Parameter(p_index, -2 * dp[p_index]);     
            first = true;         
        }
        else {
            // set it back, reduce dp[i], move on to next parameter
            Cal_Parameter(p_index, dp[p_index]);      
            dp[p_index] *= 0.9;
            // next parameter
            p_index = (p_index + 1) % 3;
            second = first = false;
        }
        total_error = 0;
        cout << "new parameters" << endl;
        cout << "P: " << Kp << ", I: " << Ki << ", D: " << Kd << endl;        
    }
    step++;
}

double PID::TotalError() {
    //return 0.0;  
    return (-Kp*p_error)-(Ki*i_error)-(Kd*d_error);  // TODO: Add your total error calc here!
}

void PID::Cal_Parameter(int index, double dp_value) {
    if (index == 0) {
        Kp += dp_value;
    }
    else if (index == 1) {
        Kd += dp_value;
    }
    else if (index == 2) {
        Ki += dp_value;
    }
    else {
        std::cout << "Cal_Parameter: index out of bounds";
    }
}

double PID::GetSumdp(){
    return dp[0] + dp[1] + dp[2];
}

