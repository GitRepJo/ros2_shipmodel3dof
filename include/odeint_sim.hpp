// Copyright (c) 2023 Jonas Mahler
	
// This file is part of shipmodel3dof.

// shipmodel3dof is free software: you can redistribute it and/or modify it under the terms 
// of the GNU General Public License as published by the Free Software Foundation, 
// either version 3 of the License, or (at your option) any later version.

// shipmodel3dof is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; 
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
// See the GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along 
// with shipmodel3dof. If not, see <https://www.gnu.org/licenses/>. 

#ifndef ODEINT_SIM_HPP
#define ODEINT_SIM_HPP

#include <nomoto_ode.hpp>
#include <odeint_obs.hpp>

#include <eigen3/Eigen/Dense>
#include <boost/numeric/odeint.hpp>
#include <tuple>
#include <string>
#include <vector>
#include <iostream>
#include <boost/math/interpolators/cubic_b_spline.hpp>
#include <boost/math/differentiation/finite_difference.hpp>


/* Read model and simulation parameters and run a ship model simulation*/ 
class OdeintSim {
public:

    OdeintSim();// Default constructor
    
    ~OdeintSim(); // Deconstructor

    typedef boost::numeric::odeint::runge_kutta_dopri5< std::array< double, 3>  > stepper_type;
    
    /* Parameters used for the simulation of the ship model*/
    struct paramSim
    {
        double initYaw = 0.0; // Initial yaw in degree
        double initX = 0.0; // Initial position of x in meter
        double initY = 0.0; // Initial position of y in meter
        
        double initYawRate = 0.0; // Initial yaw rate in degree/s
        double initXVelocity = 0.0; // Initial x velcoity in m/s
        double initYVelocity = 0.0; // Initial y velocity in m/s
        
        double step = 0.0; // Stepsize for integration
        double time = 0.0; // Time to end of integration
        bool terminal_output = true ; // Write result struct to terminal if true
    };
    
    /* Save the results of the simulation of the ship model*/
    struct resultSim
    {
        std::vector<double> time; // Time corresponding to the state
        std::vector<double> x_pos; // Position x in m two dimensional carthesian global coordinates
        std::vector<double> y_pos; // Position y in m two dimensional carthesian global coordinates
        std::vector<double> x_vel; // Velocity x in m/s two dimensional carthesian global coordinates
        std::vector<double> y_vel; // Velocity y in m/s two dimensional carthesian global coordinates
        std::vector<double> x_acc; // Acceleration x in m/s^2 two dimensional carthesian global coordinates
        std::vector<double> y_acc; // Acceleration y in m/s^2 two dimensional carthesian global coordinates
        std::vector<double> yaw; // Orientation in global coordinates relative to north in radians
        std::vector<double> yaw_rate; // Yaw rate in radians/s
        std::vector<double> yaw_acc;  // yaw rate acceleration in radians/s*s
    };

     /**
    * @brief Run the ship model simulation by using a predefined ordinary differential equation
    * @param -
    * @return resultNomoto struct with the results (actual yaw angle, position ...)
    */
    OdeintSim::resultSim runSim();
    
    /**
    * @brief Read variables for the simulation of Nomotos ship model
    * @param simFile -yanl file that matches variables in varSim struct
    * @return varSim struct with variables for specific simulation
    * @details The variables will be used by boost integration function to set up the simulation
    */
    paramSim readSimulation(std::string simFile);
    
    NomotoOde nOde; // Equation to simulate with
    paramNomoto pNomoto; // Parameters for equation
    OdeintSim::paramSim pSim; // Parameters for simulation
    
private:
    /**
    * @brief Calculate the yaw angle based on the simulation parameters and the position
    * @param vS varSim struct to save constants variables specific to the simulation
    * @param states yaw rate result of the simulation, in this case only a one dimensional state vector
    * @param times Corresponding time for the yaw rate (no constant delta t with adaptive intregration)
    * @return resultNomoto struct with the results (actual yaw angle, position ...)
    */
    resultSim calcResult(paramSim vS, std::vector<std::array<double,3>> &states , std::vector< double > &times);

    /**
    * @brief Write the result of the simulation to the terminal 
    * @param res struct that contains the result of the simulation
    * @details The values will be rounded to 2 digits after the comma
    */
    void writeTerminal(OdeintSim::resultSim res);

    Observer sav; // Observer for odeint to get states of integration
    std::vector<std::array<double,3>> m_states; // Save states of state vector in integration
    std::vector<double> m_times; // Save time of integration step
};

#endif //ODEINT_SIM_HPP