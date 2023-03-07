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

#ifndef NOMOTO_ODE_HPP
#define NOMOTO_ODE_HPP

#include <eigen3/Eigen/Dense>
#include <iostream>
#include "yaml-cpp/yaml.h"

/* Constants used for Nomoto ship model*/
struct paramNomoto
    {
        double delta = 0.0; // Rudder angle of the vessel, positive value equals rudder deflection to the left [degree]
        double rpm = 0.0; // Rounds per minute of propeller

        double a_x = 0.0; // Arbitrary surge scaling function 
        double b_x = 0.0; // Arbitrary surge scaling function 
        double a_y = 0.0; // Arbitrary sway scaling function 
        double b_y = 0.0; // Arbitrary sway scaling function 
        
        double K = 0.0; // Hydrodynamic constant dependent on vessel
        double T = 0.0; // Time constant dependent on vessel
    };

/** 
* @brief Functor Ordinary Differential Equation for Nomoto ship model
* @param m_p struct of coefficients used by the nomto ordinary differential equation
*/
class NomotoOde {

public:

    paramNomoto m_p; // Parameters for equation
    
    /**
    * @brief Set parameters for equation
    * @param nomCoeff Struct with parameters for equation
    * @return -
    * @details -
    */
    void set(const paramNomoto &nomCoeff)
    {
        m_p = nomCoeff;
    }
    
    /**
    * @brief Read constants for the Nomoto ship model
    * @param nomotoFile .yaml file that matches variables in constNomoto struct
    * @return constNomoto struct with constants for Nomoto
    * @details The constants will be used to set up the Nomoto ship model functor
    */
    paramNomoto read(std::string nomotoFile)
    {
        paramNomoto con;

        YAML::Node config = YAML::LoadFile(nomotoFile);

        con.a_x = config["a_x"].as<double>();
        con.b_x = config["b_x"].as<double>();
        con.a_y = config["a_y"].as<double>();
        con.b_y = config["b_y"].as<double>();
        con.K = config["K"].as<double>();
        con.T = config["T"].as<double>();
        con.delta = config["delta"].as<double>();
        con.rpm = config["rpm"].as<double>();

        return con;
    }

    /**
    * @brief Nomto equation to calculate the yaw acceleration of a ship over time
    * @param x array for state vector derivative which is in this case three (yaw, x, y rate) 
    * @param dxdt array for state vector second derivative which is in this case three (yaw, x and y acceleration) 
    * @param t double time of integration step
    * @return -
    * @details This function is used by odeint internally to compute the integration step according to time
    * The equation follows K.Nomoto 1956 " On the steering qualities of ships" in International shipbuilding progress
    * 
    * ----Variables/Constants 
    * 
    * delta  : Rudder angle
    * rpm    : Rounds per minute
    * 
    * x[0]   : X velocity /surge velocity
    * dxdt[0]: X acceleration /surge acceleration
    * a_x, b_x: Arbitrary surge scaling function
    * 
    * x[1]   : Y velocity / sway velocity
    * dxdt[1]: Y acceleration /sway acceleration
    * a_y, b_y: Arbitrary sway scaling function 
    * 
    * x[2]   : Yaw velocity
    * dxdt[2]: Yaw acceleration
    * K      : Hydrodynamic constant dependent on vessel
    * T      : Time constant dependent on vessel
    * a      : -1/T
    * b      : K/T
    */
    void operator() ( const std::array<double,3> &x , std::array<double,3> &dxdt , double t ) // Use vector of arrays to extend state vector of ode if required
    {
        // Boost integrator e.g. integrate_adaptive requires a variable t
        // t is not used by the ode
        // Create a dummy use case to avoid warnings
        t = 0.0;
        
        double a_yaw = - 1/m_p.T + t;
        double b_yaw = m_p.K/m_p.T + t;
        
        int sgn = 1; // reverse drift direction for backwards driving
        if (m_p.rpm < 0) sgn = -1 ;
        
        dxdt[0] = m_p.a_x * x[0] + m_p.b_x * m_p.rpm ; 
        dxdt[1] = m_p.a_y * x[1] - sgn * m_p.b_y * m_p.delta ; 
        dxdt[2] = a_yaw * x[2] + b_yaw * m_p.delta ; 
    }
};
#endif //NOMOTO_ODE_HPP