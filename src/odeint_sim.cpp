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

#include <odeint_sim.hpp>
#include <odeint_obs.hpp>

OdeintSim::OdeintSim()  // Custom constructor
{  
    sav.set(m_states, m_times);  
}

OdeintSim::~OdeintSim()
{}

OdeintSim::resultSim OdeintSim::runSim()
{
    
    std::array<double,3> start_state1 = {pSim.initXVelocity, pSim.initYVelocity,pSim.initYawRate};
       
    boost::numeric::odeint::integrate_adaptive( make_controlled( 1E-12 , 1E-12 , stepper_type() ),
                        nOde , start_state1 , 0.0 , pSim.time , pSim.step , sav );

    resultSim res = calcResult(pSim,m_states,m_times);
    
    m_states.clear();
    m_times.clear();

    if (pSim.terminal_output == true)
    {
        writeTerminal(res);
    }  

    return res;
}

OdeintSim::resultSim OdeintSim::calcResult(paramSim pSim, std::vector<std::array<double,3>> &states , std::vector< double > &times)
{
    OdeintSim::resultSim res;
    
    res.x_pos.push_back(pSim.initX);
    res.y_pos.push_back(pSim.initY);
    res.yaw.push_back(pSim.initYaw);
    
    res.yaw_rate.push_back(pSim.initYawRate);
    res.x_vel.push_back(pSim.initXVelocity);
    res.y_vel.push_back(pSim.initYVelocity);
    
    res.time.push_back(0);
    res.yaw_acc.push_back(0);
    res.x_acc.push_back(0);
    res.y_acc.push_back(0);

    double pi = 2*acos(0.0);

    std::vector<double> delta_time;

    for (std::vector<double>::size_type i = 1; i < states.size(); i++)
    {
        // Access first array element of vector of arrays "states" at position i. In this implementation, the 
        // array is of size 1 because the state array is of size 1 (only yaw rate).
        res.x_vel.push_back(states.at(i)[0]);
        res.y_vel.push_back(states.at(i)[1]);
        res.yaw_rate.push_back(states.at(i)[2]);

        res.time.push_back(times.at(i));

        double delta_time = res.time.at(i) - res.time.at(i-1);
        
        // yaw = (yaw rate of step * time of step) + previous yaw
        double yaw = res.yaw_rate.at(i) * delta_time + res.yaw.at(i-1);
        res.yaw.push_back(yaw);

        // yaw acceleration = (yaw rate of step - yaw rate of previous step) * time of step
        double yaw_acc = (res.yaw_rate.at(i) - res.yaw_rate.at(i-1)) * delta_time;
        res.yaw_acc.push_back(yaw_acc);
        
        double yaw_rad = yaw * pi / 180;
        
        // Position in x, x_vel and y_vel are in ship coordinate system, x_pos and y_pos in global, therefore take the yaw to transform
        double x_pos =(res.x_vel.at(i) * cos(yaw_rad) - res.y_vel.at(i) * sin(yaw_rad))* delta_time + res.x_pos.at(i-1) ; 
        res.x_pos.push_back(x_pos); 

        // Position in y, x_vel and y_vel are in ship coordinate system, x_pos and y_pos in global, therefore take the yaw to transform
        double y_pos = (res.x_vel.at(i) * sin(yaw_rad) + res.y_vel.at(i) * cos(yaw_rad))* delta_time + res.y_pos.at(i-1) ;  
        res.y_pos.push_back(y_pos); 
    }

    return res;
}

OdeintSim::paramSim OdeintSim::readSimulation(std::string simFile)
{
    OdeintSim::paramSim var;

    YAML::Node config = YAML::LoadFile(simFile);

    var.step    = config["step"].as<double>(); 
    var.time    = config["time"].as<double>(); 
    
    var.initYaw = config["initYaw"].as<double>(); 
    var.initX = config["initX"].as<double>(); 
    var.initY = config["initY"].as<double>(); 

    var.initYawRate = config["initYawRate"].as<double>(); 
    var.initXVelocity= config["initXVelocity"].as<double>();
    var.initYVelocity= config["initYVelocity"].as<double>();

    var.terminal_output = config["terminal_output"].as<bool>();

    return var;
}

void OdeintSim::writeTerminal(OdeintSim::resultSim res)
{
    for (std::vector<double>::size_type i = 0; i < res.time.size(); i++)
        {
            double t = round(res.time.at(i)  *100)/100;
            double x = round(res.x_pos.at(i) *100)/100;
            double y = round(res.y_pos.at(i) *100)/100;
            double yaw = round(res.yaw.at(i) *100)/100;

            double x_vel = round(res.x_vel.at(i) *100)/100;
            double y_vel = round(res.y_vel.at(i) *100)/100;
            double yaw_rate = round(res.yaw_rate.at(i) *100)/100;
            
            std::cout <<"t[sec]: "<< t <<" x[m]: "<< x <<" y[m]: "<< y <<" yaw[deg]: " << yaw << '\n';
            std::cout <<"t[sec]: "<< t <<" x_vel[m/s]: "<< x_vel <<" y_vel[m/s]: "<< y_vel <<" yaw_rate[deg/s]: " << yaw_rate << '\n' << "\n";
        } 
}