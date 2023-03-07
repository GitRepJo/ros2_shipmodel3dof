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

#ifndef ODEINT_OBS_HPP
#define ODEINT_OBS_HPP

#include <eigen3/Eigen/Dense>
#include <vector>

/**
* @brief Functor to save the result of an integration with boost e.g. adaptive integration
* @param states std::vector<std::vector<double>> intermediate and result state(s) of integration
* @param times std::vector<double> Time that corresponds to according state
*/
class Observer
{   
public:
     /**
    * @brief Set parameters for observations
    * @param states vector of states (x velocity, y velocity, yaw rate)
    * @param times vector of integration steps
    * @return -
    * @details -
    */
    void set (std::vector<std::array<double,3>> &states, std::vector<double> &times) 
    {
        m_states = &states;
        m_times = &times;
    };
    
    /**
    * @brief Observe intermediate steps of odeint
    * @param x vector of arrays to save state vector derivative (which is in this case three (x velocity, y velocity, yaw rate)) 
    * @param t double time of integration step 
    * @return -
    * @details Pass by reference to allows observing the states outside of this function
    * Use vector of arrays to extend state vector of observer if required
    */
    void operator()( const std::array<double,3> &x, const double t )
    {    
        m_states->push_back(x);
        m_times->push_back(t);
    }
private:

    std::vector<std::array<double,3>> * m_states;
    std::vector<double> * m_times;
};

#endif //ODEINT_OBS_HPP