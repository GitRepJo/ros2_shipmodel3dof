#include <catch2/catch_test_macros.hpp>

#include <odeint_sim.hpp>


TEST_CASE( "Test the Nomoto model")
{
    
    OdeintSim exampleSim;
    exampleSim.pNomoto = exampleSim.nOde.read("./test_config.yaml");
    exampleSim.nOde.set(exampleSim.pNomoto);
    exampleSim.pSim = exampleSim.readSimulation("./test_config.yaml");

    OdeintSim::resultSim resN;
    

    SECTION("read values should match expectation")
    {      
        REQUIRE( exampleSim.pNomoto.delta == 512332123.3212321);
        REQUIRE( exampleSim.pSim.initX == 12312.123342123) ;
    }
    
    // Use Figure 3 of K.Nomoto 1956 On the steering qualities of ships in International shipbuilding progress
    // Small deviations because delta rudder angle is set directly to 15 degree instead of rising as described
    SECTION("Compare model with figure 3 K= 0.065 & T = 30")
    {   
        // Set back struct to values in definition
        exampleSim.pNomoto = {};
        exampleSim.pSim = {};

        exampleSim.pNomoto.delta = 15.0;
        exampleSim.pNomoto.K = 0.065;
        exampleSim.pNomoto.T = 30; 
        exampleSim.pSim.time = 100;
        exampleSim.pSim.step = 0.1;
        exampleSim.pSim.terminal_output = false;
        
        exampleSim.nOde.set(exampleSim.pNomoto);
        
        resN = exampleSim.runSim();
        
        int s_result = resN.yaw_rate.size();
        double end_yaw_rate = resN.yaw_rate.at(s_result-1);
        
        REQUIRE( (end_yaw_rate < 1.0) & (end_yaw_rate > 0.90) );
    }
    
    // Use Figure 3 of K.Nomoto 1956 On the steering qualities of ships in International shipbuilding progress 
    // Small deviations because delta rudder angle is set directly to 15 degree instead of rising as described
     SECTION("Compare model with figure 3 K= 0.005 & T = 50")
    {   
        // Set back struct to values in definition
        exampleSim.pNomoto = {};
        exampleSim.pSim = {};

        exampleSim.pNomoto.delta = 15.0;
        exampleSim.pNomoto.K = 0.05;
        exampleSim.pNomoto.T = 50; 
        exampleSim.pSim.time = 100;
        exampleSim.pSim.step = 0.1;
        exampleSim.pSim.terminal_output = false;
        
        exampleSim.nOde.set(exampleSim.pNomoto);
        
        resN = exampleSim.runSim();
        
        int s_result = resN.yaw_rate.size();
        double end_yaw_rate = resN.yaw_rate.at(s_result-1); 
        
        REQUIRE( (end_yaw_rate < 0.7) & (end_yaw_rate > 0.60) );
    }
    

    SECTION("position after 1 second and rudder angle 0 with constant velocity")
    {   
        // Set back struct to values in definition
        exampleSim.pNomoto = {};
        exampleSim.pSim = {};

        exampleSim.pNomoto.delta = 0.0;
        exampleSim.pNomoto.K = 0.05;
        exampleSim.pNomoto.T = 50; 
        exampleSim.pSim.time = 1.0;
        exampleSim.pSim.step = 0.1;
        exampleSim.pNomoto.rpm  = 15.0;
        exampleSim.pNomoto.b_x = 1.0;
        exampleSim.pNomoto.b_y = 1.0;
        exampleSim.pSim.terminal_output = false;

        exampleSim.nOde.set(exampleSim.pNomoto);
     
        resN = exampleSim.runSim();

        int s_result = resN.x_pos.size();

        // Round because of floating point errors
        double end_pos_x = round( resN.x_pos.at(s_result-1) * 100 ) / 100; 
        double end_pos_y = round( resN.y_pos.at(s_result-1) * 100 ) / 100; 
        
        REQUIRE(end_pos_x == 10.61);
        REQUIRE(end_pos_y == 0.0);
    }

       SECTION("position after 1 second and rudder angle 0 with constant velocity and initial heading to 90 degree ")
    {   
        // Set back struct to values in definition
        exampleSim.pNomoto = {};
        exampleSim.pSim = {};

        exampleSim.pNomoto.delta = 0.0;
        exampleSim.pNomoto.K = 0.5;
        exampleSim.pNomoto.T = 50.0; 
        exampleSim.pNomoto.rpm = 15.0;
        exampleSim.pNomoto.b_x = 1.0;
        exampleSim.pNomoto.b_y = 1.0;
        exampleSim.pSim.time = 1.0;
        exampleSim.pSim.step = 0.1;
        exampleSim.pSim.initYaw = 90.0;
        exampleSim.pSim.terminal_output = false;

        exampleSim.nOde.set(exampleSim.pNomoto);
        
        resN = exampleSim.runSim();
        
        int s_result = resN.x_pos.size();

        // Round because of floating point errors
        double end_pos_x = round( resN.x_pos.at(s_result-1) * 100 ) / 100; 
        double end_pos_y = round( resN.y_pos.at(s_result-1) * 100 ) / 100; 
        
        REQUIRE(end_pos_x == 0.0);
        REQUIRE(end_pos_y == 10.61);
    }

    SECTION("yaw angle has to be in positive x,y section for positive rudder deflection")
    {   
        // Set back struct to values in definition
        exampleSim.pNomoto = {};
        exampleSim.pSim = {};

        exampleSim.pNomoto.delta = 30.0;
        exampleSim.pNomoto.K = 0.5;
        exampleSim.pNomoto.T = 50.0; 
        exampleSim.pSim.time = 1.0;
        exampleSim.pSim.step = 0.1;
        exampleSim.pNomoto.rpm = 15.0;
        exampleSim.pSim.initYaw = 0.0;
        exampleSim.pSim.terminal_output = false;

        exampleSim.nOde.set(exampleSim.pNomoto);
        
        resN = exampleSim.runSim();
        
        int s_result = resN.x_pos.size();

        // Round because of floating point errors
        double yaw = round( resN.yaw.at(s_result-1) * 100 ) / 100; 
    
        REQUIRE((yaw > 0) & (yaw < 90 ));
    }

    SECTION("yaw angle has to be in negative x,y section for negative rudder deflection")
    {   
        // Set back struct to values in definition
        exampleSim.pNomoto = {};
        exampleSim.pSim = {};

        exampleSim.pNomoto.delta = -30.0;
        exampleSim.pNomoto.K = 0.5;
        exampleSim.pNomoto.T = 50.0; 
        exampleSim.pSim.time = 1.0;
        exampleSim.pSim.step = 0.1;
        exampleSim.pNomoto.rpm = 15.0;
        exampleSim.pSim.initYaw = 0.0;
        exampleSim.pSim.terminal_output = false;

        exampleSim.nOde.set(exampleSim.pNomoto);
        
        resN = exampleSim.runSim();
        
        int s_result = resN.x_pos.size();

        // Round because of floating point errors
        double yaw = round( resN.yaw.at(s_result-1) * 100 ) / 100; 
    
        REQUIRE((yaw < 0) & (yaw > -90 ));
    }


}

