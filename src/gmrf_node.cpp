//========================================================================================
//	GMRF_node
//	Description: implements the Gaussian Markov rando field gas-distribution mapping algorithm.
//  See: http://mapir.isa.uma.es/mapirwebsite/index.php/mapir-downloads/papers/193	
//		
//	topics subscribed:
//  topics published:
//	services:
//				
//----------------------------------------------------------------------------------------
//----------------------------------------------------------------------------------------
//	Revision log:
//	version: 1.0	16/09/2016
//========================================================================================

#include "gmrf_node.h"

// -------------
// Cgmrf
//--------------
Cgmrf::Cgmrf()
{
    printf("\n=================================================================");
    printf("\n=	GMRF Gas-Distribution Mapping Node\n");
    printf("\n=================================================================\n");

    //------------------
    // Load Parameters
    //------------------
    ros::NodeHandle param_n("~");
    param_n.param<std::string>("frame_id", frame_id, "/map");
    param_n.param<std::string>("occupancy_map_topic", occupancy_map_topic, "/map");
    param_n.param<std::string>("sensor_topic", sensor_topic, "/PID/Sensor_reading");
    param_n.param<std::string>("output_csv_file", output_csv_file, "");
    param_n.param<double>("exec_freq", exec_freq, 2.0 );
    param_n.param<double>("cell_size", cell_size, 0.5);

    param_n.param<double>("GMRF_lambdaPrior", GMRF_lambdaPrior, 0.5);   // [GMRF model] The information (Lambda) of prior factors
    param_n.param<double>("GMRF_lambdaObs", GMRF_lambdaObs, 10.0);       // [GMRF model] The initial information (Lambda) of each observation (this information will decrease with time)
    param_n.param<double>("GMRF_lambdaObsLoss", GMRF_lambdaObsLoss, 0.0);// [GMRF model] The loss of information (Lambda) of the observations with each iteration (see AppTick)

    param_n.param<std::string>("colormap", colormap, "jet");
    param_n.param<int>("max_pclpoints_cell", max_pclpoints_cell, 20);
    param_n.param<double>("min_sensor_val", min_sensor_val, 0.0);
    param_n.param<double>("max_sensor_val", max_sensor_val, 0.0);

    param_n.param<double>("suggest_next_location_sensor_th", suggest_next_location_sensor_th, 0.1);


    ROS_DEBUG("[GMRF] frame_id: %s",  frame_id.c_str());
    ROS_DEBUG("[GMRF] occupancy_map_topic: %s",  occupancy_map_topic.c_str());
    ROS_DEBUG("[GMRF] sensor_topic: %s",  sensor_topic.c_str());
    ROS_DEBUG("[GMRF] output_csv_file: %s",  output_csv_file.c_str());



    //----------------------------------
    // Subscriptions
    //----------------------------------
    sub_sensor = param_n.subscribe(sensor_topic, 1, &Cgmrf::sensorCallback, this);
    occupancyMap_sub = param_n.subscribe(occupancy_map_topic, 1, &Cgmrf::mapCallback, this);
    //----------------------------------
    // Publishers
    //----------------------------------
    mean_advertise = param_n.advertise<sensor_msgs::PointCloud2>("mean_map", 20);
    var_advertise = param_n.advertise<sensor_msgs::PointCloud2>("var_map", 20);
    gas_grid_advertise = param_n.advertise<gas_map_msgs::GasGrid>("gas_grid_map", 20);
    //----------------------------------
    // Services
    //----------------------------------
    //ros::ServiceServer service = param_n.advertiseService("suggestNextObservationLocation", suggestNextObservationLocation);

    module_init = false;
    first_time_get_map = true;
}


Cgmrf::~Cgmrf(){}


//-------------
// CALLBACKS
//-------------
void Cgmrf::sensorCallback(const olfaction_msgs::gas_sensorPtr msg)
{
    mutex_nose.lock();
    try
    {
        if(msg->raw_units == msg->UNITS_PPM)
        {
            //Normalize to [0,1]
            curr_reading = ((double)msg->raw - min_sensor_val)/ (max_sensor_val-min_sensor_val);
            if (curr_reading < 0)
                curr_reading = 0.0;
            if (curr_reading > 1)
                curr_reading = 1.0;
        }
        else if(msg->raw_units == msg->UNITS_OHM)
            curr_reading = (msg->raw_air - msg->raw)/msg->raw_air;  //range[0,1]
        else if(msg->raw_units == msg->UNITS_VOLT)
            curr_reading = msg->raw;
        else if(msg->raw_units == msg->UNITS_AMP)
            curr_reading = msg->raw;
    }catch(std::exception e){
        ROS_ERROR("[GMRF] Exception at new Obs: %s ", e.what() );
    }

    mutex_nose.unlock();


    //Get pose of the sensor in the /map reference system
    tf::StampedTransform transform;
    bool know_sensor_pose = true;
    try
    {
      tf_listener.lookupTransform(frame_id.c_str(),msg->header.frame_id.c_str(), ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("[GMRF] %s",ex.what());
        know_sensor_pose = false;
        ros::Duration(1.0).sleep();
    }


    if (module_init)
    {
        ROS_INFO("[TRACE GMRF] PASS 000");
        //Current sensor pose
        float x_pos = transform.getOrigin().x();
        float y_pos = transform.getOrigin().y();

        //Add new observation to the map
        if (curr_reading <0 || curr_reading > 1)
        {
            ROS_WARN("[GMRF] Obs is out of bouns! %.2f [0,1]. Normalizing!", curr_reading);
            curr_reading = 1.0;
        }
        mutex_nose.lock();
        ROS_INFO("[TRACE GMRF] PASS 001");
        ROS_WARN("[GMRF] New obs: %.2f at (%.2f,%.2f)", curr_reading,x_pos,y_pos);
        my_map->insertObservation_GMRF(curr_reading,x_pos,y_pos, GMRF_lambdaObs);
        mutex_nose.unlock();
        ROS_INFO("[TRACE GMRF] PASS 002");
    }
}



void Cgmrf::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    ROS_INFO("[GMRF] WAIT MAP CALLBACK...");
    while(updating_map);
    ROS_INFO("[GMRF] MAP CALLBACK!!");
    receiving_new_map = true;
    ROS_DEBUG("%s - Got the map of the environment!", __FUNCTION__);
    occupancyMap = *msg;

    //Set GasMap dimensions as the OccupancyMap
    double map_min_x = msg->info.origin.position.x;
    double map_max_x =  msg->info.origin.position.x + msg->info.width*msg->info.resolution;
    double map_min_y =  msg->info.origin.position.y;
    double map_max_y =  msg->info.origin.position.y + msg->info.height*msg->info.resolution;

    //my_map = new CGMRF_map(occupancyMap, cell_size, GMRF_lambdaPrior,colormap,max_pclpoints_cell);
    
    //Create GasMap
    if(first_time_get_map)
    {
        ROS_INFO("[GMRF] GasGridMap is initializing");
        my_map = new CGMRF_map(occupancyMap, cell_size, GMRF_lambdaPrior,colormap,max_pclpoints_cell);
        ROS_INFO("[GMRF] GasGridMap initialized");
        first_time_get_map = false;
    }
    else
    {
        ROS_INFO("[GMRF] New GasGridMap is updating");
        my_map->updateNewMapEstimation_GMRF(occupancyMap,cell_size,GMRF_lambdaPrior,colormap,max_pclpoints_cell);
        ROS_INFO("[GMRF] New GasGridMap updated");
    }

    module_init = true;
    receiving_new_map = false;
    ROS_INFO("[GMRF] MAP CALLBACK DONE");

}


void Cgmrf::publishMaps()
{
    //ROS_INFO("[GMRF] Getting Maps!");
    sensor_msgs::PointCloud2 meanPC, varPC;
    my_map->get_as_pointClouds(meanPC, varPC);
    meanPC.header.frame_id = frame_id.c_str();
    varPC.header.frame_id = frame_id.c_str();

    mean_advertise.publish(meanPC);
    var_advertise.publish(varPC);
    //ROS_INFO("[GMRF] Publishing maps!");

    gas_map_msgs::GasGrid gas_grid;
    my_map->get_as_GasGrid(gas_grid);
    gas_grid.header.frame_id = frame_id.c_str();

    gas_grid_advertise.publish(gas_grid);

    //STORE AS CSV FILE
    if (output_csv_file != "")
    {
        my_map->store_as_CSV(output_csv_file);
    }
}


//----------------------------
//      MAIN
//----------------------------
int main(int argc, char **argv)
{
	ros::init(argc, argv, "gmrf_node");
	Cgmrf my_gmrf_map;

	ROS_INFO("[gmrf] LOOP....");
	ros::Time last_publication_time = ros::Time::now();
	ros::Rate loop_rate(my_gmrf_map.exec_freq);
	
	while (ros::ok())
	{
		ros::spinOnce();                    //Callbacks & Services
        ROS_INFO("[GMRF] PASS 001 (WAIT UPDATING MAP...)");
        while(receiving_new_map);
		if (my_gmrf_map.module_init)
		{
            updating_map = true;
            ROS_INFO("[GMRF] PASS 001 (UPDATING MAP!!!)");
			// Update and Publish maps
            ros::Time update_time_begin = ros::Time::now();
            my_gmrf_map.my_map->updateMapEstimation_GMRF(my_gmrf_map.GMRF_lambdaObsLoss);
            ros::Time update_time_end = ros::Time::now();
            ros::Duration update_time_duration = update_time_end - update_time_begin;
            ROS_INFO("Update spent time: %lf s",update_time_duration.toSec());\
            
            ROS_INFO("[GMRF] PASS 002");
            ros::Time publish_time_begin = ros::Time::now();
            my_gmrf_map.publishMaps();
            ros::Time publish_time_end = ros::Time::now();
            ros::Duration publish_time_duration = publish_time_end - publish_time_begin;
            ROS_INFO("Publish spent time: %lf s",publish_time_duration.toSec());

			updating_map = false;
            ROS_INFO("[GMRF] PASS 001 (UPDATING MAP DONE)");

			// Info about actual rate
			ROS_INFO("[gmrf] Updating every %f seconds. Intended preiod was %f", (ros::Time::now()-last_publication_time).toSec(), 1.0/my_gmrf_map.exec_freq);
			last_publication_time = ros::Time::now();
		}
        else
        {
			ROS_INFO("[gmrf] Waiting for initialization (Map of environment).");
		}
        loop_rate.sleep();
	}
}







