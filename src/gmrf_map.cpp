#include "gmrf_map.h"




/*---------------------------------------------------------------
                        Constructor
  ---------------------------------------------------------------*/

CGMRF_map::CGMRF_map(const nav_msgs::OccupancyGrid &oc_map, float cell_size, float m_lambdaPrior, std::string m_colormap, int max_points_cell)
{
    try
    {
        m_resolution = cell_size;
        GMRF_lambdaPrior = m_lambdaPrior;

        //Set GasMap dimensions as the OccupancyMap
        double x_min =  oc_map.info.origin.position.x;
        double x_max =  oc_map.info.origin.position.x + oc_map.info.width*oc_map.info.resolution;
        double y_min =  oc_map.info.origin.position.y;
        double y_max =  oc_map.info.origin.position.y + oc_map.info.height*oc_map.info.resolution;

        // Adjust sizes to adapt them to full sized cells acording to the resolution:
        m_x_min = cell_size*round(x_min/cell_size);
        m_y_min = cell_size*round(y_min/cell_size);
        m_x_max = cell_size*round(x_max/cell_size);
        m_y_max = cell_size*round(y_max/cell_size);

        // Res:
        res_coef = m_resolution / oc_map.info.resolution;

        // Now the number of cells should be integers:
        m_size_x = round((m_x_max-m_x_min)/m_resolution);
        m_size_y = round((m_y_max-m_y_min)/m_resolution);
        N = m_size_x*m_size_y;

        // Init the map container
        //-----------------------
        TRandomFieldCell init_cell;
        init_cell.mean = 0.0;
        init_cell.std = 0.0;
        m_map.assign(N, init_cell);
        ROS_INFO("[CGMRF] Map created: %u cells, x=(%.2f,%.2f) y=(%.2f,%.2f)", static_cast<unsigned int>(m_map.size()), m_x_min, m_x_max, m_y_min, m_y_max);


        //---------------------
        // INIT RANDOM FIELD
        //---------------------
        ROS_INFO("[CGMRF] Generating Prior based on 'Squared Differences' ");
        //Set initial factors: L "prior factors" + 0 "Observation factors"
        nPriorFactors = (m_size_x -1) * m_size_y + m_size_x * (m_size_y -1);	// L = (Nr-1)*Nc + Nr*(Nc-1); Full connected
        nObsFactors = 0;                                                        // M
        nFactors = nPriorFactors + nObsFactors;
        ROS_INFO("[CGMRF] %lu factors for a map size of N=%lu", nFactors, N);


        //Initialize H_prior, gradient = 0, and the vector of active observations = empty
        H_prior.clear();
        H_prior.reserve(nPriorFactors);
        g.resize(N);			//Initially the gradient is all 0's
        g.fill(0.0);
        activeObs.resize(N);	//No initial Observations

        ROS_INFO("[CGMRF] PASS 001");
        //-------------------------------------
        // Load default values for H_prior:
        //-------------------------------------
        //Use region growing algorithm to determine the gascell interconnections (Factors)
        size_t cx = 0;
        size_t cy = 0;
        size_t cxoj_min, cxoj_max, cyoj_min, cyoj_max, seed_cxo, seed_cyo;				//Cell j limits in the Occupancy
        size_t cxoi_min, cxoi_max, cyoi_min, cyoi_max, objective_cxo, objective_cyo;	//Cell i limits in the Occupancy
        size_t cxo_min, cxo_max, cyo_min, cyo_max;										//Cell i+j limits in the Occupancy

        for (size_t j=0; j<N; j++)		//For each cell in the gas_map
        {
            // Get cell_j indx-limits in Occuppancy gridmap
            cxoj_min = floor(cx*res_coef);
            cxoj_max = cxoj_min + ceil(res_coef-1);
            cyoj_min = floor(cy*res_coef);
            cyoj_max = cyoj_min + ceil(res_coef-1);

            seed_cxo = cxoj_min + ceil(res_coef/2-1);
            seed_cyo = cyoj_min + ceil(res_coef/2-1);


            //If a cell is free then add observation with very low information
            //to force non-visited cells to have a 0.0 mean
            //The map data, in row-major order, starting with (0,0).  Occupancy
            //probabilities are in the range [0,100].  Unknown is -1.
            size_t cell_idx = seed_cxo + seed_cyo*m_size_x;
            if ( oc_map.data[cell_idx] < 50.0 )
            {
                TobservationGMRF new_obs;
                new_obs.obsValue = 0.0;
                new_obs.Lambda = 10e-10f;
                new_obs.time_invariant = true;	//Obs that will not dissapear with time.
                activeObs[j].push_back(new_obs);
            }


            //Factor with the right node: H_ji = - Lamda_prior
            //-------------------------------------------------
            if (cx<(m_size_x-1))
            {
                size_t i = j+1;
                size_t cxi = cx+1;
                size_t cyi = cy;

                // Get cell_i indx-limits in Occuppancy gridmap
                cxoi_min = floor(cxi*res_coef);
                cxoi_max = cxoi_min + ceil(res_coef-1);
                cyoi_min = floor(cyi*res_coef);
                cyoi_max = cyoi_min + ceil(res_coef-1);

                objective_cxo = cxoi_min + ceil(res_coef/2-1);
                objective_cyo = cyoi_min + ceil(res_coef/2-1);

                //Get overall indx of both cells together
                cxo_min = std::min(cxoj_min, cxoi_min );
                cxo_max = std::max(cxoj_max, cxoi_max );
                cyo_min = std::min(cyoj_min, cyoi_min );
                cyo_max = std::max(cyoj_max, cyoi_max );

                //Check using Region growing if cell j is connected to cell i (Occupancy gridmap)
                if( exist_relation_between2cells(&oc_map, cxo_min,cxo_max,cyo_min,cyo_max,seed_cxo,seed_cyo,objective_cxo,objective_cyo))
                {
                    Eigen::Triplet<double> Hentry(i,j, -GMRF_lambdaPrior);
                    H_prior.push_back( Hentry );

                    //Save relation between cells
                    cell_interconnections.insert ( std::pair<size_t,size_t>(j,i) );
                    cell_interconnections.insert ( std::pair<size_t,size_t>(i,j) );
                }
            }

            //Factor with the upper node: H_ji = - Lamda_prior
            //-------------------------------------------------
            if (cy<(m_size_y-1))
            {
                size_t i = j+m_size_x;
                size_t cxi = cx;
                size_t cyi = cy+1;

                // Get cell_i indx-limits in Occuppancy gridmap
                cxoi_min = floor(cxi*res_coef);
                cxoi_max = cxoi_min + ceil(res_coef-1);
                cyoi_min = floor(cyi*res_coef);
                cyoi_max = cyoi_min + ceil(res_coef-1);

                objective_cxo = cxoi_min + ceil(res_coef/2-1);
                objective_cyo = cyoi_min + ceil(res_coef/2-1);

                //Get overall indx-limits of both cells together
                cxo_min = std::min(cxoj_min, cxoi_min );
                cxo_max = std::max(cxoj_max, cxoi_max );
                cyo_min = std::min(cyoj_min, cyoi_min );
                cyo_max = std::max(cyoj_max, cyoi_max );


                //Check using Region growing if cell j is connected to cell i (Occupancy gridmap)
                if( exist_relation_between2cells(&oc_map, cxo_min,cxo_max,cyo_min,cyo_max,seed_cxo,seed_cyo,objective_cxo,objective_cyo))
                {
                    Eigen::Triplet<double> Hentry(i,j, -GMRF_lambdaPrior);
                    H_prior.push_back( Hentry );

                    //Save relation
                    cell_interconnections.insert ( std::pair<size_t,size_t>(j,i) );
                    cell_interconnections.insert ( std::pair<size_t,size_t>(i,j) );
                }
            }

            //Factors of cell_j: H_jj = Nº factors * Lambda_prior
            //----------------------------------------------------
            std::pair < std::multimap<size_t,size_t>::iterator, std::multimap<size_t,size_t>::iterator > range;
            range = cell_interconnections.equal_range(j);
            size_t nFactors_j = 0;
            while ( range.first!=range.second )
            {
                nFactors_j++;
                range.first++;
            }
            Eigen::Triplet<double> Hentry(j,j , nFactors_j * GMRF_lambdaPrior );
            H_prior.push_back( Hentry );
            //std::cout << H_prior << std::endl;
            // Increment j coordinates (row(x), col(y))
            if (++cx>=m_size_x)
            {
                cx=0;
                cy++;
            }
        } // end for "j"

        ROS_INFO("[CGMRF] PASS 002");
        //DEBUG - Save cell interconnections to file
        /*
        std::ofstream myfile;
        myfile.open("/home/jgmonroy/GMRF.txt");
        for (std::multimap<size_t,size_t>::iterator it=cell_interconnections.begin(); it!=cell_interconnections.end(); ++it)
            myfile << (*it).first << " " << (*it).second << '\n';
        myfile.close();
        */

        //Pre-generate Point-Clouds for visualization
        init_pcl_templates(m_colormap, max_points_cell);
        ROS_INFO("[CGMRF] PASS 003");

    }catch(std::exception e){
        ROS_ERROR("[GMRF-Constructor] Exception: %s ", e.what() );
    }
}



CGMRF_map::~CGMRF_map(){}



bool CGMRF_map::exist_relation_between2cells(
    const nav_msgs::OccupancyGrid *m_Ocgridmap,
    size_t cxo_min,
    size_t cxo_max,
    size_t cyo_min,
    size_t cyo_max,
    const size_t seed_cxo,
    const size_t seed_cyo,
    const size_t objective_cxo,
    const size_t objective_cyo)
{
    try
    {
        //ROS_INFO("Checking relation between cells (%i,%i) and (%i,%i)", seed_cxo,seed_cyo,objective_cxo,objective_cyo);

        //Ensure delimited region is within the Occupancy map
        cxo_min = std::max (cxo_min, (size_t)0);
        cxo_max = std::min (cxo_max, (size_t)m_Ocgridmap->info.width-1);
        cyo_min = std::max (cyo_min, (size_t)0);
        cyo_max = std::min (cyo_max, (size_t)m_Ocgridmap->info.height-1);

        //ROS_INFO("Under gridlimits cx=(%i,%i) and cy=(%i,%i) \n", cxo_min,cxo_max,cyo_min,cyo_max);

        //Check that seed and objective are inside the delimited Occupancy gridmap
        if( (seed_cxo < cxo_min) || (seed_cxo >= cxo_max) || (seed_cyo < cyo_min) || (seed_cyo >= cyo_max) )
        {
            //cout << "Seed out of bounds (false)" << endl;
            return false;
        }
        if( (objective_cxo < cxo_min) || (objective_cxo >= cxo_max) || (objective_cyo < cyo_min) || (objective_cyo >= cyo_max) )
        {
            //cout << "Objective out of bounds (false)" << endl;
            return false;
        }

        // Check that seed and obj have similar occupancy (0,1)

        if ( (m_Ocgridmap->data[seed_cxo + seed_cyo*m_Ocgridmap->info.width]<50.0) != (m_Ocgridmap->data[objective_cxo + objective_cyo*m_Ocgridmap->info.width]<50.0) )
        {
            //cout << "Seed and objective have diff occupation (false)" << endl;
            return false;
        }


        //Create Matrix for region growing (row,col)
        Eigen::MatrixXi matExp(cxo_max-cxo_min+1, cyo_max-cyo_min+1);
        //mrpt::math::CMatrixUInt matExp(cxo_max-cxo_min+1, cyo_max-cyo_min+1);
        //cout << "Matrix creted with dimension:" << matExp.getRowCount() << " x " << matExp.getColCount() << endl;
        //CMatrix matExp(cxo_max-cxo_min+1, cyo_max-cyo_min+1);
        //matExp.fill(0);
        matExp.setZero();

        //Add seed
        matExp(seed_cxo-cxo_min,seed_cyo-cyo_min) = 1;
        int seedsOld = 0;
        int seedsNew = 1;

        //NOT VERY EFFICIENT!!
        while (seedsOld < seedsNew)
        {
            seedsOld = seedsNew;

            for (size_t col=0; col<matExp.cols(); col++)
            {
                for (size_t row=0; row<matExp.rows(); row++)
                {
                    //test if cell needs to be expanded
                    if( matExp(row,col) == 1)
                    {
                        matExp(row,col) = 2;	//mark as expanded
                        //check if neighbourds have similar occupancy (expand)
                        for (int i=-1;i<=1;i++)
                        {
                            for (int j=-1;j<=1;j++)
                            {
                                //check that neighbour is inside the map
                                if( (int(row)+j>=0) && (int(row)+j<=int(matExp.rows()-1)) && (int(col)+i>=0) && (int(col)+i<=int(matExp.cols())-1) )
                                {
                                    if( !( (i==0 && j==0) || !(matExp(row+j,col+i)==0) ))
                                    {
                                        //check if expand


                                        if ( (m_Ocgridmap->data[row+cxo_min + (col+cyo_min)*m_Ocgridmap->info.width]<50.0) == (m_Ocgridmap->data[row+j+cxo_min + (col+i+cyo_min)*m_Ocgridmap->info.width]<50.0))
                                        {
                                            if ( (row+j+cxo_min == objective_cxo) && (col+i+cyo_min == objective_cyo) )
                                            {
                                                //cout << "Connection Success (true)" << endl;
                                                return true;		//Objective connected
                                            }
                                            matExp(row+j,col+i) = 1;
                                            seedsNew++;
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
        //if not connection to he objective is found, then return false
        //cout << "Connection not found (false)" << endl;
        return false;
    }catch(std::exception e){
        ROS_ERROR("[GMRF] Exception while checking cells interconnections: %s ", e.what() );
    }
}


void CGMRF_map::insertObservation_GMRF(float normReading, float x_pos, float y_pos, float lambdaObs)
{
    try{
        //Get index of observed cell
        const int cellIdx = xy2idx( x_pos, y_pos );

        // Insert observation in the vector of Active Observations
        TobservationGMRF new_obs;
        new_obs.obsValue = normReading;
        new_obs.Lambda = lambdaObs;
        new_obs.time_invariant = false;		//Default behaviour, the obs will lose weight with time.
        //ROS_INFO("[GMRF] Adding obs %.2f at cell_idx %i", normReading, cellIdx);
        activeObs[cellIdx].push_back(new_obs);


    }catch(std::exception e){
        ROS_ERROR("[GMRF] Exception while Inserting new Observation: %s ", e.what() );
    }
}



void CGMRF_map::updateNewMapEstimation_GMRF(const nav_msgs::OccupancyGrid &oc_map, float cell_size, float m_lambdaPrior, std::string m_colormap, int max_points_cell)
{
    try
    {
        ROS_INFO("[NGMRF] Prev map: %u cells, x=(%.2f,%.2f) y=(%.2f,%.2f)", static_cast<unsigned int>(m_map.size()), m_x_min, m_x_max, m_y_min, m_y_max);

        //m_resolution = cell_size;
        GMRF_lambdaPrior = m_lambdaPrior;

        //Set GasMap dimensions as the OccupancyMap
        double x_min =  oc_map.info.origin.position.x;
        double x_max =  oc_map.info.origin.position.x + oc_map.info.width*oc_map.info.resolution;
        double y_min =  oc_map.info.origin.position.y;
        double y_max =  oc_map.info.origin.position.y + oc_map.info.height*oc_map.info.resolution;

        // Adjust sizes to adapt them to full sized cells acording to the resolution:
        float new_m_x_min = cell_size*round(x_min/cell_size);
        float new_m_y_min = cell_size*round(y_min/cell_size);
        float new_m_x_max = cell_size*round(x_max/cell_size);
        float new_m_y_max = cell_size*round(y_max/cell_size);

        // Res:
        res_coef = m_resolution / oc_map.info.resolution;

        // Now the number of cells should be integers:
        size_t new_m_size_x = round((new_m_x_max-new_m_x_min)/m_resolution);
        size_t new_m_size_y = round((new_m_y_max-new_m_y_min)/m_resolution);
        size_t new_N = new_m_size_x*new_m_size_y;

        // Init the map container
        //-----------------------
        TRandomFieldCell init_cell;
        init_cell.mean = 0.0;
        init_cell.std = 0.0;
        std::vector<TRandomFieldCell> new_m_map;
        new_m_map.assign(new_N, init_cell);
        ROS_INFO("[NGMRF] New map: %u cells, x=(%.2f,%.2f) y=(%.2f,%.2f)", static_cast<unsigned int>(new_m_map.size()), new_m_x_min, new_m_x_max, new_m_y_min, new_m_y_max);

        //---------------------
        // INIT RANDOM FIELD
        //---------------------
        ROS_INFO("[NGMRF] Generating Prior based on 'Squared Differences' ");
        //Set initial factors: L "prior factors" + 0 "Observation factors"
        size_t new_nPriorFactors = (new_m_size_x -1) * new_m_size_y + new_m_size_x * (new_m_size_y -1);    // L = (Nr-1)*Nc + Nr*(Nc-1); Full connected
        size_t new_nObsFactors = 0;    
        size_t new_nFactors;                                              // M
        new_nFactors = new_nPriorFactors + new_nObsFactors;
        ROS_INFO("[NGMRF] %lu factors for a map size of N=%lu", new_nFactors, new_N);

        std::vector<Eigen::Triplet<double> > new_H_prior;        // the prior part of H

        //Initialize H_prior, gradient = 0, and the vector of active observations = empty
        new_H_prior.clear();
        new_H_prior.reserve(new_nPriorFactors);
        //Eigen::VectorXd new_g;
        //new_g.resize(new_N);            //Initially the gradient is all 0's
        //new_g.fill(0.0);
        //g.resize(new_N);
        //g.fill(0.0);
        std::vector<std::vector<TobservationGMRF> > new_activeObs;      //Vector with the active observations and their respective Information
        new_activeObs.resize(new_N);    //No initial Observations

        ROS_INFO("[NGMRF] PASS 001");
        //-------------------------------------
        // Load default values for H_prior:
        //-------------------------------------
        //Use region growing algorithm to determine the gascell interconnections (Factors)
        size_t cx = 0;
        size_t cy = 0;
        size_t cxoj_min, cxoj_max, cyoj_min, cyoj_max, seed_cxo, seed_cyo;              //Cell j limits in the Occupancy
        size_t cxoi_min, cxoi_max, cyoi_min, cyoi_max, objective_cxo, objective_cyo;    //Cell i limits in the Occupancy
        size_t cxo_min, cxo_max, cyo_min, cyo_max;                                      //Cell i+j limits in the Occupancy

        int left_bound_x = round((m_x_min-new_m_x_min)/m_resolution);
        int right_bound_x = left_bound_x+round((m_x_max-m_x_min)/m_resolution);
        int left_bound_y = round((m_y_min-new_m_y_min)/m_resolution);
        int right_bound_y = left_bound_y+round((m_y_max-m_y_min)/m_resolution);
        cout <<"[NGMRF] ("<< left_bound_x <<","<<right_bound_x<<"):("<<left_bound_y <<","<<right_bound_y<<")"<<endl;
        int old_j = 0;
        cell_interconnections.clear();
        ROS_INFO("[NGMRF] PASS 001.1");
        //cout << "new_N: " << new_N << endl;
        for (size_t j=0; j<new_N; j++)      //For each cell in the gas_map
        {
            //std::cout << "[IDX]: (" << cx << "," << cy << ")" << std::endl;
            //cout << j;
            // Get cell_j indx-limits in Occuppancy gridmap
            cxoj_min = floor(cx*res_coef);
            cxoj_max = cxoj_min + ceil(res_coef-1);
            cyoj_min = floor(cy*res_coef);
            cyoj_max = cyoj_min + ceil(res_coef-1);

            seed_cxo = cxoj_min + ceil(res_coef/2-1);
            seed_cyo = cyoj_min + ceil(res_coef/2-1);

            if((left_bound_x <= cx) && (cx < right_bound_x) && (left_bound_y <= cy) && (cy < right_bound_y))
            {
                //new_g[j] = g[old_j];
                //cout << "[TRACE] g success" << endl;
                
                new_m_map[j] = m_map[old_j];
                //cout << "(";
                //new_m_map[j].mean = m_map[old_j].mean;
                //cout << "[TRACE] mean success: [" << old_j <<"]:"<<m_map[old_j].mean <<endl;
                //new_m_map[j].std = m_map[old_j].std;
                //cout << "[TRACE] std success" << endl;
                //cout << "[" << activeObs[old_j].size() << "]";
                //for(int p=0;p<activeObs[old_j].size();p++)
                //{
                //    cout << p <<","<< activeObs[old_j][p].obsValue<<".";// << "," << activeObs[old_j][p].Lambda << "," << activeObs[old_j][p].time_invariant;
                //}
                new_activeObs[j] = activeObs[old_j];
                //cout << ")";
                //cout << "[TRACE] active obs success" << endl;
                old_j++;
            }

            //If a cell is free then add observation with very low information
            //to force non-visited cells to have a 0.0 mean
            //The map data, in row-major order, starting with (0,0).  Occupancy
            //probabilities are in the range [0,100].  Unknown is -1.
            size_t cell_idx = seed_cxo + seed_cyo*new_m_size_x;
            if ( oc_map.data[cell_idx] < 50.0 )
            {
                TobservationGMRF new_obs;
                new_obs.obsValue = 0.0;
                new_obs.Lambda = 10e-10f;
                new_obs.time_invariant = true;  //Obs that will not dissapear with time.
                new_activeObs[j].push_back(new_obs);
            }

            //cout << ",";
            //Factor with the right node: H_ji = - Lamda_prior
            //-------------------------------------------------
            if (cx<(new_m_size_x-1))
            {
                size_t i = j+1;
                size_t cxi = cx+1;
                size_t cyi = cy;

                // Get cell_i indx-limits in Occuppancy gridmap
                cxoi_min = floor(cxi*res_coef);
                cxoi_max = cxoi_min + ceil(res_coef-1);
                cyoi_min = floor(cyi*res_coef);
                cyoi_max = cyoi_min + ceil(res_coef-1);

                objective_cxo = cxoi_min + ceil(res_coef/2-1);
                objective_cyo = cyoi_min + ceil(res_coef/2-1);

                //Get overall indx of both cells together
                cxo_min = std::min(cxoj_min, cxoi_min );
                cxo_max = std::max(cxoj_max, cxoi_max );
                cyo_min = std::min(cyoj_min, cyoi_min );
                cyo_max = std::max(cyoj_max, cyoi_max );

                //Check using Region growing if cell j is connected to cell i (Occupancy gridmap)
                if( exist_relation_between2cells(&oc_map, cxo_min,cxo_max,cyo_min,cyo_max,seed_cxo,seed_cyo,objective_cxo,objective_cyo))
                {
                    Eigen::Triplet<double> Hentry(i,j, -GMRF_lambdaPrior);
                    new_H_prior.push_back( Hentry );

                    //Save relation between cells
                    cell_interconnections.insert ( std::pair<size_t,size_t>(j,i) );
                    cell_interconnections.insert ( std::pair<size_t,size_t>(i,j) );
                }
            }

            //cout << ".";
            //Factor with the upper node: H_ji = - Lamda_prior
            //-------------------------------------------------
            if (cy<(new_m_size_y-1))
            {
                size_t i = j+new_m_size_x;
                size_t cxi = cx;
                size_t cyi = cy+1;

                // Get cell_i indx-limits in Occuppancy gridmap
                cxoi_min = floor(cxi*res_coef);
                cxoi_max = cxoi_min + ceil(res_coef-1);
                cyoi_min = floor(cyi*res_coef);
                cyoi_max = cyoi_min + ceil(res_coef-1);

                objective_cxo = cxoi_min + ceil(res_coef/2-1);
                objective_cyo = cyoi_min + ceil(res_coef/2-1);

                //Get overall indx-limits of both cells together
                cxo_min = std::min(cxoj_min, cxoi_min );
                cxo_max = std::max(cxoj_max, cxoi_max );
                cyo_min = std::min(cyoj_min, cyoi_min );
                cyo_max = std::max(cyoj_max, cyoi_max );


                //Check using Region growing if cell j is connected to cell i (Occupancy gridmap)
                if( exist_relation_between2cells(&oc_map, cxo_min,cxo_max,cyo_min,cyo_max,seed_cxo,seed_cyo,objective_cxo,objective_cyo))
                {
                    Eigen::Triplet<double> Hentry(i,j, -GMRF_lambdaPrior);
                    new_H_prior.push_back( Hentry );

                    //Save relation
                    cell_interconnections.insert ( std::pair<size_t,size_t>(j,i) );
                    cell_interconnections.insert ( std::pair<size_t,size_t>(i,j) );
                }
            }

            //cout << "<";
            //Factors of cell_j: H_jj = Nº factors * Lambda_prior
            //----------------------------------------------------
            std::pair < std::multimap<size_t,size_t>::iterator, std::multimap<size_t,size_t>::iterator > range;
            range = cell_interconnections.equal_range(j);
            size_t nFactors_j = 0;
            while ( range.first!=range.second )
            {
                nFactors_j++;
                range.first++;
            }
            Eigen::Triplet<double> Hentry(j,j , nFactors_j * GMRF_lambdaPrior );
            new_H_prior.push_back( Hentry );
            //std::cout << H_prior << std::endl;
            // Increment j coordinates (row(x), col(y))
            //cout << "[TRACE] trace 3" << endl;
            //cout << ">";

            if (++cx>=new_m_size_x)
            {
                cx=0;
                cy++;
            }

        } // end for "j"
        //cout << endl;
        ROS_INFO("[NGMRF] PASS 001.5");
        //g.resize(new_N);
        //g = new_g;
        m_map.resize(new_N);
        m_map = new_m_map;
        activeObs.resize(new_N);
        activeObs = new_activeObs;
        g.resize(new_N);
        g.fill(0.0);
        N = new_N;
        m_x_min = new_m_x_min;
        m_x_max = new_m_x_max;
        m_y_min = new_m_y_min;
        m_y_max = new_m_y_max;
        m_size_x = new_m_size_x;
        m_size_y = new_m_size_y;
        H_prior.reserve(new_nPriorFactors);
        H_prior = new_H_prior;
        nPriorFactors = new_nPriorFactors;
        nFactors = new_nFactors;
        nObsFactors = new_nObsFactors;
        ROS_INFO("[NGMRF] PASS 002");
        
        for(size_t j=0;j<N;j++)
        {
            cout << m_map[j].std << " ";
            if((j+1)% m_size_x == 0)
                cout << endl;
        }
        
        //DEBUG - Save cell interconnections to file
        /*
        std::ofstream myfile;
        myfile.open("/home/jgmonroy/GMRF.txt");
        for (std::multimap<size_t,size_t>::iterator it=cell_interconnections.begin(); it!=cell_interconnections.end(); ++it)
            myfile << (*it).first << " " << (*it).second << '\n';
        myfile.close();
        */

        //Pre-generate Point-Clouds for visualization
        init_pcl_templates(m_colormap, max_points_cell);
        ROS_INFO("[NGMRF] PASS 003");

    }catch(std::exception e){
        ROS_ERROR("[NGMRF-Constructor] Exception: %s ", e.what() );
    }
}



/*---------------------------------------------------------------
                    updateMapEstimation_GMRF
  ---------------------------------------------------------------*/
void  CGMRF_map::updateMapEstimation_GMRF(float lambdaObsLoss)
{
    try
    {
        //------------------
        //  1- HESSIAN
        //------------------
        ROS_INFO("[GMRF] H_prior");
        //Build Sparse Hessian H, from list of triplets (Hprior):
        std::vector<Eigen::Triplet<double> > H_tri(H_prior.size());
        H_tri.reserve(H_prior.size()+N);
        std::copy(H_prior.begin(), H_prior.end(), H_tri.begin());

        ROS_INFO("[GMRF] H_obs");
        //Add H_obs
        for (size_t j=0; j<N; j++)
        {
            //Sum the information of all observations on cell j
            std::vector<TobservationGMRF>::iterator ito;
            float Lambda_obs_j = 0.0;
            for (ito = activeObs[j].begin(); ito !=activeObs[j].end(); ++ito)
                Lambda_obs_j += ito->Lambda;

            if (Lambda_obs_j != 0.0)
                H_tri.push_back( Eigen::Triplet<double>(j,j, Lambda_obs_j)); // Duplicated entries (from obs + from the prior) will be summed in setFromTriplets()
        }

        Eigen::SparseMatrix<double> Hsparse(N,N);				// declares a column-major sparse matrix type of float
        Hsparse.setFromTriplets(H_tri.begin(), H_tri.end() );




        //------------------
        //  2- GRADIENT
        //------------------
        ROS_INFO("[GMRF] gradient");
        //Reset and Built Gradient Vector
        g.setZero();
        size_t cx = 0;
        size_t cy = 0;
        for (size_t j=0; j<N; j++)
        {
            //A- Gradient due to Observations
            std::vector<TobservationGMRF>::iterator ito;
            for (ito = activeObs[j].begin(); ito !=activeObs[j].end(); ++ito)
                g[j] += ((m_map[j].mean - ito->obsValue) * ito->Lambda);


            //B- Gradient due to Prior
            //Consider only cells correlated with the cell j
            std::pair < std::multimap<size_t,size_t>::iterator, std::multimap<size_t,size_t>::iterator > range;
            range = cell_interconnections.equal_range(j);
            while ( range.first!=range.second )
            {
                size_t cell_i_indx = range.first->second;
                g[j] += ( m_map[j].mean - m_map[cell_i_indx].mean) * GMRF_lambdaPrior;
                range.first++;
            }


            // Increment j coordinates (row(x), col(y))
            if (++cx>=m_size_x)
            {
                cx=0;
                cy++;
            }
        }//end-for



        ROS_INFO("[GMRF] Chol");
        //Cholesky Factorization of Hessian --> Realmente se hace: chol( P * H * inv(P) )
        Eigen::SimplicialLLT< Eigen::SparseMatrix <double> > solver;
        solver.compute(Hsparse);
        // Solve System:    m = m + H\(-g);
        // Note: we solve for (+g) to avoid creating a temporary "-g", then we'll substract the result in m_inc instead of adding it:

        // ====================

        // H.m = g
        // ====================
        Eigen::VectorXd m_inc = solver.solve(g);
        
        bool g_is_null = true;
        for(int i=0;i<N;i++)
        {
            if(g[i] != 0)
            {
                g_is_null = false;
                break;
            }
        }
        if(g_is_null)
        {
            ROS_INFO("g is NULL");
            throw 2;
        }
        

        ROS_INFO("[GMRF] variance");
        //std::cout << m_inc << std::endl;
        ROS_INFO("+++++++++++++++");
        //std::cout << g << std::endl;
        //std::cout << Hsparse << std::endl;
        /*
        ofstream myfile;
        myfile.open("/home/yaqub/yaqub_ws/matrix.txt");
        for(int i=0;i<N;i++)
        {
            for(int j=0;j<N;j++)
            {
                myfile << Hsparse.coeff(i,j) << " ";
            }
            myfile << endl;
        }

        myfile.close();
        for(int i=0;i<N;i++)
        {
            
            if(Hsparse.coeff(i,i) == 0)
                Hsparse.coeffRef(i,i) = 1;
            std:: cout << Hsparse.coeff(i,i) << " ";
        }
        std::cout << std:: endl;
        */
        // VARIANCE SIGMA = inv(P)*inv(P*H*inv(P)) * P
        //Get triangular supperior P*H*inv(P) = UT' * UT = P * R'*R * inv(P)
        ROS_INFO("[GMRF] variance 000.0");
        cout << g.size() <<" "<< Hsparse.size()<< endl;
        Eigen::SparseMatrix<double> UT = solver.matrixU();
        ROS_INFO("[GMRF] variance 000.1");
        Eigen::SparseMatrix<double> Sigma(N,N);								//Variance Matrix
        ROS_INFO("[GMRF] variance 000.2");
        Sigma.reserve(UT.nonZeros());

        // TODO: UT.coeff() implies a heavy time cost in sparse matrices... the following
        //       should be rewritten for efficiency exploiting the knowledge about the nonzero pattern.
        ROS_INFO("[GMRF] variance 000");
        //Apply custom equations to obtain the inverse -> inv( P*H*inv(P) )
        for (int l=N-1; l>=0; l--)
        {
            //Computes variances in the inferior submatrix of "l"
            double subSigmas = 0.0;
            for(size_t j=l+1; j<N; j++)
            {
                if (UT.coeff(l,j) != 0)
                {
                    //Compute off-diagonal variances Sigma(j,l) = Sigma(l,j);
                    //ROS_INFO("[GMRF] variance 001");
                    //SUM 1
                    double sum = 0.0;
                    for(size_t i=l+1; i<=j; i++)
                    {
                        if( UT.coeff(l,i) !=0 )
                        {
                            sum += UT.coeff(l,i) * Sigma.coeff(i,j);
                        }
                    }
                    //ROS_INFO("[GMRF] variance 002");
                    //SUM 2
                    for(size_t i=j+1; i<N; ++i)
                    {
                        if( UT.coeff(l,i) !=0 )
                        {
                            sum += UT.coeff(l,i) * Sigma.coeff(j,i);
                        }
                    }
                    //Save off-diagonal variance (only Upper triangular)
                    Sigma.insert(l,j) = ( -sum/UT.coeff(l,l) );
                    subSigmas += UT.coeff(l,j) * Sigma.coeff(l,j);
                }
            }

            Sigma.insert(l,l) = (1/UT.coeff(l,l)) * ( 1/UT.coeff(l,l) - subSigmas );
        }


        ROS_INFO("[GMRF] update");
        // Update Mean-Variance and force (0 < m_map[i].mean < 1)
        for (size_t j=0; j<N; j++)
        {
            // Recover the diagonal covariance values, undoing the permutation:
            int idx = solver.permutationP().indices().coeff(j);
            const double variance = Sigma.coeff(idx,idx);

            m_map[j].std = std::sqrt(variance);
            m_map[j].mean -= m_inc[j]; // "-" because we solved for "+grad" instead of "-grad".
            if (m_map[j].mean >1) m_map[j].mean = 1.0;
            if (m_map[j].mean <0) m_map[j].mean = 0.0;
        }


        ROS_INFO("[GMRF] TimeVariant");
        // Update Information/Strength of Active Observations
        //---------------------------------------------------------
        for (size_t j=0; j<activeObs.size(); j++)
        {
            std::vector<TobservationGMRF>::iterator ito = activeObs[j].begin();
            while ( ito!=activeObs[j].end() )
            {
                if (ito->time_invariant==false)
                {
                    ito->Lambda -= lambdaObsLoss;
                    if (ito->Lambda <= 0.0)
                        ito = activeObs[j].erase(ito);
                    else
                        ++ito;
                }else
                    ++ito;
            }
        }
    }catch(int e){//std::exception e){
        //ROS_ERROR("[GMRF] Exception Updating the maps: %s ", e.what() );
        if(e == 2)
            ROS_ERROR("ERRORRRRRRRRRRRRR g null");
    }
}



//To improve speed Pre-generate different PCL-Templates based on selected color scheme
void CGMRF_map::init_pcl_templates(std::string colormap, int max_points_cell)
{
    //------------------------------------------
    //Build the template pointclouds (colormap)
    //------------------------------------------
    float color_r[NUM_CELL_TEMPLATES];
    float color_g[NUM_CELL_TEMPLATES];
    float color_b[NUM_CELL_TEMPLATES];
    if(colormap.compare("jet")==0){
        float temp_color_r[NUM_CELL_TEMPLATES]={0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.02,0.04,0.06,0.08,0.10,0.12,0.14,0.16,0.18,0.20,0.22,0.24,0.26,0.28,0.30,0.32,0.34,0.36,0.38,0.40,0.42,0.44,0.46,0.48,0.50,0.52,0.54,0.56,0.58,0.60,0.62,0.64,0.66,0.68,0.70,0.72,0.74,0.76,0.78,0.80,0.82,0.84,0.86,0.88,0.90,0.92,0.94,0.96,0.98,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,0.98,0.96,0.94,0.92,0.90,0.88,0.86,0.84,0.82,0.80,0.78,0.76,0.74,0.72,0.70,0.68,0.66,0.64,0.62,0.60,0.58,0.56,0.54,0.52,0.50};
        float temp_color_g[NUM_CELL_TEMPLATES]={0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.02,0.04,0.06,0.08,0.10,0.12,0.14,0.16,0.18,0.20,0.22,0.24,0.26,0.28,0.30,0.32,0.34,0.36,0.38,0.40,0.42,0.44,0.46,0.48,0.50,0.52,0.54,0.56,0.58,0.60,0.62,0.64,0.66,0.68,0.70,0.72,0.74,0.76,0.78,0.80,0.82,0.84,0.86,0.88,0.90,0.92,0.94,0.96,0.98,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,0.98,0.96,0.94,0.92,0.90,0.88,0.86,0.84,0.82,0.80,0.78,0.76,0.74,0.72,0.70,0.68,0.66,0.64,0.62,0.60,0.58,0.56,0.54,0.52,0.50,0.48,0.46,0.44,0.42,0.40,0.38,0.36,0.34,0.32,0.30,0.28,0.26,0.24,0.22,0.20,0.18,0.16,0.14,0.12,0.10,0.08,0.06,0.04,0.02,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};
        float temp_color_b[NUM_CELL_TEMPLATES]={0.52,0.54,0.56,0.58,0.60,0.62,0.64,0.66,0.68,0.70,0.72,0.74,0.76,0.78,0.80,0.82,0.84,0.86,0.88,0.90,0.92,0.94,0.96,0.98,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,0.98,0.96,0.94,0.92,0.90,0.88,0.86,0.84,0.82,0.80,0.78,0.76,0.74,0.72,0.70,0.68,0.66,0.64,0.62,0.60,0.58,0.56,0.54,0.52,0.50,0.48,0.46,0.44,0.42,0.40,0.38,0.36,0.34,0.32,0.30,0.28,0.26,0.24,0.22,0.20,0.18,0.16,0.14,0.12,0.10,0.08,0.06,0.04,0.02,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};
        for (int ix=0;ix<NUM_CELL_TEMPLATES;ix++) {
            color_r[ix]=temp_color_r[ix];
            color_g[ix]=temp_color_g[ix];
            color_b[ix]=temp_color_b[ix];
        }
    }
    else if(colormap.compare("hot")==0){
        float temp_color_r[NUM_CELL_TEMPLATES]={0.01,0.03,0.04,0.05,0.07,0.08,0.09,0.11,0.12,0.13,0.15,0.16,0.17,0.19,0.20,0.21,0.23,0.24,0.25,0.27,0.28,0.29,0.31,0.32,0.33,0.35,0.36,0.37,0.39,0.40,0.41,0.43,0.44,0.45,0.47,0.48,0.49,0.51,0.52,0.53,0.55,0.56,0.57,0.59,0.60,0.61,0.63,0.64,0.65,0.67,0.68,0.69,0.71,0.72,0.73,0.75,0.76,0.77,0.79,0.80,0.81,0.83,0.84,0.85,0.87,0.88,0.89,0.91,0.92,0.93,0.95,0.96,0.97,0.99,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00};
        float temp_color_g[NUM_CELL_TEMPLATES]={0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.01,0.03,0.04,0.05,0.07,0.08,0.09,0.11,0.12,0.13,0.15,0.16,0.17,0.19,0.20,0.21,0.23,0.24,0.25,0.27,0.28,0.29,0.31,0.32,0.33,0.35,0.36,0.37,0.39,0.40,0.41,0.43,0.44,0.45,0.47,0.48,0.49,0.51,0.52,0.53,0.55,0.56,0.57,0.59,0.60,0.61,0.63,0.64,0.65,0.67,0.68,0.69,0.71,0.72,0.73,0.75,0.76,0.77,0.79,0.80,0.81,0.83,0.84,0.85,0.87,0.88,0.89,0.91,0.92,0.93,0.95,0.96,0.97,0.99,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00};
        float temp_color_b[NUM_CELL_TEMPLATES]={0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.02,0.04,0.06,0.08,0.10,0.12,0.14,0.16,0.18,0.20,0.22,0.24,0.26,0.28,0.30,0.32,0.34,0.36,0.38,0.40,0.42,0.44,0.46,0.48,0.50,0.52,0.54,0.56,0.58,0.60,0.62,0.64,0.66,0.68,0.70,0.72,0.74,0.76,0.78,0.80,0.82,0.84,0.86,0.88,0.90,0.92,0.94,0.96,0.98,1.00};

        for (int ix=0;ix<NUM_CELL_TEMPLATES;ix++) {
            color_r[ix]=temp_color_r[ix];
            color_g[ix]=temp_color_g[ix];
            color_b[ix]=temp_color_b[ix];
        }
    }
    else if(colormap.compare("cool")==0){
        float temp_color_r[NUM_CELL_TEMPLATES]={0.00,0.01,0.01,0.02,0.02,0.03,0.03,0.04,0.04,0.05,0.05,0.06,0.06,0.07,0.07,0.08,0.08,0.09,0.09,0.10,0.10,0.11,0.11,0.12,0.12,0.13,0.13,0.14,0.14,0.15,0.15,0.16,0.16,0.17,0.17,0.18,0.18,0.19,0.19,0.20,0.20,0.21,0.21,0.22,0.22,0.23,0.23,0.24,0.24,0.25,0.25,0.26,0.26,0.27,0.27,0.28,0.28,0.29,0.29,0.30,0.30,0.31,0.31,0.32,0.32,0.33,0.33,0.34,0.34,0.35,0.35,0.36,0.36,0.37,0.37,0.38,0.38,0.39,0.39,0.40,0.40,0.41,0.41,0.42,0.42,0.43,0.43,0.44,0.44,0.45,0.45,0.46,0.46,0.47,0.47,0.48,0.48,0.49,0.49,0.50,0.50,0.51,0.51,0.52,0.52,0.53,0.53,0.54,0.54,0.55,0.55,0.56,0.56,0.57,0.57,0.58,0.58,0.59,0.59,0.60,0.60,0.61,0.61,0.62,0.62,0.63,0.63,0.64,0.64,0.65,0.65,0.66,0.66,0.67,0.67,0.68,0.68,0.69,0.69,0.70,0.70,0.71,0.71,0.72,0.72,0.73,0.73,0.74,0.74,0.75,0.75,0.76,0.76,0.77,0.77,0.78,0.78,0.79,0.79,0.80,0.80,0.81,0.81,0.82,0.82,0.83,0.83,0.84,0.84,0.85,0.85,0.86,0.86,0.87,0.87,0.88,0.88,0.89,0.89,0.90,0.90,0.91,0.91,0.92,0.92,0.93,0.93,0.94,0.94,0.95,0.95,0.96,0.96,0.97,0.97,0.98,0.98,0.99,0.99,1.00};
        float temp_color_g[NUM_CELL_TEMPLATES]={1.00,0.99,0.99,0.98,0.98,0.97,0.97,0.96,0.96,0.95,0.95,0.94,0.94,0.93,0.93,0.92,0.92,0.91,0.91,0.90,0.90,0.89,0.89,0.88,0.88,0.87,0.87,0.86,0.86,0.85,0.85,0.84,0.84,0.83,0.83,0.82,0.82,0.81,0.81,0.80,0.80,0.79,0.79,0.78,0.78,0.77,0.77,0.76,0.76,0.75,0.75,0.74,0.74,0.73,0.73,0.72,0.72,0.71,0.71,0.70,0.70,0.69,0.69,0.68,0.68,0.67,0.67,0.66,0.66,0.65,0.65,0.64,0.64,0.63,0.63,0.62,0.62,0.61,0.61,0.60,0.60,0.59,0.59,0.58,0.58,0.57,0.57,0.56,0.56,0.55,0.55,0.54,0.54,0.53,0.53,0.52,0.52,0.51,0.51,0.50,0.50,0.49,0.49,0.48,0.48,0.47,0.47,0.46,0.46,0.45,0.45,0.44,0.44,0.43,0.43,0.42,0.42,0.41,0.41,0.40,0.40,0.39,0.39,0.38,0.38,0.37,0.37,0.36,0.36,0.35,0.35,0.34,0.34,0.33,0.33,0.32,0.32,0.31,0.31,0.30,0.30,0.29,0.29,0.28,0.28,0.27,0.27,0.26,0.26,0.25,0.25,0.24,0.24,0.23,0.23,0.22,0.22,0.21,0.21,0.20,0.20,0.19,0.19,0.18,0.18,0.17,0.17,0.16,0.16,0.15,0.15,0.14,0.14,0.13,0.13,0.12,0.12,0.11,0.11,0.10,0.10,0.09,0.09,0.08,0.08,0.07,0.07,0.06,0.06,0.05,0.05,0.04,0.04,0.03,0.03,0.02,0.02,0.01,0.01,0.00};
        float temp_color_b[NUM_CELL_TEMPLATES]={1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00};

        for (int ix=0;ix<NUM_CELL_TEMPLATES;ix++) {
            color_r[ix]=temp_color_r[ix];
            color_g[ix]=temp_color_g[ix];
            color_b[ix]=temp_color_b[ix];
        }
    }
    else if(colormap.compare("spring")==0){
        float temp_color_r[NUM_CELL_TEMPLATES]={1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00};
        float temp_color_g[NUM_CELL_TEMPLATES]={0.00,0.01,0.01,0.02,0.02,0.03,0.03,0.04,0.04,0.05,0.05,0.06,0.06,0.07,0.07,0.08,0.08,0.09,0.09,0.10,0.10,0.11,0.11,0.12,0.12,0.13,0.13,0.14,0.14,0.15,0.15,0.16,0.16,0.17,0.17,0.18,0.18,0.19,0.19,0.20,0.20,0.21,0.21,0.22,0.22,0.23,0.23,0.24,0.24,0.25,0.25,0.26,0.26,0.27,0.27,0.28,0.28,0.29,0.29,0.30,0.30,0.31,0.31,0.32,0.32,0.33,0.33,0.34,0.34,0.35,0.35,0.36,0.36,0.37,0.37,0.38,0.38,0.39,0.39,0.40,0.40,0.41,0.41,0.42,0.42,0.43,0.43,0.44,0.44,0.45,0.45,0.46,0.46,0.47,0.47,0.48,0.48,0.49,0.49,0.50,0.50,0.51,0.51,0.52,0.52,0.53,0.53,0.54,0.54,0.55,0.55,0.56,0.56,0.57,0.57,0.58,0.58,0.59,0.59,0.60,0.60,0.61,0.61,0.62,0.62,0.63,0.63,0.64,0.64,0.65,0.65,0.66,0.66,0.67,0.67,0.68,0.68,0.69,0.69,0.70,0.70,0.71,0.71,0.72,0.72,0.73,0.73,0.74,0.74,0.75,0.75,0.76,0.76,0.77,0.77,0.78,0.78,0.79,0.79,0.80,0.80,0.81,0.81,0.82,0.82,0.83,0.83,0.84,0.84,0.85,0.85,0.86,0.86,0.87,0.87,0.88,0.88,0.89,0.89,0.90,0.90,0.91,0.91,0.92,0.92,0.93,0.93,0.94,0.94,0.95,0.95,0.96,0.96,0.97,0.97,0.98,0.98,0.99,0.99,1.00};
        float temp_color_b[NUM_CELL_TEMPLATES]={1.00,0.99,0.99,0.98,0.98,0.97,0.97,0.96,0.96,0.95,0.95,0.94,0.94,0.93,0.93,0.92,0.92,0.91,0.91,0.90,0.90,0.89,0.89,0.88,0.88,0.87,0.87,0.86,0.86,0.85,0.85,0.84,0.84,0.83,0.83,0.82,0.82,0.81,0.81,0.80,0.80,0.79,0.79,0.78,0.78,0.77,0.77,0.76,0.76,0.75,0.75,0.74,0.74,0.73,0.73,0.72,0.72,0.71,0.71,0.70,0.70,0.69,0.69,0.68,0.68,0.67,0.67,0.66,0.66,0.65,0.65,0.64,0.64,0.63,0.63,0.62,0.62,0.61,0.61,0.60,0.60,0.59,0.59,0.58,0.58,0.57,0.57,0.56,0.56,0.55,0.55,0.54,0.54,0.53,0.53,0.52,0.52,0.51,0.51,0.50,0.50,0.49,0.49,0.48,0.48,0.47,0.47,0.46,0.46,0.45,0.45,0.44,0.44,0.43,0.43,0.42,0.42,0.41,0.41,0.40,0.40,0.39,0.39,0.38,0.38,0.37,0.37,0.36,0.36,0.35,0.35,0.34,0.34,0.33,0.33,0.32,0.32,0.31,0.31,0.30,0.30,0.29,0.29,0.28,0.28,0.27,0.27,0.26,0.26,0.25,0.25,0.24,0.24,0.23,0.23,0.22,0.22,0.21,0.21,0.20,0.20,0.19,0.19,0.18,0.18,0.17,0.17,0.16,0.16,0.15,0.15,0.14,0.14,0.13,0.13,0.12,0.12,0.11,0.11,0.10,0.10,0.09,0.09,0.08,0.08,0.07,0.07,0.06,0.06,0.05,0.05,0.04,0.04,0.03,0.03,0.02,0.02,0.01,0.01,0.00};

        for (int ix=0;ix<NUM_CELL_TEMPLATES;ix++) {
            color_r[ix]=temp_color_r[ix];
            color_g[ix]=temp_color_g[ix];
            color_b[ix]=temp_color_b[ix];
        }
    }
    else if(colormap.compare("summer")==0){
        float temp_color_r[NUM_CELL_TEMPLATES]={0.00,0.01,0.01,0.02,0.02,0.03,0.03,0.04,0.04,0.05,0.05,0.06,0.06,0.07,0.07,0.08,0.08,0.09,0.09,0.10,0.10,0.11,0.11,0.12,0.12,0.13,0.13,0.14,0.14,0.15,0.15,0.16,0.16,0.17,0.17,0.18,0.18,0.19,0.19,0.20,0.20,0.21,0.21,0.22,0.22,0.23,0.23,0.24,0.24,0.25,0.25,0.26,0.26,0.27,0.27,0.28,0.28,0.29,0.29,0.30,0.30,0.31,0.31,0.32,0.32,0.33,0.33,0.34,0.34,0.35,0.35,0.36,0.36,0.37,0.37,0.38,0.38,0.39,0.39,0.40,0.40,0.41,0.41,0.42,0.42,0.43,0.43,0.44,0.44,0.45,0.45,0.46,0.46,0.47,0.47,0.48,0.48,0.49,0.49,0.50,0.50,0.51,0.51,0.52,0.52,0.53,0.53,0.54,0.54,0.55,0.55,0.56,0.56,0.57,0.57,0.58,0.58,0.59,0.59,0.60,0.60,0.61,0.61,0.62,0.62,0.63,0.63,0.64,0.64,0.65,0.65,0.66,0.66,0.67,0.67,0.68,0.68,0.69,0.69,0.70,0.70,0.71,0.71,0.72,0.72,0.73,0.73,0.74,0.74,0.75,0.75,0.76,0.76,0.77,0.77,0.78,0.78,0.79,0.79,0.80,0.80,0.81,0.81,0.82,0.82,0.83,0.83,0.84,0.84,0.85,0.85,0.86,0.86,0.87,0.87,0.88,0.88,0.89,0.89,0.90,0.90,0.91,0.91,0.92,0.92,0.93,0.93,0.94,0.94,0.95,0.95,0.96,0.96,0.97,0.97,0.98,0.98,0.99,0.99,1.00};
        float temp_color_g[NUM_CELL_TEMPLATES]={0.50,0.50,0.51,0.51,0.51,0.51,0.52,0.52,0.52,0.52,0.53,0.53,0.53,0.53,0.54,0.54,0.54,0.54,0.55,0.55,0.55,0.55,0.56,0.56,0.56,0.56,0.57,0.57,0.57,0.57,0.58,0.58,0.58,0.58,0.59,0.59,0.59,0.59,0.60,0.60,0.60,0.60,0.61,0.61,0.61,0.61,0.62,0.62,0.62,0.62,0.63,0.63,0.63,0.63,0.64,0.64,0.64,0.64,0.65,0.65,0.65,0.65,0.66,0.66,0.66,0.66,0.67,0.67,0.67,0.67,0.68,0.68,0.68,0.68,0.69,0.69,0.69,0.69,0.70,0.70,0.70,0.70,0.71,0.71,0.71,0.71,0.72,0.72,0.72,0.72,0.73,0.73,0.73,0.73,0.74,0.74,0.74,0.74,0.75,0.75,0.75,0.75,0.76,0.76,0.76,0.76,0.77,0.77,0.77,0.77,0.78,0.78,0.78,0.78,0.79,0.79,0.79,0.79,0.80,0.80,0.80,0.80,0.81,0.81,0.81,0.81,0.82,0.82,0.82,0.82,0.83,0.83,0.83,0.83,0.84,0.84,0.84,0.84,0.85,0.85,0.85,0.85,0.86,0.86,0.86,0.86,0.87,0.87,0.87,0.87,0.88,0.88,0.88,0.88,0.89,0.89,0.89,0.89,0.90,0.90,0.90,0.90,0.91,0.91,0.91,0.91,0.92,0.92,0.92,0.92,0.93,0.93,0.93,0.93,0.94,0.94,0.94,0.94,0.95,0.95,0.95,0.95,0.96,0.96,0.96,0.96,0.97,0.97,0.97,0.97,0.98,0.98,0.98,0.98,0.99,0.99,0.99,0.99,1.00,1.00};
        float temp_color_b[NUM_CELL_TEMPLATES]={0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40,0.40};

        for (int ix=0;ix<NUM_CELL_TEMPLATES;ix++) {
            color_r[ix]=temp_color_r[ix];
            color_g[ix]=temp_color_g[ix];
            color_b[ix]=temp_color_b[ix];
        }
    }
    else if(colormap.compare("autumn")==0){
        float temp_color_r[NUM_CELL_TEMPLATES]={1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00};
        float temp_color_g[NUM_CELL_TEMPLATES]={0.00,0.01,0.01,0.02,0.02,0.03,0.03,0.04,0.04,0.05,0.05,0.06,0.06,0.07,0.07,0.08,0.08,0.09,0.09,0.10,0.10,0.11,0.11,0.12,0.12,0.13,0.13,0.14,0.14,0.15,0.15,0.16,0.16,0.17,0.17,0.18,0.18,0.19,0.19,0.20,0.20,0.21,0.21,0.22,0.22,0.23,0.23,0.24,0.24,0.25,0.25,0.26,0.26,0.27,0.27,0.28,0.28,0.29,0.29,0.30,0.30,0.31,0.31,0.32,0.32,0.33,0.33,0.34,0.34,0.35,0.35,0.36,0.36,0.37,0.37,0.38,0.38,0.39,0.39,0.40,0.40,0.41,0.41,0.42,0.42,0.43,0.43,0.44,0.44,0.45,0.45,0.46,0.46,0.47,0.47,0.48,0.48,0.49,0.49,0.50,0.50,0.51,0.51,0.52,0.52,0.53,0.53,0.54,0.54,0.55,0.55,0.56,0.56,0.57,0.57,0.58,0.58,0.59,0.59,0.60,0.60,0.61,0.61,0.62,0.62,0.63,0.63,0.64,0.64,0.65,0.65,0.66,0.66,0.67,0.67,0.68,0.68,0.69,0.69,0.70,0.70,0.71,0.71,0.72,0.72,0.73,0.73,0.74,0.74,0.75,0.75,0.76,0.76,0.77,0.77,0.78,0.78,0.79,0.79,0.80,0.80,0.81,0.81,0.82,0.82,0.83,0.83,0.84,0.84,0.85,0.85,0.86,0.86,0.87,0.87,0.88,0.88,0.89,0.89,0.90,0.90,0.91,0.91,0.92,0.92,0.93,0.93,0.94,0.94,0.95,0.95,0.96,0.96,0.97,0.97,0.98,0.98,0.99,0.99,1.00};
        float temp_color_b[NUM_CELL_TEMPLATES]={0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};

        for (int ix=0;ix<NUM_CELL_TEMPLATES;ix++) {
            color_r[ix]=temp_color_r[ix];
            color_g[ix]=temp_color_g[ix];
            color_b[ix]=temp_color_b[ix];
        }
    }
    else if(colormap.compare("winter")==0){
        float temp_color_r[NUM_CELL_TEMPLATES]={0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.00};
        float temp_color_g[NUM_CELL_TEMPLATES]={0.00,0.01,0.01,0.02,0.02,0.03,0.03,0.04,0.04,0.05,0.05,0.06,0.06,0.07,0.07,0.08,0.08,0.09,0.09,0.10,0.10,0.11,0.11,0.12,0.12,0.13,0.13,0.14,0.14,0.15,0.15,0.16,0.16,0.17,0.17,0.18,0.18,0.19,0.19,0.20,0.20,0.21,0.21,0.22,0.22,0.23,0.23,0.24,0.24,0.25,0.25,0.26,0.26,0.27,0.27,0.28,0.28,0.29,0.29,0.30,0.30,0.31,0.31,0.32,0.32,0.33,0.33,0.34,0.34,0.35,0.35,0.36,0.36,0.37,0.37,0.38,0.38,0.39,0.39,0.40,0.40,0.41,0.41,0.42,0.42,0.43,0.43,0.44,0.44,0.45,0.45,0.46,0.46,0.47,0.47,0.48,0.48,0.49,0.49,0.50,0.50,0.51,0.51,0.52,0.52,0.53,0.53,0.54,0.54,0.55,0.55,0.56,0.56,0.57,0.57,0.58,0.58,0.59,0.59,0.60,0.60,0.61,0.61,0.62,0.62,0.63,0.63,0.64,0.64,0.65,0.65,0.66,0.66,0.67,0.67,0.68,0.68,0.69,0.69,0.70,0.70,0.71,0.71,0.72,0.72,0.73,0.73,0.74,0.74,0.75,0.75,0.76,0.76,0.77,0.77,0.78,0.78,0.79,0.79,0.80,0.80,0.81,0.81,0.82,0.82,0.83,0.83,0.84,0.84,0.85,0.85,0.86,0.86,0.87,0.87,0.88,0.88,0.89,0.89,0.90,0.90,0.91,0.91,0.92,0.92,0.93,0.93,0.94,0.94,0.95,0.95,0.96,0.96,0.97,0.97,0.98,0.98,0.99,0.99,1.00};
        float temp_color_b[NUM_CELL_TEMPLATES]={1.00,1.00,0.99,0.99,0.99,0.99,0.98,0.98,0.98,0.98,0.97,0.97,0.97,0.97,0.96,0.96,0.96,0.96,0.95,0.95,0.95,0.95,0.94,0.94,0.94,0.94,0.93,0.93,0.93,0.93,0.92,0.92,0.92,0.92,0.91,0.91,0.91,0.91,0.90,0.90,0.90,0.90,0.89,0.89,0.89,0.89,0.88,0.88,0.88,0.88,0.87,0.87,0.87,0.87,0.86,0.86,0.86,0.86,0.85,0.85,0.85,0.85,0.84,0.84,0.84,0.84,0.83,0.83,0.83,0.83,0.82,0.82,0.82,0.82,0.81,0.81,0.81,0.81,0.80,0.80,0.80,0.80,0.79,0.79,0.79,0.79,0.78,0.78,0.78,0.78,0.77,0.77,0.77,0.77,0.76,0.76,0.76,0.76,0.75,0.75,0.75,0.75,0.74,0.74,0.74,0.74,0.73,0.73,0.73,0.73,0.72,0.72,0.72,0.72,0.71,0.71,0.71,0.71,0.70,0.70,0.70,0.70,0.69,0.69,0.69,0.69,0.68,0.68,0.68,0.68,0.67,0.67,0.67,0.67,0.66,0.66,0.66,0.66,0.65,0.65,0.65,0.65,0.64,0.64,0.64,0.64,0.63,0.63,0.63,0.63,0.62,0.62,0.62,0.62,0.61,0.61,0.61,0.61,0.60,0.60,0.60,0.60,0.59,0.59,0.59,0.59,0.58,0.58,0.58,0.58,0.57,0.57,0.57,0.57,0.56,0.56,0.56,0.56,0.55,0.55,0.55,0.55,0.54,0.54,0.54,0.54,0.53,0.53,0.53,0.53,0.52,0.52,0.52,0.52,0.51,0.51,0.51,0.51,0.50,0.50};

        for (int ix=0;ix<NUM_CELL_TEMPLATES;ix++) {
            color_r[ix]=temp_color_r[ix];
            color_g[ix]=temp_color_g[ix];
            color_b[ix]=temp_color_b[ix];
        }
    }
    else if(colormap.compare("bone")==0){
        float temp_color_r[NUM_CELL_TEMPLATES]={0.00,0.00,0.01,0.01,0.02,0.02,0.03,0.03,0.04,0.04,0.04,0.05,0.05,0.06,0.06,0.07,0.07,0.07,0.08,0.08,0.09,0.09,0.10,0.10,0.11,0.11,0.11,0.12,0.12,0.13,0.13,0.14,0.14,0.15,0.15,0.15,0.16,0.16,0.17,0.17,0.18,0.18,0.18,0.19,0.19,0.20,0.20,0.21,0.21,0.22,0.22,0.22,0.23,0.23,0.24,0.24,0.25,0.25,0.26,0.26,0.26,0.27,0.27,0.28,0.28,0.29,0.29,0.29,0.30,0.30,0.31,0.31,0.32,0.32,0.33,0.33,0.33,0.34,0.34,0.35,0.35,0.36,0.36,0.36,0.37,0.37,0.38,0.38,0.39,0.39,0.40,0.40,0.40,0.41,0.41,0.42,0.42,0.43,0.43,0.44,0.44,0.44,0.45,0.45,0.46,0.46,0.47,0.47,0.47,0.48,0.48,0.49,0.49,0.50,0.50,0.51,0.51,0.51,0.52,0.52,0.53,0.53,0.54,0.54,0.55,0.55,0.55,0.56,0.56,0.57,0.57,0.58,0.58,0.58,0.59,0.59,0.60,0.60,0.61,0.61,0.62,0.62,0.62,0.63,0.63,0.64,0.64,0.65,0.65,0.66,0.66,0.67,0.68,0.68,0.69,0.70,0.70,0.71,0.72,0.72,0.73,0.74,0.74,0.75,0.76,0.77,0.77,0.78,0.79,0.79,0.80,0.81,0.81,0.82,0.83,0.83,0.84,0.85,0.86,0.86,0.87,0.88,0.88,0.89,0.90,0.90,0.91,0.92,0.92,0.93,0.94,0.94,0.95,0.96,0.97,0.97,0.98,0.99,0.99,1.00};
        float temp_color_g[NUM_CELL_TEMPLATES]={0.00,0.00,0.01,0.01,0.02,0.02,0.03,0.03,0.04,0.04,0.04,0.05,0.05,0.06,0.06,0.07,0.07,0.07,0.08,0.08,0.09,0.09,0.10,0.10,0.11,0.11,0.11,0.12,0.12,0.13,0.13,0.14,0.14,0.15,0.15,0.15,0.16,0.16,0.17,0.17,0.18,0.18,0.18,0.19,0.19,0.20,0.20,0.21,0.21,0.22,0.22,0.22,0.23,0.23,0.24,0.24,0.25,0.25,0.26,0.26,0.26,0.27,0.27,0.28,0.28,0.29,0.29,0.29,0.30,0.30,0.31,0.31,0.32,0.32,0.33,0.33,0.34,0.34,0.35,0.36,0.36,0.37,0.37,0.38,0.39,0.39,0.40,0.40,0.41,0.42,0.42,0.43,0.43,0.44,0.45,0.45,0.46,0.46,0.47,0.48,0.48,0.49,0.50,0.50,0.51,0.51,0.52,0.53,0.53,0.54,0.54,0.55,0.56,0.56,0.57,0.57,0.58,0.59,0.59,0.60,0.60,0.61,0.62,0.62,0.63,0.63,0.64,0.65,0.65,0.66,0.66,0.67,0.68,0.68,0.69,0.70,0.70,0.71,0.71,0.72,0.73,0.73,0.74,0.74,0.75,0.76,0.76,0.77,0.77,0.78,0.78,0.79,0.79,0.80,0.80,0.81,0.81,0.82,0.82,0.82,0.83,0.83,0.84,0.84,0.85,0.85,0.85,0.86,0.86,0.87,0.87,0.88,0.88,0.89,0.89,0.89,0.90,0.90,0.91,0.91,0.92,0.92,0.93,0.93,0.93,0.94,0.94,0.95,0.95,0.96,0.96,0.96,0.97,0.97,0.98,0.98,0.99,0.99,1.00,1.00};
        float temp_color_b[NUM_CELL_TEMPLATES]={0.00,0.01,0.01,0.02,0.03,0.03,0.04,0.04,0.05,0.06,0.06,0.07,0.07,0.08,0.09,0.09,0.10,0.10,0.11,0.12,0.12,0.13,0.14,0.14,0.15,0.15,0.16,0.17,0.17,0.18,0.18,0.19,0.20,0.20,0.21,0.21,0.22,0.23,0.23,0.24,0.24,0.25,0.26,0.26,0.27,0.27,0.28,0.29,0.29,0.30,0.30,0.31,0.32,0.32,0.33,0.34,0.34,0.35,0.35,0.36,0.37,0.37,0.38,0.38,0.39,0.40,0.40,0.41,0.41,0.42,0.43,0.43,0.44,0.44,0.45,0.45,0.46,0.46,0.47,0.47,0.48,0.48,0.49,0.49,0.49,0.50,0.50,0.51,0.51,0.52,0.52,0.53,0.53,0.53,0.54,0.54,0.55,0.55,0.56,0.56,0.56,0.57,0.57,0.58,0.58,0.59,0.59,0.60,0.60,0.60,0.61,0.61,0.62,0.62,0.63,0.63,0.64,0.64,0.64,0.65,0.65,0.66,0.66,0.67,0.67,0.67,0.68,0.68,0.69,0.69,0.70,0.70,0.71,0.71,0.71,0.72,0.72,0.73,0.73,0.74,0.74,0.74,0.75,0.75,0.76,0.76,0.77,0.77,0.78,0.78,0.78,0.79,0.79,0.80,0.80,0.81,0.81,0.82,0.82,0.82,0.83,0.83,0.84,0.84,0.85,0.85,0.85,0.86,0.86,0.87,0.87,0.88,0.88,0.89,0.89,0.89,0.90,0.90,0.91,0.91,0.92,0.92,0.93,0.93,0.93,0.94,0.94,0.95,0.95,0.96,0.96,0.96,0.97,0.97,0.98,0.98,0.99,0.99,1.00,1.00};

        for (int ix=0;ix<NUM_CELL_TEMPLATES;ix++) {
            color_r[ix]=temp_color_r[ix];
            color_g[ix]=temp_color_g[ix];
            color_b[ix]=temp_color_b[ix];
        }
    }
    else if(colormap.compare("copper")==0){
        float temp_color_r[NUM_CELL_TEMPLATES]={0.00,0.01,0.01,0.02,0.03,0.03,0.04,0.04,0.05,0.06,0.06,0.07,0.08,0.08,0.09,0.09,0.10,0.11,0.11,0.12,0.13,0.13,0.14,0.14,0.15,0.16,0.16,0.17,0.18,0.18,0.19,0.19,0.20,0.21,0.21,0.22,0.23,0.23,0.24,0.24,0.25,0.26,0.26,0.27,0.28,0.28,0.29,0.30,0.30,0.31,0.31,0.32,0.33,0.33,0.34,0.35,0.35,0.36,0.36,0.37,0.38,0.38,0.39,0.40,0.40,0.41,0.41,0.42,0.43,0.43,0.44,0.45,0.45,0.46,0.46,0.47,0.48,0.48,0.49,0.50,0.50,0.51,0.52,0.52,0.53,0.53,0.54,0.55,0.55,0.56,0.57,0.57,0.58,0.58,0.59,0.60,0.60,0.61,0.62,0.62,0.63,0.63,0.64,0.65,0.65,0.66,0.67,0.67,0.68,0.68,0.69,0.70,0.70,0.71,0.72,0.72,0.73,0.73,0.74,0.75,0.75,0.76,0.77,0.77,0.78,0.79,0.79,0.80,0.80,0.81,0.82,0.82,0.83,0.84,0.84,0.85,0.85,0.86,0.87,0.87,0.88,0.89,0.89,0.90,0.90,0.91,0.92,0.92,0.93,0.94,0.94,0.95,0.95,0.96,0.97,0.97,0.98,0.99,0.99,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00,1.00};
        float temp_color_g[NUM_CELL_TEMPLATES]={0.00,0.00,0.01,0.01,0.02,0.02,0.02,0.03,0.03,0.04,0.04,0.04,0.05,0.05,0.05,0.06,0.06,0.07,0.07,0.07,0.08,0.08,0.09,0.09,0.09,0.10,0.10,0.11,0.11,0.11,0.12,0.12,0.13,0.13,0.13,0.14,0.14,0.15,0.15,0.15,0.16,0.16,0.16,0.17,0.17,0.18,0.18,0.18,0.19,0.19,0.20,0.20,0.20,0.21,0.21,0.22,0.22,0.22,0.23,0.23,0.24,0.24,0.24,0.25,0.25,0.26,0.26,0.26,0.27,0.27,0.27,0.28,0.28,0.29,0.29,0.29,0.30,0.30,0.31,0.31,0.31,0.32,0.32,0.33,0.33,0.33,0.34,0.34,0.35,0.35,0.35,0.36,0.36,0.37,0.37,0.37,0.38,0.38,0.38,0.39,0.39,0.40,0.40,0.40,0.41,0.41,0.42,0.42,0.42,0.43,0.43,0.44,0.44,0.44,0.45,0.45,0.46,0.46,0.46,0.47,0.47,0.48,0.48,0.48,0.49,0.49,0.49,0.50,0.50,0.51,0.51,0.51,0.52,0.52,0.53,0.53,0.53,0.54,0.54,0.55,0.55,0.55,0.56,0.56,0.57,0.57,0.57,0.58,0.58,0.58,0.59,0.59,0.60,0.60,0.60,0.61,0.61,0.62,0.62,0.62,0.63,0.63,0.64,0.64,0.64,0.65,0.65,0.66,0.66,0.66,0.67,0.67,0.68,0.68,0.68,0.69,0.69,0.69,0.70,0.70,0.71,0.71,0.71,0.72,0.72,0.73,0.73,0.73,0.74,0.74,0.75,0.75,0.75,0.76,0.76,0.77,0.77,0.77,0.78,0.78};
        float temp_color_b[NUM_CELL_TEMPLATES]={0.00,0.00,0.01,0.01,0.01,0.01,0.01,0.02,0.02,0.02,0.03,0.03,0.03,0.03,0.03,0.04,0.04,0.04,0.04,0.05,0.05,0.05,0.06,0.06,0.06,0.06,0.07,0.07,0.07,0.07,0.07,0.08,0.08,0.08,0.08,0.09,0.09,0.09,0.10,0.10,0.10,0.10,0.11,0.11,0.11,0.11,0.12,0.12,0.12,0.12,0.12,0.13,0.13,0.13,0.14,0.14,0.14,0.14,0.14,0.15,0.15,0.15,0.16,0.16,0.16,0.16,0.17,0.17,0.17,0.17,0.18,0.18,0.18,0.18,0.19,0.19,0.19,0.19,0.19,0.20,0.20,0.20,0.21,0.21,0.21,0.21,0.22,0.22,0.22,0.22,0.23,0.23,0.23,0.23,0.23,0.24,0.24,0.24,0.25,0.25,0.25,0.25,0.26,0.26,0.26,0.26,0.27,0.27,0.27,0.27,0.27,0.28,0.28,0.28,0.29,0.29,0.29,0.29,0.29,0.30,0.30,0.30,0.30,0.31,0.31,0.31,0.32,0.32,0.32,0.32,0.33,0.33,0.33,0.33,0.34,0.34,0.34,0.34,0.34,0.35,0.35,0.35,0.35,0.36,0.36,0.36,0.36,0.37,0.37,0.37,0.38,0.38,0.38,0.38,0.39,0.39,0.39,0.39,0.40,0.40,0.40,0.40,0.40,0.41,0.41,0.41,0.41,0.42,0.42,0.42,0.42,0.43,0.43,0.43,0.44,0.44,0.44,0.44,0.45,0.45,0.45,0.45,0.45,0.46,0.46,0.46,0.47,0.47,0.47,0.47,0.48,0.48,0.48,0.48,0.48,0.49,0.49,0.49,0.50,0.50};

        for (int ix=0;ix<NUM_CELL_TEMPLATES;ix++) {
            color_r[ix]=temp_color_r[ix];
            color_g[ix]=temp_color_g[ix];
            color_b[ix]=temp_color_b[ix];
        }
    }
    else if(colormap.compare("red")==0){
        float temp_color_r[NUM_CELL_TEMPLATES];
        float temp_color_g[NUM_CELL_TEMPLATES];
        float temp_color_b[NUM_CELL_TEMPLATES];

        for (int idx_c=0;idx_c<NUM_CELL_TEMPLATES;idx_c++){
            temp_color_r[idx_c]=(float)idx_c/(float)NUM_CELL_TEMPLATES;
            temp_color_g[idx_c]=0;
            temp_color_b[idx_c]=0;
        }
        for (int ix=0;ix<NUM_CELL_TEMPLATES;ix++) {
            color_r[ix]=temp_color_r[ix];
            color_g[ix]=temp_color_g[ix];
            color_b[ix]=temp_color_b[ix];
        }
    }
    else if(colormap.compare("green")==0){

        for (int idx_c=0;idx_c<NUM_CELL_TEMPLATES;idx_c++){
            color_r[idx_c]=0;
            color_g[idx_c]=(float)idx_c/(float)NUM_CELL_TEMPLATES;
            color_b[idx_c]=0;
        }
    }
    else{
        for (int idx_c=0;idx_c<NUM_CELL_TEMPLATES;idx_c++){
            color_r[idx_c]=0;
            color_g[idx_c]=0;
            color_b[idx_c]=(float)idx_c/(float)NUM_CELL_TEMPLATES;
        }
    }

    // For each possible sensor-value create a pointcloud (template)
    // The pointcloud will change the color of the points (colormap),
    // as well as the number of points.
    for (int t = 0; t < NUM_CELL_TEMPLATES; t++)
    {
        //Discretize the range (0,1) into 200 steps
        float conc = (float)t / (float)NUM_CELL_TEMPLATES;

        pcl::PointCloud<pcl::PointXYZRGB> curr_pc;
        curr_pc.width = round(conc * max_points_cell);
        //ROS_INFO("For %d, the number of cells is %d",t,curr_pc.width);
        curr_pc.height = 1;
        curr_pc.points.resize (curr_pc.width * curr_pc.height);

        //Generate points for this concentration value
        for (size_t j = 0; j < curr_pc.points.size(); j++)
        {
            //position of point
            curr_pc.points[j].x = - m_resolution / 2.0 +  m_resolution * rand() / (RAND_MAX + 1.0f);
            curr_pc.points[j].y = - m_resolution / 2.0 +  m_resolution * rand() / (RAND_MAX + 1.0f);
            //curr_pc.points[j].z = - m_resolution / 2.0 +  m_resolution * rand() / (RAND_MAX + 1.0f);
            curr_pc.points[j].z = 0.0;
            //color of point according to concentration value
            curr_pc.points[j].r = round(255*color_r[t]);//0*255;
            curr_pc.points[j].g = round(255*color_g[t]);//conc*255;//1;//conc*255;
            curr_pc.points[j].b = round(255*color_b[t]);//0*255;
        }
        template_cells[t] = curr_pc;
    }
}

void CGMRF_map:: get_as_GasGrid(gas_map_msgs::GasGrid &gas_grid)
{
    gas_grid.mean.clear();
    gas_grid.var.clear();
    for (unsigned int cell_idx=0; cell_idx<N; cell_idx++)
    {
        gas_grid.mean.push_back(m_map[cell_idx].mean);
        gas_grid.var.push_back(m_map[cell_idx].std);
    }
    gas_grid.m_x_min = m_x_min;
    gas_grid.m_x_max = m_x_max;
    gas_grid.m_y_min = m_y_min;
    gas_grid.m_y_max = m_y_max;
    gas_grid.m_resolution = m_resolution;
    gas_grid.m_size_x = m_size_x;
    gas_grid.m_size_y = m_size_x;
    gas_grid.N = N;
}


void CGMRF_map::get_as_pointClouds(sensor_msgs::PointCloud2 &meanPC, sensor_msgs::PointCloud2 &varPC)
{
    try
    {
        pcl::PointCloud<pcl::PointXYZRGB> m_cloud;
        pcl::PointCloud<pcl::PointXYZRGB> v_cloud;
        int mean_pcl_initialized = 0;
        int var_pcl_initialized = 0;

        //Generate Point-Cloud (for visualization)
        for (unsigned int cell_idx=0; cell_idx<N; cell_idx++)
        {
            //Get mean and var (already normalized [0,1])
            double mean = m_map[cell_idx].mean;
            double std = m_map[cell_idx].std;
            //std::cout <<"[PUBLISH] mean:"<< mean << " "<<std::endl;
            //Generate mean pointCloud
            if(mean > 0 && mean<=1)
            {
                //Load PointCloud template based on current concentration
                pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
                int idx_mean = floor((float)NUM_CELL_TEMPLATES * mean);
                idx_mean = std::min(idx_mean,199);
                temp_cloud = template_cells[idx_mean];

                //Set point-cloud location (cell center)
                double cell_center_x = ((cell_idx % m_size_x)+0.5)*m_resolution + m_x_min;
                double cell_center_y = (floor(cell_idx/m_size_x)+0.5)*m_resolution + m_y_min;
                for (size_t ic = 0; ic < temp_cloud.points.size(); ic++)
                {
                    temp_cloud.points[ic].x += cell_center_x;
                    temp_cloud.points[ic].y += cell_center_y;
                    temp_cloud.points[ic].z += 0;//-1.5;
                }

                if(mean_pcl_initialized==0)
                {
                    m_cloud = temp_cloud;
                    mean_pcl_initialized = 1;
                }
                else
                    m_cloud += temp_cloud;
            }
            else if (mean!=0)
                ROS_WARN("[GMRF Pointcloud] MEAN value out of bounds! %.2f", mean);


            //Generate var pointCloud
            if(std > 0)
            {
                //STD values are not normalized...Normalize the plot
                std = std::min(std,2.0);
                std = std/2.0;
                //Load PointCloud template based on current concentration
                pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
                int idx_std = floor((float)NUM_CELL_TEMPLATES * std);
                idx_std = std::min(idx_std,199);
                //ROS_INFO("STD: %.2f,%d", std,idx_std);
                temp_cloud = template_cells[idx_std];

                //Set point-cloud location (cell center)
                double cell_center_x = ((cell_idx % m_size_x)+0.5)*m_resolution + m_x_min;
                double cell_center_y = (floor(cell_idx/m_size_x)+0.5)*m_resolution + m_y_min;
                for (size_t ic = 0; ic < temp_cloud.points.size(); ic++)
                {
                    temp_cloud.points[ic].x += cell_center_x;
                    temp_cloud.points[ic].y += cell_center_y;
                    temp_cloud.points[ic].z += 0;//-1.5;
                }

                if(var_pcl_initialized==0)
                {
                    v_cloud = temp_cloud;
                    var_pcl_initialized = 1;
                }
                else
                    v_cloud += temp_cloud;
            }
        }

        //ROS_INFO("[GMRF] Maps-ToPCL2");
        //Publish the generated PointClouds
        pcl::PCLPointCloud2 temp_m, temp_v;
        pcl::toPCLPointCloud2(m_cloud, temp_m);
        pcl::toPCLPointCloud2(v_cloud, temp_v);
        //Transform to ROS pointcloud_msg
        pcl_conversions::moveFromPCL(temp_m, meanPC);
        pcl_conversions::moveFromPCL(temp_v, varPC);
        //ROS_INFO("[GMRF] Maps-End");
    }catch(std::exception e){
        ROS_ERROR("[GMRF Pointcloud] Exception publishing as PCL: %s ", e.what() );
    }
}

void CGMRF_map::store_as_CSV(std::string output_csv_file)
{
    try{
        std::string filename_mean = output_csv_file + " - mean.csv";
        std::ofstream file_mean(filename_mean.c_str());

        std::string filename_std = output_csv_file + " - std.csv";
        std::ofstream file_std(filename_std.c_str());


        //Generate Point-Cloud (for visualization)
        for (unsigned int cell_idx=0; cell_idx<N; cell_idx++)
        {
            //Get mean and var (already normalized [0,1])
            double mean = m_map[cell_idx].mean;
            double std = m_map[cell_idx].std;

            //Compute cell coordinates
            double cell_x = (cell_idx % m_size_x);
            double cell_y = floor(cell_idx/m_size_x);

            file_mean << mean;
            file_std << std;

            if (cell_x >= m_size_x-1)
            {
                file_mean << '\n';
                file_std << '\n';
            } else {
 		file_mean << ',';
                file_std << ',';
		//ROS_INFO("[GMRF] CSV: %d/%d\t%d/%d ", int(cell_x), int(m_size_x), int(cell_y), int(m_size_y) );
            }
        }

    }catch(std::exception e){
        ROS_ERROR("[GMRF] Exception: %s ", e.what() );
    }
}

