/// Includes
#include "local_planner/localplan.hpp"

int Localplan::ALGO_TYPE = ALGO_TYPE_LINVIS; // see defines in header

/*****************************************************************************
 * Functions
 ****************************************************************************/


/*****************************************************************************
 * reset
 * @brief   clear all data
 ****************************************************************************/
void Localplan::reset()
 {
    vis_vertices.clear();
    vis_dist_matrix.clear();
    temp_vertices.clear();
    obstacle_vertices.clear();
    aStarPath.clear(); 
 }

/*****************************************************************************
 * setGoalInfo
 * @brief   Update vertex data with new start/goal. Does NOT update adjacency matrix.
 ****************************************************************************/
void Localplan::setGoalInfo(Vector2f start, Vector2f finish)
{
    ROS_INFO("Localplan::setGoalInfo: Enter");
    if(vis_vertices.size() < 2)
    {
        vis_vertices.clear(); // meh ... will never actually have size < 2
        vis_vertices.push_back(start);
        vis_vertices.push_back(finish);
    }
    else
    {
        vis_vertices[0] = start;
        vis_vertices[1] = finish;
        clearTempVertices();
    }
    aStarPath.clear();
    // ROS_DEBUG("Localplan::setGoalInfo: %s",vertexListStr().c_str());

    updateGraphGoalAndPlan();
}

/*****************************************************************************
 * setObstacleInfo
 * @brief   Update vertex data with new obstacles. Does NOT update adjacency matrix.
 ****************************************************************************/
void Localplan::setObstacleInfo(const vector< vector<Vector2f> > &obstacles)
{
    ROS_INFO("Localplan::setObstacleInfo: Enter");

    // set up vertex data structure
    // clear all obstacle data ... dont touch start/goal data
    vis_vertices.resize(2); // replaces entire if statement below
    // if(vis_vertices.empty())
    // {
    //     // no data exists ... add dummy vertices for start/goal position
    //     vis_vertices.push_back(Vector2f());
    //     vis_vertices.push_back(Vector2f());
    // }
    // else
    // {
    //     vis_vertices.erase(vis_vertices.begin()+2, vis_vertices.end());
    // }
    clearTempVertices();
    aStarPath.clear();    

    // save obstacle data internally
    obstacle_list = obstacles;

    // extract vertex data from obstacles list
    obstacle_vertices.clear();
    for(unsigned int i = 0; i< obstacles.size(); ++i) // for each obstacle
    {
        obstacle_vertices.push_back(vector<unsigned int>());
        for(unsigned int j =0; j<obstacles[i].size(); ++j) // for each vertex
        {
            vis_vertices.push_back(obstacles[i][j]);
            obstacle_vertices[i].push_back(vis_vertices.size()-1);
        }
    }
    // ROS_DEBUG("Localplan::setObstacleInfo: %s",vertexListStr().c_str());
}

/*****************************************************************************
 * updateGraphObstacles
 * @brief   Update obstacle vertices and rebuild the obstacle graph
 *          Start and goal vertices are NOT connected (i.e. assume goal is not available)
 ****************************************************************************/
void Localplan::updateGraphObstacles()
{
    ROS_INFO("Localplan::updateGraphObstacles: Enter");

    // clear previous data
    vis_dist_matrix.clear();
    // add first 2 rows
    vis_dist_matrix.push_back( vector<double>(vis_vertices.size(), INFINITY) );
    vis_dist_matrix.push_back( vector<double>(vis_vertices.size(), INFINITY) );
    // add the rest of the rows
    for (unsigned int i=2; i<vis_vertices.size(); ++i)
    {
        vis_dist_matrix.push_back( vector<double>(vis_vertices.size(), INFINITY) );
    }
    // vis_dist_matrix = vector< vector<double> >(vis_vertices.size(), vector<double>(vis_vertices.size(), INFINITY) );
    // obstaclesConnected = vector<bool>(obstacle_vertices.size(), false);

    if(ALGO_TYPE == ALGO_TYPE_BUG)
    {
        ROS_INFO("Localplan::updateGraphObstacles: Adding obstacle edges");
        // add all the obstacles' edges
        for(unsigned int i =0; i<obstacle_vertices.size(); ++i) // for each obstacle
        {
            for(unsigned int j = 0; j<obstacle_vertices[i].size(); ++j) // for each vertex
            {
                unsigned int v1 = obstacle_vertices[i][j];
                unsigned int v2 = (j==obstacle_vertices[i].size()-1) ? obstacle_vertices[i][0] : obstacle_vertices[i][j+1];
                vis_dist_matrix[v1][v2] = (vis_vertices[v1]-vis_vertices[v2]).norm();
                vis_dist_matrix[v2][v1] = vis_dist_matrix[v1][v2];
            }
        }
    }
    else if(ALGO_TYPE == ALGO_TYPE_VIS)
    {
        ROS_INFO("Localplan::updateGraphObstacles: Building visibility graph");
        //*
        vector< vector<linestringboost> > temp;
        for (unsigned int i=2; i<vis_vertices.size(); ++i)
            for (unsigned int j=0; j<vis_vertices.size(); ++j)
            {
                // if( (i>=obstacle_vertices[0].front() && i<= obstacle_vertices[0].back())
                //     || (j>=obstacle_vertices[0].front() && j<= obstacle_vertices[0].back()) )
                // {
                //     // vertex is part of the border ... dont connect it
                //     // useful for testing/viz
                //     vis_dist_matrix[i].push_back(INFINITY);
                //     continue;
                // }

                if( (j < 2) ) //|| (i < 2) )
                {
                    // corresponds to start and goal vetices ... ignore for now
                    vis_dist_matrix[i][j] = INFINITY;
                }
                else if(j < i)
                {
                    // lower triangular part -- same as upper part
                    vis_dist_matrix[i][j] = vis_dist_matrix[j][i];
                }
                else if(i == j)
                {
                    // diagonal entries
                    vis_dist_matrix[i][j] = 0;
                }
                else if ( obstacle_collision(vis_vertices[i], vis_vertices[j], false, temp) )
                {
                    vis_dist_matrix[i][j] = INFINITY;
                }
                else
                {
                    vis_dist_matrix[i][j] =  (vis_vertices[i]-vis_vertices[j]).norm();
                }
            }
        //*/
    }
    else if(ALGO_TYPE == ALGO_TYPE_LINVIS)
    {
        ROS_INFO("Localplan::updateGraphObstacles: Doing nothing");
    }
    else
    {
        ROS_ERROR("Localplan::updateGraphObstacles: Invalid ALGO_TYPE(%u)", ALGO_TYPE);
    }

    // ROS_DEBUG( (MATRIX_STR<double>(vis_dist_matrix)).c_str() );
    // ROS_DEBUG(edgeListStr().c_str());

    ROS_INFO("Localplan::updateGraphObstacles: Exit");
}

/*****************************************************************************
 * updateGraphGoal
 * @brief   Connect up start/goal vertices to rest of graph
 *          Assume already have obstacle graph (i.e., cost matrix has been initialized) and goal info
 *          Run AStar search to get 
 ****************************************************************************/
void Localplan::updateGraphGoalAndPlan()
{	
    ROS_INFO("Localplan::updateGraphGoal: Enter");
	
    if(ALGO_TYPE == ALGO_TYPE_VIS)
    {
        vector< vector<linestringboost> > temp;
		
        if(!fixStartAndEnd()) return;
		
        if(ignoreObs)
        {
            ROS_DEBUG("Localplan::updateGraphGoal: ignoring obstacles");
            // just connect goal to start
            vis_dist_matrix[0][1] = (vis_vertices[0]-vis_vertices[1]).norm();
            vis_dist_matrix[1][0] = vis_dist_matrix[0][1];
        }
        else
            for (unsigned int i=0; i<2; ++i)
            {
                // For first 2 rows go through all the cols to connect up start/goal vertices to the rest
                // For the third row onwards, need to fill in only the first 2 columns
                // (this will really be just copying data from the first 2 rows)
                for (unsigned int j=i; j<vis_vertices.size(); ++j)
                {
                    // if( (i>=obstacle_vertices[0].front() && i<= obstacle_vertices[0].back())
                    //     || (j>=obstacle_vertices[0].front() && j<= obstacle_vertices[0].back()) )
                    // {
                    //     // vertex is part of the border ... dont connect it
                    //     // useful for testing/viz
                    //     vis_dist_matrix[i].push_back(INFINITY);
                    //     continue;
                    // }

                    if(i == j)
                    {
                        // diagonal entries
                        vis_dist_matrix[i][j] = 0;
                    }
                    else if ( obstacle_collision(vis_vertices[i], vis_vertices[j], false, temp) )
                    {
                        vis_dist_matrix[i][j] = INFINITY;
                    }
                    else
                    {
                        vis_dist_matrix[i][j] = (vis_vertices[i]-vis_vertices[j]).norm();
                    }

                    vis_dist_matrix[j][i] = vis_dist_matrix[i][j];
                }
            }

        asRunAStarSearch(0,1);
    }
    else if(ALGO_TYPE == ALGO_TYPE_BUG)
    {
        vector< unsigned int > iPoints; // basically just list of temp_vertices except sorted w.r.t. the start vertex

        // clear previous connections
        for(unsigned int i = 0; i < 2; ++i)
            for(unsigned int j = i; j< vis_dist_matrix[i].size(); ++j)
            {
                vis_dist_matrix[i][j] = INFINITY;
                vis_dist_matrix[j][i] = INFINITY;
            }

        if(!fixStartAndEnd()) return;

        if(!ignoreObs) createTempVertices(); // else (temp_vertices.size()==0)
        else ROS_DEBUG("Localplan::updateGraphGoal: ignoring obstacles");

        // make new connections
        if(temp_vertices.size() > 0)
        {
            // sort vertices
            ROS_DEBUG("Localplan::updateGraphGoal: sorting intersections");
            for( unsigned int i = 0; i<temp_vertices.size(); ++i)
            {
                unsigned int j;
                bool duplicate = false;
                for( j=0 ; j<iPoints.size(); ++j)
                {
                    double diff = (vis_vertices[iPoints[j]]-vis_vertices[0]).norm() - (vis_vertices[temp_vertices[i]] - vis_vertices[0]).norm();
                    if(diff >= 0-PERCENT_TOL) // if iPoints[j] >= vis_vertices[temp_vertices[i]]
                    {
                        // dont insert the same point ... should never happen
                        if( abs(diff) <= PERCENT_TOL ) duplicate = true;
                        break;
                    }
                }
                if(!duplicate) iPoints.insert(iPoints.begin()+j, temp_vertices[i]);
            }
            ROS_DEBUG("Localplan::updateGraphGoal: iPoints.size()=%lu ... %s", iPoints.size(), ((iPoints.size() & 1)==0)?"E":"O" );
            ROS_DEBUG("Localplan::updateGraphGoal: sorted vertices: %s", VECTOR_STR(iPoints).c_str());

            // make inter-obstacle connections
            ROS_DEBUG("Localplan::updateGraphGoal: making inter-obstacle connections");
            ROS_DEBUG("Localplan::updateGraphGoal: connect %u to %u", 0, iPoints[0]);
            vis_dist_matrix[0][iPoints[0]] = (vis_vertices[iPoints[0]] - vis_vertices[0]).norm();
            vis_dist_matrix[iPoints[0]][0] = vis_dist_matrix[0][iPoints[0]];
            for( unsigned int i = 1; i<iPoints.size()-2; i+=2)
            {
                ROS_DEBUG("Localplan::updateGraphGoal: connect %u to %u", iPoints[i], iPoints[i+1]);
                vis_dist_matrix[iPoints[i]][iPoints[i+1]] = (vis_vertices[iPoints[i]] - vis_vertices[iPoints[i+1]]).norm();
                vis_dist_matrix[iPoints[i+1]][iPoints[i]] = vis_dist_matrix[iPoints[i]][iPoints[i+1]];
            }
            ROS_DEBUG("Localplan::updateGraphGoal: connect %u to %u", 1, iPoints.back());
            vis_dist_matrix[1][iPoints.back()] = (vis_vertices[iPoints.back()] - vis_vertices[1]).norm();
            vis_dist_matrix[iPoints.back()][1] = vis_dist_matrix[1][iPoints.back()];
        }
        else
        {
            ROS_DEBUG("Localplan::updateGraphGoal: no intersections ... direct connection");
            //connect start to finish
            vis_dist_matrix[0][1] = (vis_vertices[1] - vis_vertices[0]).norm();
            vis_dist_matrix[1][0] = vis_dist_matrix[0][1];
        }

        asRunAStarSearch(0,1);
    }
    else if(ALGO_TYPE == ALGO_TYPE_LINVIS)
    {
        vector<bool> obstaclesIntersected(obstacle_vertices.size());
        bool haveIntersections = true; // basically just an OR of obstaclesIntersected
        vector< bool > obstaclesConnected(obstacle_vertices.size(), false); // which obstacles have been added to the visibility graph
        vector<unsigned int> obstaclesConnectedList;
        unsigned int iterations = 0; // just for info
        vector< vector<linestringboost> > temp;

        // clear previous connections
        for(unsigned int i = 0; i < 2; ++i)
            for(unsigned int j = i; j< vis_dist_matrix[i].size(); ++j)
            {
                vis_dist_matrix[i][j] = INFINITY;
                vis_dist_matrix[j][i] = INFINITY;
            }

        if(!fixStartAndEnd()) return;

        // Construct initial path
        vis_dist_matrix[0][1] = (vis_vertices[0]-vis_vertices[1]).norm();
        vis_dist_matrix[1][0] = vis_dist_matrix[0][1];

        // iteratively connect obstacles
        while(haveIntersections)
        {
            ROS_INFO("Localplan::updateGraphGoal: iteration(%u)", ++iterations);

            // run Astar
            asRunAStarSearch(0,1);

            if(ignoreObs) 
            {
                ROS_DEBUG("Localplan::updateGraphGoal: ignoring obstacles");
                break; // ignoring obstacles .. no need to iterate
            }

            // check for intersections
            haveIntersections = false;
            for(unsigned int i=0; i<obstacle_vertices.size(); ++i) // for each obstacle
            {
                obstaclesIntersected[i] = false;
                for(unsigned int j=0; j<aStarPath.size()-1; ++j) // for each edge in the path
                    if( findObstacleIntersections_boost(vis_vertices[aStarPath[j]], vis_vertices[aStarPath[j+1]], getObstaclePoints(i), (i==0)) )
                    {
                        obstaclesIntersected[i] = true;
                        haveIntersections = true;
                        break;
                    }
            }

            if(haveIntersections)
            {
                // Clear previous connections to start and goal
                for(unsigned int i = 0; i < 2; ++i)
                    for(unsigned int j = i; j< vis_dist_matrix[i].size(); ++j)
                    {
                        vis_dist_matrix[i][j] = INFINITY;
                        vis_dist_matrix[j][i] = INFINITY;
                    }

                // Modify Visibility graph
                for(unsigned int i=0; i<obstaclesIntersected.size(); ++i) // for each obstacle
                {
                    if(obstaclesIntersected[i]) // only add obstacles that intersect the path
                    {
                        ROS_INFO("Localplan::updateGraphGoal: Obstacle(%u) intersected", i);

                        // set as connected
                        obstaclesConnected[i] = true;
                        obstaclesConnectedList.push_back(i);

                        // add all edges of obstacle
                        for(unsigned int ii=0; ii<obstacle_vertices[i].size(); ++ii)
                        {
                            unsigned int v1 = obstacle_vertices[i][ii];
                            unsigned int v2 = (ii==obstacle_vertices[i].size()-1) ? obstacle_vertices[i][0] : obstacle_vertices[i][ii+1];
                            vis_dist_matrix[v1][v2] = (vis_vertices[v1]-vis_vertices[v2]).norm();
                            vis_dist_matrix[v2][v1] = vis_dist_matrix[v1][v2];
                        }

                        // connect to other obstacles
                        // NOTE: going to (obstaclesConnectedList.size()-1) cuz dont need to connect to self
                        //       ... did that when adding edges above
                        for(unsigned int j = 0; j < obstaclesConnectedList.size()-1; ++j) // for each connected obstacle
                        {
                            // connect to new obstacle
                            unsigned int idx = obstaclesConnectedList[j];
                            for (unsigned int ii=0; ii < obstacle_vertices[i].size(); ++ii) // for each vertex in obstacle 1
                                for (unsigned int jj=0; jj < obstacle_vertices[idx].size(); ++jj) // for each vertex in obstacle 2
                                {
                                    unsigned int vi = obstacle_vertices[i][ii];
                                    unsigned int vj = obstacle_vertices[idx][jj];

                                    if( obstacle_collision_sub(vis_vertices[vi], vis_vertices[vj], obstaclesConnectedList, false, temp) ) vis_dist_matrix[vi][vj] = INFINITY;
                                    else vis_dist_matrix[vi][vj] =  (vis_vertices[vi]-vis_vertices[vj]).norm();
                                    vis_dist_matrix[vj][vi] = vis_dist_matrix[vi][vj];
                                }

                            // remove previous edges that intersect new obstacle
                            vector<unsigned int> obsListDelta;
                            obsListDelta.push_back(i);
                            for(unsigned int k = j+1; k < obstaclesConnectedList.size()-1; ++k) // for all other obstacles
                            {
                                unsigned int idx2 = obstaclesConnectedList[k];
                                for (unsigned int jj=0; jj < obstacle_vertices[idx].size(); ++jj)
                                    for (unsigned int kk=0; kk < obstacle_vertices[idx2].size(); ++kk)
                                    {
                                        unsigned int vj = obstacle_vertices[idx][jj];
                                        unsigned int vk = obstacle_vertices[idx2][kk];

                                        // if an edge existed previously, test to see if it intersects the new obstacle
                                        if(vis_dist_matrix[vj][vk] < INFINITY)
                                            if( obstacle_collision_sub(vis_vertices[vj], vis_vertices[vk], obsListDelta, false, temp) )
                                            {
                                                vis_dist_matrix[vj][vk] = INFINITY;
                                                vis_dist_matrix[vk][vj] = vis_dist_matrix[vj][vk];
                                            }
                                    }
                            }
                        }
                    }
                }

                // Connect start and goal to the new visibility graph
                for(unsigned int i=0; i<obstaclesConnectedList.size(); ++i) // for each obstacle in the connected obstacles list
                {
                    unsigned int idx = obstaclesConnectedList[i]; 
                    for (unsigned int j=0; j<obstacle_vertices[idx].size(); ++j) // for each vertex in obstacle
                    {
                        unsigned int v = obstacle_vertices[idx][j];

                        if( !obstacle_collision_sub(vis_vertices[0], vis_vertices[v], obstaclesConnectedList, false, temp) ) {
                            vis_dist_matrix[0][v] =  (vis_vertices[0]-vis_vertices[v]).norm();
                            vis_dist_matrix[v][0] = vis_dist_matrix[0][v];
                        }

                        if( !obstacle_collision_sub(vis_vertices[1], vis_vertices[v], obstaclesConnectedList, false, temp) ) {
                            vis_dist_matrix[1][v] =  (vis_vertices[1]-vis_vertices[v]).norm();
                            vis_dist_matrix[v][1] = vis_dist_matrix[1][v];
                        }
                    }
                }
            }
        }
    }
    else
    {
        ROS_ERROR("Localplan::updateGraphGoal: Invalid ALGO_TYPE(%u)", ALGO_TYPE);
    }

    // ROS_DEBUG( (MATRIX_STR<double>(vis_dist_matrix)).c_str() );
    ROS_INFO("Localplan::updateGraphGoal: Exit");
}

/*****************************************************************************
 * fixStartAndEnd
 * @brief   Shift start and end vertices if necessary
 ****************************************************************************/
bool Localplan::fixStartAndEnd()
{
    point2dboost startPtBoost = vector2fToBoost(vis_vertices[0]);
    point2dboost endPtBoost = vector2fToBoost(vis_vertices[1]);
    unsigned int obsS=obstacle_vertices.size(), obsE=obstacle_vertices.size();

    // ROS_INFO("Localplan::fixStartAndEnd: Enter");

    // Check if start or end pt is inside an obstacle
    for(unsigned int i=0; i<obstacle_vertices.size(); ++i)
    {
        polygonboost obsBoost = convertPolyVector2fToBoost(getObstaclePoints(i));
        bool startInObs, endInObs;
        if(i==0) {
            // Check if start is in the border EXTERIOR (boundary is OK)
            startInObs = !boost::geometry::covered_by(startPtBoost, obsBoost);
            // Check if end is in the border EXTERIOR (boundary is OK)
            endInObs = !boost::geometry::covered_by(endPtBoost, obsBoost);
        } else {
            // Check if start is in the obstacle INTERIOR (boundary is OK)
            startInObs = boost::geometry::within(startPtBoost, obsBoost);
            // Check if end is in the obstacle INTERIOR (boundary is OK)
            endInObs = boost::geometry::within(endPtBoost, obsBoost);
        }
        if(startInObs) obsS = i;
        if(endInObs) obsE = i;
    }
    
	//if (startInObs && endInObs){ //both start and end points inside same obs
		//if ( obsS == obsE){
		//}
	//}
    if(obsS < obstacle_vertices.size())
    {
        // start point is inside an obstacle
        ROS_INFO("Localplan::fixStartAndEnd: Start inside obsatacle");
        //ROS_ERROR("Localplan::createTempVertices: IM LOST :'( ... Stuck inside an obstacle! HELP ME!!");
        //return false;
        vis_vertices[0] = closestBoundaryPoint(vis_vertices[0], getObstaclePoints(obsS)).second;
        startPtBoost = vector2fToBoost(vis_vertices[0]);
    }
    if(obsE < obstacle_vertices.size())
    {
        // end point is inside an obstacle ... move towards start point
        vector<linestringboost> segs;

        ROS_INFO("Localplan::fixStartAndEnd: End inside obsatacle");
        findObstacleIntersections_boost(vis_vertices[0], vis_vertices[1], getObstaclePoints(obsE), (obsE==0), &segs );
        // ROS_DEBUG("%s",linestringboostvector2Str(segs).c_str());
        // get intersection point closest to goal
        double minDist = INFINITY;
        point2dboost minPoint = point2dboost();
        for(unsigned int i = 0; i<segs.size(); ++i) // for each linestring
            for(unsigned int j = 0; j<segs[i].size(); ++j) // for each point
            {
                double d = boost::geometry::distance(segs[i][j], endPtBoost);
                if( (d > 1e-10) && (d < minDist) )
                {
                    minDist = d;
                    minPoint = segs[i][j];
                }
            }
        vis_vertices[1] = Vector2f(minPoint.x(), minPoint.y());
    }
    return true;
}

/*****************************************************************************
 * createTempVertices
 * @brief   Create temp vertices and move start and end points if necessary
 ****************************************************************************/
bool Localplan::createTempVertices()
{
    vector< vector<linestringboost> > segs;
    bool hasCollisions;

    // create temp vertices
    hasCollisions = obstacle_collision(vis_vertices[0], vis_vertices[1], true, segs);
    if( hasCollisions )
    {
        // Create temp vertices if needed and fix end location
        ROS_DEBUG("Localplan::createTempVertices: creating points and edge connections");
        for(unsigned int i = 0; i<segs.size(); ++i) // for each obstacle
        {
            if(segs[i].size() == 0) continue; // no intersection

            polygonboost obsBoost = convertPolyVector2fToBoost(getObstaclePoints(i));

            for(unsigned int j = 0; j<segs[i].size(); ++j) // for each intersection segment
            {
                ROS_DEBUG("Localplan::createTempVertices: Checking segment %u on obstacle %u", j, i);

                // create points and connect each point with the respective edge
                for(unsigned int k=0; k<2; ++k) // for each point on segment (only 2 points)
                {
                    unsigned int newV = addTempVertex( Vector2f(segs[i][j][k].x(), segs[i][j][k].y()) );

                    // find edge that point belongs to
                    // TODO: check for vertex as well? -- if is a vertex then just add one edge instead of 2
                    for(unsigned int l = 0; l<obstacle_vertices[i].size(); ++l) // for each obstacle vertex
                    {
                        unsigned int edgeP1 = obstacle_vertices[i][l];
                        unsigned int edgeP2 = obstacle_vertices[i][ (l==obstacle_vertices[i].size()-1) ? 0 : (l+1) ];
                        if(pointOnSegment(vis_vertices[newV], vis_vertices[edgeP1], vis_vertices[edgeP2]))
                        {
                            ROS_DEBUG("Localplan::createTempVertices: point on edge (%u,%u)", edgeP1, edgeP2);
                            // add new edges to graph
                            double distP1 = (vis_vertices[newV]-vis_vertices[edgeP1]).norm();
                            double distP2 = (vis_vertices[newV]-vis_vertices[edgeP2]).norm();
                            
                            bool vIsP1 = distP1 < 1e-8;
                            bool vIsP2 = distP2 < 1e-8;

                            // probably will never happen but the edge just might be small enough?
                            if( !vIsP1 || (vIsP1 && vIsP2) ) {
                                vis_dist_matrix[newV][edgeP1] = distP1;
                                vis_dist_matrix[edgeP1][newV] = vis_dist_matrix[newV][edgeP1];
                            }

                            if( !vIsP2 || (vIsP1 && vIsP2) ) {
                                vis_dist_matrix[newV][edgeP2] = distP2;
                                vis_dist_matrix[edgeP2][newV] = vis_dist_matrix[newV][edgeP2];
                            }
                        }
                    }
                }
            }
        }
    }
    return true;
}

/*****************************************************************************
 * getObstaclePoints
 * @brief   Get a list of points for the obstacle specified
 ****************************************************************************/
vector< Vector2f > Localplan::getObstaclePoints(unsigned int o)
{
    vector< Vector2f > points;
    
    if(o >= obstacle_vertices.size()) { // meh
        ROS_WARN("Localplan::getObstaclePoints: invlalid obstacle specified");
        return points;
    }

    // for(unsigned int i = 0; i< obstacle_vertices[o].size(); ++i)
    //     points.push_back(vis_vertices[obstacle_vertices[o][i]]);
    // return points;
    return obstacle_list[o];
}

/*****************************************************************************
 * addTempVertex
 * @brief   Add a temporary vertex to the graph
 *          Should only be called if matrix exists
 ****************************************************************************/
unsigned int Localplan::addTempVertex(Vector2f v)
{
    ROS_DEBUG("Localplan::addTempVertex: Enter");
    // add vertex to vertiex list
    vis_vertices.push_back(v);

    // add vertex to adjacency matrix
    // extend each existing row
    for(unsigned int i=0; i< vis_dist_matrix.size(); ++i)
        vis_dist_matrix[i].push_back(INFINITY);
    // add row
    vis_dist_matrix.push_back(vector<double>(vis_vertices.size(), INFINITY));

    // add vertex to list of temp vertices
    temp_vertices.push_back(vis_vertices.size()-1);

    ROS_DEBUG("Localplan::addTempVertex: Added vertex %u:(%f,%f)", temp_vertices.back(), v.x(), v.y());
    return temp_vertices.back();
}

/*****************************************************************************
 * clearTempVertices
 * @brief   Clear list of temporary vertices and any associated data
 ****************************************************************************/
void Localplan::clearTempVertices()
{
    ROS_INFO("Localplan::clearTempVertices: Enter");
    
    if(temp_vertices.empty()) return;

    for(unsigned int i = 1; i<temp_vertices.size(); ++i)
        if(temp_vertices[i-1]>= temp_vertices[i])
            ROS_ERROR("Localplan::clearTempVertices: Test Failed!");

    unsigned int startV = temp_vertices.front();
    unsigned int endV = temp_vertices.back();

    if(vis_vertices.size() > startV)
        vis_vertices.erase(vis_vertices.begin()+startV, vis_vertices.begin()+endV+1);
    if(vis_dist_matrix.size() > startV)
    {
        vis_dist_matrix.erase(vis_dist_matrix.begin()+startV, vis_dist_matrix.begin()+endV+1);
        for(unsigned int i=0;i<vis_dist_matrix.size(); ++i)
            vis_dist_matrix[i].erase(vis_dist_matrix[i].begin()+startV, vis_dist_matrix[i].begin()+endV+1);
    }

    temp_vertices.clear();
    ROS_INFO("Localplan::clearTempVertices: Exit");
}

/**********************************************************************
 * obstacle_collision
 * @brief   Obstacle Collision check for inter-vertex lines
 *********************************************************************/
bool Localplan::obstacle_collision(Vector2f p1, Vector2f p2, bool findAll, vector< vector<linestringboost> > &otpt)
{ 
    ROS_DEBUG("Localplan::obstacle_collision: Checking (%f,%f)-(%f,%f) for obstacle collision",p1.x(),p1.y(),p2.x(),p2.y());

    vector< Vector2f > obstacle;
    bool found = false;
    otpt.clear();
    for (unsigned int i=0; i < obstacle_vertices.size(); i++) { // for each obstacle
        obstacle = getObstaclePoints(i);
        otpt.push_back( vector<linestringboost>() );
        // check for collision with line segment
        if ( findObstacleIntersections_boost(p1, p2, obstacle, (i==0), &(otpt.back()) ) ) {
            ROS_DEBUG("Localplan::obstacle_collision: TRUE(%u)", i);
            found = true;
            if(!findAll) break;
        }
    }

    ROS_DEBUG("Localplan::obstacle_collision: exiting");
    return found;
}

/**********************************************************************
 * obstacle_collision_sub
 * @brief   Obstacle Collision check for inter-vertex lines
 *          Only checks for collisions with obstacles in obsList
 *********************************************************************/
bool Localplan::obstacle_collision_sub(Vector2f p1, Vector2f p2, const vector<unsigned int> &obsList, bool findAll, vector< vector<linestringboost> > &otpt)
{ 
    ROS_DEBUG("Localplan::obstacle_collision_sub: Checking (%f,%f)-(%f,%f) for obstacle collision",p1.x(),p1.y(),p2.x(),p2.y());

    vector< Vector2f > obstacle;
    bool found = false;
    otpt.clear();
    for (unsigned int i=0; i < obsList.size(); i++) { // for each obstacle
        unsigned int obsIdx = obsList[i];
        obstacle = getObstaclePoints(obsIdx);
        otpt.push_back( vector<linestringboost>() );
        // check for collision with line segment
        if ( findObstacleIntersections_boost(p1, p2, obstacle, (obsIdx==0), &(otpt.back()) ) ) {
            ROS_DEBUG("Localplan::obstacle_collision_sub: TRUE(%u)", obsIdx);
            found = true;
            if(!findAll) break;
        }
    }

    ROS_DEBUG("Localplan::obstacle_collision_sub: exiting");
    return found;
}

/**********************************************************************
 * asRunAStarSearch
 * @brief   Main function to run AStar search
 *********************************************************************/
void Localplan::asRunAStarSearch(int startVertex, int finishVertex)
{
    ROS_INFO("Localplan::asRunAStarSearch: Starting Search ...");
    ROS_DEBUG("Localplan::asRunAStarSearch: startVertex(%u)-finishVertex(%u)",startVertex,finishVertex);

    staStarData data(startVertex, finishVertex);
    
    // Initialize search
    data.nextNodes.push_back(staStarNode());
    data.nextNodes[0].nodeIdx = 0;
    data.nextNodes[0].sDist = 0;
    data.nextNodes[0].fDist = asHeuristicCost(startVertex,finishVertex);
    ROS_DEBUG("Localplan::asRunAStarSearch: Done initalization");

    // main A* loop
    while(!data.nextNodes.empty())
    {
        ROS_DEBUG("Localplan::asRunAStarSearch: vertices left = %lu",data.nextNodes.size());

        // find vertex with best fDist
        staStarNode curBest = asFindAndRemoveBestNode(data);
        if(curBest.nodeIdx == -1) break; // all remaining vertices are disconnected from start vertex

        curBest.visited = true;
        data.visitedNodes.push_back(curBest);
        ROS_DEBUG("Localplan::asRunAStarSearch: curBest=%u", curBest.nodeIdx);

        // reached finish vertex
        if(curBest.nodeIdx == finishVertex)
        {
            ROS_INFO("Localplan::asRunAStarSearch: FOUND PATH");
            aStarPath = asReconstructBestPath(data);
            return;
        }

        // update all unvisited neighbours of chosen vertex
        for(unsigned int i = 0; i < vis_dist_matrix[curBest.nodeIdx].size(); ++i)
        {
            if( (vis_dist_matrix[curBest.nodeIdx][i] < INFINITY) && (asNodeVisitedIdx(i,data) == data.visitedNodes.size()) )
            {
                ROS_DEBUG("Localplan::asRunAStarSearch: updating neighbour vertex=%u", i);
                unsigned int j = 0; 
                for( ; j<data.nextNodes.size(); ++j)
                    if( data.nextNodes[j].nodeIdx == ((int)i) ) break;
                if(j == data.nextNodes.size()) // node does not already exist
                {
                    data.nextNodes.push_back(staStarNode());
                    data.nextNodes.back().nodeIdx = i;
                }
                double tempdist = curBest.sDist + vis_dist_matrix[curBest.nodeIdx][i];
                if(tempdist < data.nextNodes[j].sDist)
                {
                    data.nextNodes[j].sDist = tempdist;
                    data.nextNodes[j].fDist = data.nextNodes[j].sDist + asHeuristicCost(i,finishVertex);
                    data.nextNodes[j].parent = curBest.nodeIdx;
                }
            }
        }
    }

    ROS_ERROR("Localplan::asRunAStarSearch: Error ... no path found");
}

/**********************************************************************
 * asHeuristicCost
 * @brief   Calculate the heuristic cost between 2 vertices
 * @input   indecies of vertices in vis_vertices
 *********************************************************************/
double Localplan::asHeuristicCost(int fromVertex, int toVertex)
{
    return (vis_vertices[fromVertex]-vis_vertices[toVertex]).norm();
}

/**********************************************************************
 * asNodeVisitedIdx
 * @brief   index in unvisitedNodes of node with given idx
 *********************************************************************/
unsigned int Localplan::asNodeVisitedIdx(int idx, const staStarData &data)
{
    for(unsigned int i = 0; i< data.visitedNodes.size(); ++i)
        if(data.visitedNodes[i].nodeIdx==idx) return i;
    return data.visitedNodes.size();
}

/**********************************************************************
 * asFindAndRemoveBestNode
 * @brief   combine asFindBestNode and asRemoveNode for efficiency
 * @Returns Index in aStarNodes
 *********************************************************************/
staStarNode Localplan::asFindAndRemoveBestNode(staStarData &d)
{
    ROS_DEBUG("Localplan::asFindAndRemoveBestNode: enter");
    if(d.nextNodes.empty()) return staStarNode();

    // find min node
    int minIdx = 0;
    ROS_DEBUG("Localplan::asFindAndRemoveBestNode: minIdx=%u",minIdx);
    for(unsigned int i = 1; i<d.nextNodes.size(); ++i)
    {
        if(d.nextNodes[i].fDist < d.nextNodes[minIdx].fDist)
        {
            minIdx = i;
            ROS_DEBUG("Localplan::asFindAndRemoveBestNode: minIdx=%u",minIdx);
        }
    }
    ROS_DEBUG("Localplan::asFindAndRemoveBestNode: done search for min");
    staStarNode minNode = d.nextNodes[minIdx];
    ROS_DEBUG("Localplan::asFindAndRemoveBestNode: minNode.nodeIdx=%u",minNode.nodeIdx);

    // remove from list
    d.nextNodes.erase(d.nextNodes.begin()+minIdx);

    ROS_DEBUG("Localplan::asFindAndRemoveBestNode: exit");
    return minNode;
}

/**********************************************************************
 * asReconstructBestPath
 * @brief   Reconstructs best AStar path
 * @Returns List of waypoints (vertices)
 *********************************************************************/
vector<int> Localplan::asReconstructBestPath(const staStarData &d)
{
    vector<int> waypoints;
    double pathCost = 0;
    
    ROS_INFO("Localplan::asReconstructBestPath: enter");

    // get all waypoints
    int curIdx = d.finishVertex;
    while(curIdx != -1)
    {
        waypoints.push_back(curIdx);
        unsigned int dequeIdx = asNodeVisitedIdx(curIdx, d);
        if(curIdx == d.finishVertex) pathCost = d.visitedNodes[dequeIdx].sDist;
        curIdx = d.visitedNodes[dequeIdx].parent;
    }
    ROS_INFO("Localplan::asReconstructBestPath: pathLength=%lu, pathCost=%f", waypoints.size(), pathCost);

    // reverse list
    unsigned int sz = waypoints.size();
    for (unsigned int i=0; i < sz/2; ++i)
    {
        int temp;
        temp = waypoints[sz-1-i];
        waypoints[sz-1-i]=waypoints[i];
        waypoints[i]=temp;
    }

    stringstream ss;
    for(unsigned int i = 0; i<waypoints.size(); ++i)
        ss << (i==0 ? "[" : "") << waypoints[i] << (i==waypoints.size()-1 ? "]" : ",");
    ROS_INFO("Localplan::asReconstructBestPath: %s", ss.str().c_str());

    ROS_INFO("Localplan::asReconstructBestPath: exit");
    return waypoints;
}

/**********************************************************************
 * pathExists
 * @brief   Returns TRUE if path has been constructed
 *********************************************************************/
bool Localplan::pathExists() const
{
    return !aStarPath.empty();
}

/**********************************************************************
 * getPath
 * @brief   Returns the path that has been constructed
 *********************************************************************/
vector< Vector2f > Localplan::getPath() const
{
	ROS_INFO("Localplan::getPath: Enter");
	vector<Vector2f> path(aStarPath.size());
	for(unsigned int i=0; i<aStarPath.size(); ++i)
		path[i] = vis_vertices[aStarPath[i]];
	
	ROS_INFO("A star Path length = %d", path.size());
	return path;
}

/**********************************************************************
 * checkPathCollision
 * @brief   Checks if the path intersects the given obstacles. First obstacle is environment boundary.
 *********************************************************************/
 bool Localplan::checkPathCollision(const vector< vector<Vector2f> > &obstacles)
 {
    ROS_INFO("Localpath::checkPathCollision: Enter");
    for(unsigned int i=0; i<obstacles.size(); ++i) {// for each obstacle
        for(unsigned int j=0; j<aStarPath.size()-1; ++j) // for each edge in the path
            if( findObstacleIntersections_boost(vis_vertices[aStarPath[j]], vis_vertices[aStarPath[j+1]], obstacles[i], (i==0)) ) {// check collision
                ROS_DEBUG("Localpath::checkPathCollision: Edge %u intersects Obstacle %u", j, i);
                return true;
            }
    }
    ROS_INFO("Localpath::checkPathCollision: No collision detected");
    return false;
 }

/*************************************************************************//**
 * getVisualMsg
 * @brief                       Returns the visual messages
 *
 * Message Includes:
 *  - Visibilty Graph
 *  - Start/Finish points
 *  - Chosen path
 *  - Text
 ****************************************************************************/
visualization_msgs::MarkerArray Localplan::getVisualMsg () {
    String frameId = "/global";

    // colours  
    std_msgs::ColorRGBA red, green, blue, white, black;
    red.r = 1;      red.g = 0;      red.b = 0;      red.a = 1;
    green.r = 0;    green.g = 1;    green.b = 0;    green.a = 1;
    blue.r = 0;     blue.g = 0;     blue.b = 1;     blue.a = 1;
    white.r = 1;    white.g = 1;    white.b = 1;    white.a = 1;
    black.r = 0;    black.g = 0;    black.b = 0;    black.a = 0;

    // >>>>> message headers
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker msg_line; // edges
    visualization_msgs::Marker msg_strip; // path
    visualization_msgs::Marker msg_dots; // vertices
    visualization_msgs::Marker msg_text; // text
    geometry_msgs::Point msg_point;
    stringstream text;
    int id = 0;

    // >>>>> general init
    msg_line.header.frame_id = frameId;
    msg_line.type = visualization_msgs::Marker::LINE_LIST;
    msg_line.action = visualization_msgs::Marker::ADD;
    msg_line.lifetime = ros::Duration(0);

    msg_dots.header.frame_id = frameId;
    msg_dots.type = visualization_msgs::Marker::SPHERE_LIST; // POINTS?
    msg_dots.action = visualization_msgs::Marker::ADD;
    msg_dots.lifetime = ros::Duration(0);
    msg_dots.pose.orientation.w = 1;

    msg_strip.header.frame_id = frameId;
    msg_strip.type = visualization_msgs::Marker::LINE_STRIP;
    msg_strip.action = visualization_msgs::Marker::ADD;
    msg_strip.lifetime = ros::Duration(0);

    msg_text.header.frame_id = frameId;
    msg_text.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    msg_text.action = visualization_msgs::Marker::ADD;
    msg_text.lifetime = ros::Duration(0);

    // >>>>> Construct Markers
    // edges - Lines
    msg_line.id = id;
    msg_line.ns = "Edges";
    msg_line.scale.x = 0.2;
    msg_line.color = green;
    for(unsigned int i = 0; i<vis_dist_matrix.size(); ++i) {
        for(unsigned int j = i+1; j<vis_dist_matrix[i].size(); ++j) { // only need to look at upper half of the matrix
            if( vis_dist_matrix[i][j] < INFINITY ) {
                // lines are drawn b/w every pair of points (0-1, 2-3, ...)
                msg_point.x = vis_vertices[i].x();
                msg_point.y = vis_vertices[i].y();
                msg_line.points.push_back(msg_point);
                msg_point.x = vis_vertices[j].x();
                msg_point.y = vis_vertices[j].y();
                msg_line.points.push_back(msg_point);
            }
        }
    }
    marker_array.markers.push_back(msg_line);
    ++id;

    // vertex list - Dots
    msg_dots.id = id;
    msg_dots.ns = "Obs Verticies";
    msg_dots.scale.x = 0.5;
    msg_dots.scale.y = 0.5;
    msg_dots.scale.z = 0.5;
    msg_dots.color = red;
    msg_dots.points.clear();
    
    for(unsigned int i=2; i < vis_vertices.size(); ++i) {
        msg_point.x = vis_vertices[i].x();
        msg_point.y = vis_vertices[i].y();
        msg_dots.points.push_back(msg_point);
    }
    marker_array.markers.push_back(msg_dots);
    ++id;

    msg_dots.id = id;
    msg_dots.ns = "Start";
    msg_dots.color = blue;
    msg_dots.points.clear();
    msg_point.x = vis_vertices[0].x();
    msg_point.y = vis_vertices[0].y();
    msg_dots.points.push_back(msg_point);
    marker_array.markers.push_back(msg_dots);
    ++id;

    msg_dots.id = id;
    msg_dots.ns = "Goal";
    msg_dots.color = white;
    msg_dots.points.clear();
    msg_point.x = vis_vertices[1].x();
    msg_point.y = vis_vertices[1].y();
    msg_dots.points.push_back(msg_point);
    marker_array.markers.push_back(msg_dots);
    ++id;

    // aStar Path - Strip
    msg_strip.id = id;
    msg_strip.ns = "A* Path";
    msg_strip.scale.x = 0.5;
    msg_strip.color = red;
    for(unsigned int i = 0; i < aStarPath.size(); ++i) {
        // lines are drawn b/w successive points (0-1, 1-2, ...)
        msg_point.x = vis_vertices[aStarPath[i]].x();
        msg_point.y = vis_vertices[aStarPath[i]].y();
        msg_point.z = 0.5;
        msg_strip.points.push_back(msg_point);
    }
    marker_array.markers.push_back(msg_strip);
    ++id;

    // Vertex Labels - Text
    msg_text.ns = "Vertex Labels";
    msg_text.scale.z = 1;
    msg_text.color = white;
    for(unsigned int i=0; i < vis_vertices.size(); i++) {
        msg_text.id = id;
        msg_text.lifetime = ros::Duration(0);
        msg_text.pose.position.x = vis_vertices[i].x();
        msg_text.pose.position.y = vis_vertices[i].y();
        msg_text.pose.position.z = 1;

        text << "V" << i;
        text << " \n";
        msg_text.text = text.str();
        marker_array.markers.push_back(msg_text);
        text.str("");
        text.clear();
        id++;
    }

    return marker_array;
}

/**********************************************************************
 * edgeListStr
 * @brief   Get a list edges in the graph
 *********************************************************************/
String Localplan::edgeListStr() const
{
    stringstream ss;
    for(unsigned int i=0; i< vis_vertices.size(); ++i)
        for(unsigned int j = i+1; j<vis_vertices.size(); ++j) // symmetric graph
            if(vis_dist_matrix[i][j] < INFINITY)
                ss << "(" << i << "," << j << "),";
    
    return ss.str();
}

/**********************************************************************
 * vertexListStr
 * @brief   Get a list edges in the graph
 *********************************************************************/
String Localplan::vertexListStr() const
{
    stringstream ss;
    for(unsigned int i=0; i< vis_vertices.size(); ++i)
        ss << "(" << i << ":" << vis_vertices[i].x() << "," << vis_vertices[i].y() << "),";
    
    return ss.str();
}
