        
/*************************************************************************//**
 *****************************************************************************
 * @file        testMap.cpp
 * @brief       Generate a occupancy grid used for testing. Published as map
 * @author      Frank Imeson
 * @date        2012-01-02
 *****************************************************************************
 ****************************************************************************/

//#define DEBUG
#include "local_planner/map_tools.hpp"           
#include <string>

string msg_header_text = "/nasa";
const char* msg_header_str = msg_header_text.c_str();

/***********************************************************************************************
 * GridMap Class Methods
 *
 *
 ***********************************************************************************************/




/*************************************************************************//**
 *
 ****************************************************************************/
GridMap::GridMap ()
    : width(0)
    , height(0)
{}


/*************************************************************************//**
 *
 ****************************************************************************/
GridMap::GridMap (nav_msgs::OccupancyGrid* _map)
{
	map = _map;
	if (GridMap::map->info.resolution <= 0)
		GridMap::map->info.resolution = 1;
	width = map->info.width;
	height = map->info.height;
	resolution = map->info.resolution;
}



/*************************************************************************//**
 *
 ****************************************************************************/
GridMap::~GridMap ()
{}







/*************************************************************************//**
 *
 ****************************************************************************/
int GridMap::getMap (Vector2i index) {
	return map->data[ index(0) + index(1)*map->info.width ];
}



/*************************************************************************//**
 *
 ****************************************************************************/
int GridMap::getMap (Vector2f pos) {
	return getMap( getIndex(pos) );
}





/*************************************************************************//**
 *
 ****************************************************************************/
Vector2f GridMap::getPos (Vector2i index) {
	double x = index(0)*map->info.resolution + map->info.origin.position.x;
	double y = index(1)*map->info.resolution + map->info.origin.position.y;

	return Vector2f (x,y);
}



/*************************************************************************//**
 *
 ****************************************************************************/
Vector2i GridMap::getIndex (Vector2f pos) {
	int i = (pos(0)-map->info.origin.position.x)/map->info.resolution;
	int j = (pos(1)-map->info.origin.position.y)/map->info.resolution;
	return Vector2i (i,j);
}



/*************************************************************************//**
 *
 ****************************************************************************/
bool GridMap::testMap (Vector2i index) {
	//index = index + Vector2i(NGRID/2, NGRID/2);
	//index = index * RESOLUTION;
	if (index(0) >= 0 && index(0) < width && index(1) >= 0 && index(1) < height)
		return map->data[ index(0) + index(1)*map->info.width ] > 0;	//linear indexing.
	else
		return false;
}



/*************************************************************************//**
 *
 ****************************************************************************/
bool GridMap::testMap (Vector2f pos) {
	Vector2i index = getIndex(pos);
	return testMap(index);
}


/*************************************************************************//**
 *
 ****************************************************************************/
bool GridMap::testMap (int region, Vector2i index) {
	if (index(0) >= 0 && index(0) < width && index(1) >= 0 && index(1) < height)
		return map->data[ index(0) + index(1)*map->info.width ] == region;
	else
		return false;
}



/*************************************************************************//**
 *
 ****************************************************************************/
bool GridMap::testMap (int region, Vector2f pos) {
	Vector2i index = getIndex(pos);
	return testMap(region,index);
}


/*************************************************************************//**
 *
 ****************************************************************************/
void GridMap::setMap (int region, Vector2i index) {
	map->data[index(0) + index(1)*width] = region;
}

/*************************************************************************//**
 *
 ****************************************************************************/
void GridMap::setMap (int region, Vector2f pos) {
	Vector2i index = getIndex(pos);
	setMap(region, index);
}



/*************************************************************************//**
 *
 ****************************************************************************/
vector<Vector2i> GridMap::getTestIndex01 (Vector2i index, bool left) {
	vector<Vector2i> test_index;
	test_index.push_back( Vector2i(0,1) );
	test_index.push_back( Vector2i(1,1) );
	test_index.push_back( Vector2i(1,0) );
	test_index.push_back( Vector2i(1,-1) );
	test_index.push_back( Vector2i(0,-1) );
	test_index.push_back( Vector2i(-1,-1) );
	test_index.push_back( Vector2i(-1,0) );
	test_index.push_back( Vector2i(-1,1) );

	for (unsigned int i=0; i < test_index.size(); i++) {
		test_index[i] = test_index[i] + index;
	}

	vector<Vector2i> left_index;
	if (!left) {
		for (int i=test_index.size()-1; i >= 0; i--) {
			left_index.push_back(test_index[i]);
		}
		return left_index;
	} else {
		return test_index;
	}

}


/*************************************************************************//**
 *
 ****************************************************************************/
vector<Vector2i> GridMap::getTestIndex02 (Vector2i index, bool left) {
	vector<Vector2i> test_index;
	test_index.push_back( Vector2i(-1,2) );
	test_index.push_back( Vector2i(0,2) );
	test_index.push_back( Vector2i(1,2) );
	test_index.push_back( Vector2i(2,2) );
	test_index.push_back( Vector2i(2,1) );
	test_index.push_back( Vector2i(2,0) );
	test_index.push_back( Vector2i(2,-1) );
	test_index.push_back( Vector2i(2,-2) );
	test_index.push_back( Vector2i(1,-2) );
	test_index.push_back( Vector2i(0,-2) );
	test_index.push_back( Vector2i(-1,-2) );
	test_index.push_back( Vector2i(-2,-2) );
	test_index.push_back( Vector2i(-2,-1) );
	test_index.push_back( Vector2i(-2,0) );
	test_index.push_back( Vector2i(-2,1) );
	test_index.push_back( Vector2i(-2,2) );

	for (unsigned int i=0; i < test_index.size(); i++) {
		test_index[i] = test_index[i] + index;
	}

	vector<Vector2i> left_index;
	if (!left) {
		for (int i=test_index.size()-1; i >= 0; i--) {
			left_index.push_back(test_index[i]);
		}
		return left_index;
	} else {
		return test_index;
	}

}


/*************************************************************************//**
 *
 ****************************************************************************/
IndexAns GridMap::gotoNextNeighbour (Vector2i index, bool left) {
	IndexAns result;
	bool found_zero = false;

	// Index01 inner circle
	if ( testMap(index) ) {
		vector<Vector2i> test_index = getTestIndex01(index, left);
		for(unsigned int i=0; i<=test_index.size()*2; i++) {
			if ( testMap(0, test_index[i%test_index.size()]) ) {
				found_zero = true;
			}
			if ( found_zero && testMap(test_index[i%test_index.size()]) ) {
				result.ans = true;
				result.index = test_index[i%test_index.size()];
				break;
			}
		}
	}

	// Index02 outer circle
	if ( testMap(index) && !found_zero ) {
		vector<Vector2i> test_index = getTestIndex02(index, left);
		for(unsigned int i=0; i<=test_index.size()*2; i++) {
			if ( testMap(0, test_index[i%test_index.size()]) ) {
				found_zero = true;
			}
			if ( found_zero && testMap(test_index[i%test_index.size()]) ) {
				result.ans = true;
				result.index = test_index[i%test_index.size()];
				break;
			}
		}
	}

	return result;
}


/*************************************************************************//**
 * @brief						Find the map edge between object and free space
 *
 *								If the start vector is within an object then, traverse until free space
 ****************************************************************************/
ObstacleAns GridMap::collide (Vector2f start, Vector2f stop, COLLISIONS mode) {

	ObstacleAns result;
	result.collision = false;
	result.in = start;
	result.out = stop;
	int fog_counter(0);

		
	/**************************************
	 * Find collision (linear)
	 **************************************/
	Vector2f delta =  (stop-start)/(stop-start).norm() * map->info.resolution;
	//for (unsigned int i=0; i<(stop-start).norm()/delta.norm() ; i++) {		//coarser for longer vectors.
	for (unsigned int i=0; i<(stop-start).norm() ; i++) {						//FINE TOU
		Vector2f test01 = start + i*delta;	//working in grid/image space
		result.out = test01;


		/**************************************
		 * Collide with the fog (exit after penitration depth exceeded)
		 **************************************/
		if ( testMap(-1,test01) && !result.collision ) {
			fog_counter++;

			if ( fog_counter > 25 ) {
				#ifdef DEBUG
					ROS_DEBUG("Fog Found %.2f", i*map->info.resolution);
				#endif
				break;
			}
		}


		/**************************************
		 * Object found
		 **************************************/
		//if ( testMap(test01) && !result.collision && i > 10 ) {
		if ( testMap(test01) && !result.collision) {
			#ifdef DEBUG
				ROS_DEBUG("Collision Found %.2f", i*map->info.resolution);
			#endif
			result.in = test01;
			result.collision = true;
			if (mode == UP_TO) {
				break;
			}
		}

		/**************************************
		 * Exit found
		 **************************************/
		if ( result.collision && testMap(0,test01) ) {
			#ifdef DEBUG
				ROS_DEBUG("Exit Found %.2f", i*map->info.resolution);
			#endif
			result.out = test01;
			result.exit = true;
			break;
		}

	}


	/**************************************
	 * Trace permiter
	 **************************************/

	int right_fog_index(0);
	int left_fog_index(0);

	if (result.collision && !result.exit && (mode==AROUND) ) {
		result.right = result.in;
		result.left = result.in;


		// trace right
		bool done(false);
		IndexAns test02;
		test02.index = getIndex(result.right);
		Vector2i old_index = test02.index;
		int watchdog;
		watchdog = 0;
		while (!done) {
			test02 = gotoNextNeighbour(test02.index, false);
			if (test02.ans) {
				result.right = getPos(test02.index);
				right_fog_index++;
			} else {
				break;
			}

			if (test02.index == old_index) {
				result.discovered = true;
				break;
			}

			watchdog++;
			if (watchdog > 16000) {
				result.discovered = true;
				break;
			}
		}


		// trace left
		watchdog = 0;
		if (!result.discovered) {
			bool done(false);
			test02.index = getIndex(result.left);
			while (!done) {
				test02 = gotoNextNeighbour(test02.index, true);
				if (test02.ans) {
					result.left = getPos(test02.index);
					left_fog_index++;
				} else {
					break;
				}

				if (test02.index == old_index) {
					result.discovered = true;
					break;
				}

				watchdog++;
				if (watchdog > 16000) {
					result.discovered = true;
					break;
				}

			}
		}


		// choose the closest fog index
		if (!result.discovered) {
			if (left_fog_index > right_fog_index) {
				result.out = result.left;
			} else {
				result.out = result.right;
			}
		}

	}



	/**************************************
	 * Debug code
	 **************************************/

	#ifdef DEBUG
		if (result.collision) {
			ROS_DEBUG("Collision -> (%.2f,%.2f)", result.in.x(), result.in.y());
			if (result.exit) {
				ROS_DEBUG("Exit Found -> (%.2f,%.2f)", result.out.x(), result.out.y());
			}
			if (!result.discovered) {
				ROS_DEBUG("Not Discovered");
				ROS_DEBUG("  Left Fog (%.2f,%.2f)", result.left.x(), result.left.y());
				ROS_DEBUG("  Right Fog (%.2f,%.2f)", result.right.x(), result.right.y());
				ROS_DEBUG("  Out Fog (%.2f,%.2f)", result.out.x(), result.out.y());
			}
		} else {
			ROS_DEBUG("No collision: (%.2f,%.2f)", result.out.x(), result.out.y());
		}

	#endif

	return result;
}



/*************************************************************************//**
 * @brief						
 *
 *
 ****************************************************************************/
Mat GridMap::getImage () {
	Mat image(map->info.height, map->info.width, CV_8UC1, Scalar(0));

	for (int i = 0; i<image.cols; i++) {
		for (int j = 0; j<image.rows; j++) {
			if ( !testMap( Vector2i(i,j) ) ) {
				image.at<uchar>(image.rows-j-1,i) = 255;
			}
		}
	}
	#ifdef DEBUG
		cout << "Image: \n" << image << "\n";
	#endif
	return image;
}



/***********************************************************************************************
 * ImageMap Class Methods
 *
 *
 ***********************************************************************************************/	


/*************************************************************************//**
 * @brief						
 *
 *
 ****************************************************************************/
ImageMap::ImageMap ()
{
	initialized = false;
	resolution = 1;
	tolerance = 1;
	origin = Vector2f(0,0);
}


/*************************************************************************//**
 * @brief						
 *
 *
 ****************************************************************************/
ImageMap::ImageMap (Mat _image, Vector2f _origin, double _res)
	: image(_image)
	, tolerance(1)	
{
	// convert to greyscale (1 ch)
	image.convertTo(image, CV_8U, 1);
	// convert to binary image
	image = image > 128;
//	threshold(image, image, 128, 0, 1);

	// populations
	resolution = _res;
	origin = _origin;
	initialized = true;
}



/*************************************************************************//**
 * @brief						
 ****************************************************************************/
ImageMap::ImageMap (GridMap map)
	: initialized(true)
	, tolerance(1)
{
	image = map.getImage();
	resolution = map.resolution;
	origin = map.getPos( Vector2i(0,0) );
}



/*************************************************************************//**
 * @brief						
 ****************************************************************************/
ImageMap::ImageMap (const ImageMap& image_map)
{
	image_map.image.copyTo(image);
	initialized = image_map.initialized;
	origin = image_map.origin;
	resolution = image_map.resolution;
	tolerance = image_map.tolerance;
}




/*************************************************************************//**
 * @brief						
 *
 *
 ****************************************************************************/
void ImageMap::initilize (Mat _image, Vector2f _origin, double _res) {

	_image.copyTo(image);
	// convert to greyscale (1 ch)
	image.convertTo(image, CV_8U, 1);
	// convert to binary image
	image = image < 128;

	// populations
	tolerance = 1;
	resolution = _res;
	origin = _origin;
	initialized = true;
}



/*************************************************************************//**
 *
 ****************************************************************************/
Vector2f ImageMap::getPos (Vector2i index) {
	double x = index.x()*resolution + origin.x();
	double y = (image.rows - index.y()-1)*resolution + origin.y();
	return Vector2f(x,y);
}



/*************************************************************************//**
 *
 ****************************************************************************/
Point2i ImageMap::getIndex (Vector2f pos) {
	int x = (pos.x()-origin.x())/resolution;
	int y = image.rows - 1 - (pos.y()-origin.y())/resolution;
	return Point2i(x,y);
}



/*************************************************************************//**
 *
 ****************************************************************************/
int ImageMap::getMap (Point2i index) {
	return image.at<uchar>(index.x, index.y);
}



/*************************************************************************//**
 *
 ****************************************************************************/
int ImageMap::getMap (Vector2f pos) {
	return getMap( getIndex(pos) );
}



/*************************************************************************//**
 *
 ****************************************************************************/
void ImageMap::setMap (uchar value, Vector2f pos) {

	Point2i index = getIndex(pos);
	if (index.x >= 0 && index.y >= 0 && index.x < image.rows && index.y < image.cols) {
		image.at<uchar>(index.y,index.x) = value;
	} else {

	}

}





/*************************************************************************//**
 *
 ****************************************************************************/
nav_msgs::OccupancyGrid ImageMap::getMapMsg () {
	nav_msgs::OccupancyGrid map_msg;
	map_msg.header.seq = 1;
	map_msg.header.stamp = ros::Time::now();
	map_msg.header.frame_id = msg_header_str;
	map_msg.info.resolution = resolution;
	map_msg.info.width = image.cols;
	map_msg.info.height = image.rows;
	map_msg.info.origin.position.x = origin.x();
	map_msg.info.origin.position.y = origin.y();

	for (int i = image.rows-1; i>=0; i--) {
		uchar *s = image.ptr<uchar>(i);    // for 1 channel
		for (int j = 0; j<image.cols; j++) {
			if ( s[j] >= 1 ) {
				map_msg.data.push_back(0);
			} else {
				map_msg.data.push_back(100);
			}
		}
	}
	return map_msg;
}




/*************************************************************************//**
 *
 ****************************************************************************/
vector< vector<Vector2f> > ImageMap::findPolys () {

	vector< vector<Vector2f> > poly_list;
	Mat new_image;
	image.copyTo(new_image);
	erode(new_image, new_image, Mat(), Point2i(-1,-1), 1);
	dilate(new_image, new_image, Mat(), Point2i(-1,-1), 5);

	/**************************************
	 * Find edges
	 **************************************/
	Mat detected_edges;
	Canny(new_image, detected_edges, 0, 2, 3);
	#ifdef DEBUG
		cout << "detected_edges: \n" << detected_edges << "\n";
	#endif
	ROS_DEBUG("Image -> Edges");

	
	/**************************************
	 * Find contours
	 **************************************/
	vector<Vec4i> hierarchy;
	vector< vector<Point> > contours;
//	findContours( new_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	findContours( detected_edges, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
//	findContours( detected_edges, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );
	ROS_DEBUG("Countours found: %lu", contours.size());

	// approximate contours
	vector< vector<Point> > contour_polys( contours.size() );
	for (unsigned int i=0; i<contours.size(); i++) {
		approxPolyDP( Mat(contours[i]), contour_polys[i], tolerance/resolution, true );
		ROS_DEBUG("Poly of size(%lu) found: Tree(%d,%d,%d,%d)", contour_polys[i].size(), hierarchy[i][0], hierarchy[i][1], hierarchy[i][2], hierarchy[i][3]);
		#ifdef DEBUG
			for (unsigned int j=0; j<contour_polys[i].size(); j++) {
				cout << contour_polys[i][j];
			}
			cout << "\n";
		#endif
	}
	ROS_DEBUG("Vector List Created: %lu", contour_polys.size());

	if (contour_polys.size() > 0) {

		/**************************************
		 * Find boundry (max area)
		 **************************************/
		double max_area(0);
		int border_index(-1);
		for (unsigned int i=0; i<contour_polys.size(); i++) {
			if ( (contour_polys[i].size() > 2) && (hierarchy[i][0] < 0 || hierarchy[i][1] < 0) ) {
				double test_area = contourArea(contour_polys[i]);
				if ( test_area > max_area ) {
					border_index = i;
					max_area = test_area;
				}
			}
		}

		// default poly
		bool border_failed = false;
		if ( border_index >= 0 ) {
			if ( (max_area < 1000) || (contour_polys[border_index].size() < 4) ) {
				ROS_DEBUG("No Border Found!");
				poly_list.push_back( vector<Vector2f>() );
				poly_list.back().push_back( getPos( Vector2i(0,0) ) );
				poly_list.back().push_back( getPos( Vector2i(0,image.rows) ) );
				poly_list.back().push_back( getPos( Vector2i(image.cols,image.rows) ) );
				poly_list.back().push_back( getPos( Vector2i(image.cols,0) ) );
				ROS_DEBUG("Default poly used = Image Boarder");
				max_area = 99999999999999;	
				border_failed = true;
			} else {
				ROS_DEBUG("Border Poly of size(%lu) found", contour_polys[border_index].size());
				poly_list.push_back( vector<Vector2f>() );
				for (unsigned int j=0; j<contour_polys[border_index].size(); j++) {
					Vector2i pos_index(contour_polys[border_index][j].x, contour_polys[border_index][j].y);
					poly_list.back().push_back( getPos(pos_index) );
					ROS_DEBUG("Poly : (%.2f,%.2f)", poly_list.back()[j].x(), poly_list.back()[j].y());
				}
			}
		} else {
			ROS_DEBUG("No Border Found!");
			poly_list.push_back( vector<Vector2f>() );
			poly_list.back().push_back( getPos( Vector2i(0,0) ) );
			poly_list.back().push_back( getPos( Vector2i(0,image.rows) ) );
			poly_list.back().push_back( getPos( Vector2i(image.cols,image.rows) ) );
			poly_list.back().push_back( getPos( Vector2i(image.cols,0) ) );
			ROS_DEBUG("Default poly used = Image Boarder");			
			max_area = 99999999999999;	
			border_failed = true;
		}



		/**************************************
		 * Find holes (0 is normally the boundry, either way it is not a hole)
		 **************************************/
		vector< vector<Point> > holes;
		for (unsigned int i=1; i<contour_polys.size(); i++) {
			// check if contour is a poly and that it is in our boundry
			double test_area = contourArea(contour_polys[i]);
			if ( (contour_polys[i].size() >= 2) && (test_area < max_area*3/5) 
												&& ( (pointPolygonTest(contour_polys[border_index], contour_polys[i][0], false) > 0) || border_failed ) ) {

				// check avg (centroid) is not in previous poly
				int avg_x(0), avg_y(0);
				for (unsigned int j=0; j<contour_polys[i].size(); j++) {
					avg_x += contour_polys[i][j].x;
					avg_y += contour_polys[i][j].y;
				}
				avg_x = avg_x / contour_polys[i].size();
				avg_y = avg_y / contour_polys[i].size();
				Point poly_avg(avg_x,avg_y);

				// check that avg is not in a previous poly
				bool in_prev(false);
				for (unsigned int j=0; j<holes.size(); j++) {
					if ( pointPolygonTest(holes[j], poly_avg, false) >= 0 ) {
						in_prev = true;
					}
				}
				if (!in_prev) {
					ROS_DEBUG("Poly of size(%lu) found, area = %.2f", contour_polys[i].size(), test_area );
					

					if (contour_polys[i].size() == 2) {
						holes.push_back( vector<Point>() );

						Vector2f delta = getPos( Vector2i(contour_polys[i][1].x-contour_polys[i][0].x , contour_polys[i][1].y-contour_polys[i][0].y) );
						Rotation2D<float> rotation(M_PI/2);
						delta = rotation*delta;
						delta = delta/delta.norm() * 0.5 * tolerance;

						Point corner;
						corner = getIndex( getPos(Vector2i(contour_polys[i][0].x, contour_polys[i][0].y) ) - delta);
						holes.back().push_back(corner);
						corner = getIndex( getPos(Vector2i(contour_polys[i][0].x, contour_polys[i][0].y) ) + delta);
						holes.back().push_back(corner);
						corner = getIndex( getPos(Vector2i(contour_polys[i][1].x, contour_polys[i][1].y) ) + delta);
						holes.back().push_back(corner);
						corner = getIndex( getPos(Vector2i(contour_polys[i][1].x, contour_polys[i][1].y) ) - delta);
						holes.back().push_back(corner);
						ROS_DEBUG("Wall");
											
					} else {
						holes.push_back( contour_polys[i] );
					}
				}
			}
		}

		for (unsigned int i=0; i<holes.size(); i++) {
			poly_list.push_back( vector<Vector2f>() );
			ROS_DEBUG("Hole of size(%lu)", holes[i].size() );
			for (unsigned int j=0; j<holes[i].size(); j++) {
				Vector2i pos_index(holes[i][j].x, holes[i][j].y);
				poly_list.back().push_back( getPos(pos_index) );
				ROS_DEBUG("Poly : (%.2f,%.2f)", poly_list.back()[j].x(), poly_list.back()[j].y());
			}		
		}

	}


	return poly_list;
}



/*************************************************************************//**
 * @brief						
 ****************************************************************************/
double ImageMap::compare(ImageMap compare) {

    if ( image.rows != compare.image.rows || image.cols != compare.image.cols )
        return 1000000;

	// find difference in images
	Mat result_image;
	bitwise_xor(image, compare.image, result_image);

	// find edges
	Canny(result_image, result_image, 0, 2, 3);
	
	// find contours
	vector<Vec4i> hierarchy;
	vector< vector<Point> > contours;
	findContours( result_image, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0) );

	// find max area
	double max_area(0);
	for (unsigned int i=0; i<contours.size(); i++) {
		if ( (contours[i].size() > 2) && (hierarchy[i][0] < 0 || hierarchy[i][1] < 0) ) {
			double test_area = contourArea(contours[i]);
			if ( test_area > max_area ) {
				max_area = test_area;
			}
		}
	}
	max_area = max_area * resolution*resolution;

	// find max dimension
	double max_dim(0);
	for (unsigned int i=0; i<contours.size(); i++) {
		Vector2f lower(999999999,9999999), upper(-9999999,-9999999);
		for (unsigned int j=0; j<contours[i].size(); j++) {
			if ( contours[i][j].x < lower.x() ) {
				lower.x() = contours[i][j].x;
			}
			if ( contours[i][j].y < lower.y() ) {
				lower.y() = contours[i][j].y;
			}
			if ( contours[i][j].x > upper.x() ) {
				upper.x() = contours[i][j].x;
			}
			if ( contours[i][j].y > upper.y() ) {
				upper.y() = contours[i][j].y;
			}
		}

		Vector2f delta;
		delta = upper - lower;
		if (delta.norm() > max_dim) {
			max_dim = delta.norm();
		}

		delta = Vector2f(upper.x(), lower.y()) - Vector2f(lower.x(), upper.y());
		if (delta.norm() > max_dim) {
			max_dim = delta.norm();
		}

	}

	return max_area;
}



/*************************************************************************//**
 * @brief						
 ****************************************************************************/
void ImageMap::clear() {
	image = image > 100000;
}




/*************************************************************************//**
 * @brief						
 ****************************************************************************/
void ImageMap::mergeMap (GridMap grid) {
	for (unsigned int i = 0; i< grid.map->info.width; i++) {
		for (unsigned int j = 0; j< grid.map->info.height; j++) {
			if ( grid.getMap(Vector2i(i,j)) > 0 ) {
				Vector2f pos = grid.getPos( Vector2i(i,j) );
				setMap(0, pos);
			}
		}
	}
//	threshold(image, image, 0.5, 1, 0);
}




/*************************************************************************//**
 * @brief						
 ****************************************************************************/
void ImageMap::mergeMap (ImageMap _image_map) {
	for (int i = 0; i< _image_map.image.rows; i++) {
		for (int j = 0; j< _image_map.image.cols; j++) {
			if ( _image_map.image.at<uchar>(j,i) == 0 ) {
				Vector2f pos = _image_map.getPos( Vector2i(i,j) );
				setMap(0, pos);
//				Point2i index = getIndex(pos);
//				image.at<uchar>(index.y,index.x) = 1;
			}
		}
	}
	threshold(image, image, 0.5, 1, 0);
}



/*************************************************************************//**
 * @brief						
 ****************************************************************************/
void ImageMap::polyFill(vector<Vector2f> _poly) {
	vector<Point> poly;
	for (unsigned i=0; i<_poly.size(); i++) {
		Point2i vertex = getIndex(_poly[i]);
		poly.push_back(vertex);
	}
	fillConvexPoly(image, poly, Scalar(0), 8, 0);
}




/***********************************************************************************************
 * ConvexPolygon Class Methods
 *
 *
 ***********************************************************************************************/	


/*************************************************************************//**
 *
 ****************************************************************************/
ConvexPolygon::ConvexPolygon () {
}



/*************************************************************************//**
 *
 ****************************************************************************/
ConvexPolygon::ConvexPolygon (vector<Vector2f> point_list) {
	ConvexPolygon::point_list = point_list;
	findCentroid();
}


/*************************************************************************//**
 *
 ****************************************************************************/
double ConvexPolygon::getArea () {

    vector<Point> contour;
	for (unsigned int i=0; i<point_list.size(); i++)
        contour.push_back( Point(point_list[i].x(), point_list[i].y()) );

    return contourArea(contour);
}


/*************************************************************************//**
 *
 ****************************************************************************/
Vector2f ConvexPolygon::findCentroid () {
	centroid << 0,0;
	for (unsigned int i=0; i<point_list.size(); i++) {
		centroid += point_list[i];
	}
	
	if (point_list.size() > 0 )
		centroid = centroid / point_list.size();

	return centroid;
}



/*************************************************************************//**
 *
 ****************************************************************************/
Vector2f ConvexPolygon::getCentroid () {
	findCentroid();
	return centroid;
}






/*************************************************************************//**
 *
 ****************************************************************************/
Vector2f ConvexPolygon::collide (Vector2f start, Vector2f direction) {
if (point_list.size()>1) {

	VectorAns collision;
	Vector2f prev = point_list[0];

	for (int i=point_list.size()-1; i>=0; i--) {

		if ( !pointOnVector(start, prev, point_list[i]) ) {
			collision = intersectVector(start, start + direction, prev, point_list[i]);
		} else {
			#ifdef DEBUG
				printf("On Line: (%.2f,%.2f) -> (%.2f,%.2f) \n", prev(0), prev(1), point_list[i](0), point_list[i](1) );
			#endif
		}

		if (collision.ans) {
			#ifdef DEBUG
				printf("Collision with (%.2f,%.2f) -> (%.2f,%.2f) \n", prev(0), prev(1), point_list[i](0), point_list[i](1) );
			#endif
			break;
		}
		prev = point_list[i];
	}

	if (collision.ans)
		return collision.vector;
	else
		return start;
	
} else
	return start;
}



/*************************************************************************//**
 *
 ****************************************************************************/
bool ConvexPolygon::test (Vector2f position) {
	if (point_list.size()>1) {

		int count = 0;
		Vector2f prev, span;
		
		// if line from point intersects with "one" wall, then we are inside
		prev = point_list[0];
		span = Vector2f(urand()-0.5,urand()-0.5);
		for (int i=point_list.size()-1; i>=0; i--) {
			if (pointOnVector(position, prev, point_list[i]) )
				return true;

			if ( intersectVector(position, position + span, prev, point_list[i]).ans )
				count++;

			prev = point_list[i];
		}

		// Check twice 
		if ( count == 1 ) {
			prev = point_list[0];
			span = Vector2f(urand()-0.5,urand()-0.5);
			for (int i=point_list.size()-1; i>=0; i--) {
				if ( intersectVector(position, position + span, prev, point_list[i]).ans )
					count++;
				prev = point_list[i];
			}
			if ( count == 2 ) {
				return true;
			// check one last time
			} else {
				prev = point_list[0];
				span = Vector2f(urand()-0.5,urand()-0.5);
				for (int i=point_list.size()-1; i>=0; i--) {
					if ( intersectVector(position, position + span, prev, point_list[i]).ans )
						count++;
					prev = point_list[i];
				}
				if ( count == 2 )
					return true;
			}
		}
	}
	return false;
}



/***********************************************************************************************
 * VisualMsg Class Methods
 *
 *
 ***********************************************************************************************/
 

VisualMsg::VisualMsg ()
    : id(0)
{
	// colours	
	red.r = 1;		red.g = 0;		red.b = 0;		red.a = 1;
	green.r = 0; 	green.g = 1; 	green.b = 0; 	green.a = 1;
	blue.r = 0;		blue.g = 0;		blue.b = 1;		blue.a = 1;
	white.r = 1;	white.g = 1;	white.b = 1;	white.a = 1;
	black.r = 0;	black.g = 0;	black.b = 0;	black.a = 0;
}


void VisualMsg::addLine (const vector<Vector2f> poly, int colour, int size) {}

void VisualMsg::addPoly (const vector< vector<Vector2f> > &polys, const std_msgs::ColorRGBA &colour, const double size, const char* name_space) {
   	
    visualization_msgs::Marker msg_line;
   	msg_line.header.frame_id = msg_header_str;
	msg_line.type = visualization_msgs::Marker::LINE_LIST;
	msg_line.action = visualization_msgs::Marker::ADD;
	msg_line.lifetime = ros::Duration(0);
	msg_line.id = id;
	msg_line.ns = name_space;
	msg_line.scale.x = size;
	msg_line.color = colour;
	
   	geometry_msgs::Point msg_point;
	for (unsigned int i=0; i < polys.size(); i++) {
	    for (unsigned int j=0; j < polys[i].size()-1; j++) {
		    msg_point.x = polys[i][j].x();
		    msg_point.y = polys[i][j].y();
		    msg_line.points.push_back(msg_point);

		    msg_point.x = polys[i][j+1].x();
		    msg_point.y = polys[i][j+1].y();
		    msg_line.points.push_back(msg_point);
	    }

	    // connect back to front
	    msg_point.x = polys[i].back().x();
	    msg_point.y = polys[i].back().y();
	    msg_line.points.push_back(msg_point);

	    msg_point.x = polys[i].front().x();
	    msg_point.y = polys[i].front().y();
	    msg_line.points.push_back(msg_point);
    }

	msg.markers.push_back(msg_line);
	id++;
}

void VisualMsg::addText (const vector<Vector2f> poly, int colour, int size) {}
void VisualMsg::addDot (const vector<Vector2f> poly, int colour, int size) {}


/***********************************************************************************************
 * Common Maptool Methods
 *
 *
 ***********************************************************************************************/	
static bool IsOnSegment(double xi, double yi, double xj, double yj,
                        double xk, double yk) {
  return (xi <= xk || xj <= xk) && (xk <= xi || xk <= xj) &&
         (yi <= yk || yj <= yk) && (yk <= yi || yk <= yj);
}

static char ComputeDirection(double xi, double yi, double xj, double yj,
                             double xk, double yk) {
  double a = (xk - xi) * (yj - yi);
  double b = (xj - xi) * (yk - yi);
  return a < b ? -1 : a > b ? 1 : 0;
}
/** Do line segments (x1, y1)--(x2, y2) and (x3, y3)--(x4, y4) intersect? */
VectorAns DoLineSegmentsIntersect(Vector2f start01, Vector2f finish01, Vector2f start02, Vector2f finish02)
{
  VectorAns result;
  double x1, y1, x2, y2,  x3,  y3,  x4,  y4;
  
  x1 = start01.x();
  y1 = start01.y();
  x2 = finish01.x();
  y2 = finish01.y();
  x3 = start02.x();
  y3 = start02.y();
  x4 = finish02.x();
  y4 = finish02.y();
  
  char d1 = ComputeDirection(x3, y3, x4, y4, x1, y1);
  char d2 = ComputeDirection(x3, y3, x4, y4, x2, y2);
  char d3 = ComputeDirection(x1, y1, x2, y2, x3, y3);
  char d4 = ComputeDirection(x1, y1, x2, y2, x4, y4);
  result.ans =  (((d1 > 0 && d2 < 0) || (d1 < 0 && d2 > 0)) &&
          ((d3 > 0 && d4 < 0) || (d3 < 0 && d4 > 0))) ||
         (d1 == 0 && IsOnSegment(x3, y3, x4, y4, x1, y1)) ||
         (d2 == 0 && IsOnSegment(x3, y3, x4, y4, x2, y2)) ||
         (d3 == 0 && IsOnSegment(x1, y1, x2, y2, x3, y3)) ||
         (d4 == 0 && IsOnSegment(x1, y1, x2, y2, x4, y4));
         
   return result;
}



/*************************************************************************//**
 *
 ****************************************************************************/
VectorAns vectorIntersect (Vector2f start01, Vector2f finish01, Vector2f start02, Vector2f finish02, bool include) {

	Matrix2f A;
	Vector2f b,x,delta01,delta02;
	double error;
	VectorAns result;

	delta01 = finish01 - start01;
	delta02 = finish02 - start02;

	A(0,0) = -delta01(0);
	A(1,0) = -delta01(1);
	A(0,1) =  delta02(0);
	A(1,1) =  delta02(1);

	b = start01 - start02;
	x = A.colPivHouseholderQr().solve(b);
	if ( b.norm() > 0 )
		error = (A*x - b).norm() / b.norm();
	else
		error = 0;


	// x(0) is a fraction of vector (a,b), x(1) of vector (c,d) likewise
	if ( include && x(0) >= 0-PERCENT_TOL && x(0) <= 1+PERCENT_TOL && x(1) >= 0-PERCENT_TOL && x(1) <= 1+PERCENT_TOL && std::abs(error) < PERCENT_TOL) {
		result.ans = true;
		result.vector = start01 + delta01*x(0);
	}

	// x(0) is a fraction of vector (a,b), x(1) of vector (c,d) likewise
	if ( !include && x(0) > PERCENT_TOL && x(0) < 1-PERCENT_TOL && x(1) > PERCENT_TOL && x(1) < 1-PERCENT_TOL && std::abs(error) < PERCENT_TOL) {
		result.ans = true;
		result.vector = start01 + delta01*x(0);
	}

	#ifdef DEBUG
		printf("---------------------intersectVector() Calculations--------------------- \n");
		printf("start01 (%2.2f,%2.2f), finish01 (%2.2f,%2.2f) \n", start01(0), start01(1), finish01(0), finish01(1) );
		printf("start02 (%2.2f,%2.2f), finish02 (%2.2f,%2.2f) \n", start02(0), start02(1), finish02(0), finish02(1) );
		printf("A = (%2.2f,%2.2f ; %2.2f,%2.2f) \n", A(0,0),A(0,1),A(1,0),A(1,1) );
		printf("b = (%2.2f,%2.2f) \n", b(0),b(1) );
		printf("x = (%f,%f) \n", x(0),x(1) );
		printf("error = %f, True or False = %d \n", error, result.ans);
	#endif
	return result;
}


/*************************************************************************//**
 * Unlimited length of start01, finish01 but no direction change aloud
 ****************************************************************************/
VectorAns intersectVector (Vector2f start01, Vector2f finish01, Vector2f start02, Vector2f finish02) {

	Matrix2f A;
	Vector2f b,x,delta01,delta02;
	double error;
	VectorAns result;

	delta01 = finish01 - start01;
	delta02 = finish02 - start02;

	A(0,0) = -delta01(0);
	A(1,0) = -delta01(1);
	A(0,1) =  delta02(0);
	A(1,1) =  delta02(1);

	b = start01 - start02;
	x = A.colPivHouseholderQr().solve(b);
	if ( b.norm() > 0 )
		error = (A*x - b).norm() / b.norm();
	else
		error = 0;


	// x(0) is a fraction of vector (a,b), x(1) of vector (c,d) likewise
	if ( x(0) > PERCENT_TOL && x(1) > -PERCENT_TOL && x(1) < 1+PERCENT_TOL && std::abs(error) < PERCENT_TOL) {
		result.ans = true;
		result.vector = start01 + delta01*x(0);
	}

	#ifdef DEBUG
		printf("---------------------vectorIntersect() Calculations--------------------- \n");
		printf("start01 (%2.2f,%2.2f), finish01 (%2.2f,%2.2f) \n", start01(0), start01(1), finish01(0), finish01(1) );
		printf("start02 (%2.2f,%2.2f), finish02 (%2.2f,%2.2f) \n", start02(0), start02(1), finish02(0), finish02(1) );
		printf("A = (%2.2f,%2.2f ; %2.2f,%2.2f) \n", A(0,0),A(0,1),A(1,0),A(1,1) );
		printf("b = (%2.2f,%2.2f) \n", b(0),b(1) );
		printf("x = (%f,%f) \n", x(0),x(1) );
		printf("error = %f, True or False = %d \n", error, result.ans);
	#endif
	return result;
}


/*************************************************************************//**
 *
 ****************************************************************************/
bool pointOnVector (Vector2f point, Vector2f start, Vector2f finish) {

	Matrix2f A;
	Vector2f b,x,delta;
	double error;
	bool result(false);

	delta = finish - start;

	A(0,0) = 1;
	A(1,0) = 1;
	A(0,1) =  delta(0);
	A(1,1) =  delta(1);

	b = point - start;
	x = A.colPivHouseholderQr().solve(b);

	if ( b.norm() > 0 )
		error = (A*x - b).norm() / b.norm();
	else
		error = 0;

	// x(0) is a fraction of vector (a,b), x(1) of vector (c,d) likewise
	if (std::abs(x(0)) < PERCENT_TOL && x(1) >= 0 && x(1) <= 1+PERCENT_TOL && std::abs(error) < PERCENT_TOL) {
		result = true;
	}

	#ifdef DEBUG
		printf("---------------------pointOnVector() Calculations--------------------- \n");
		printf("start (%2.2f,%2.2f), finish (%2.2f,%2.2f) \n", start(0), start(1), finish(0), finish(1) );
		printf("point (%2.2f,%2.2f) \n", point(0), point(1) );
		printf("A = (%2.2f,%2.2f ; %2.2f,%2.2f) \n", A(0,0),A(0,1),A(1,0),A(1,1) );
		printf("b = (%2.2f,%2.2f) \n", b(0),b(1) );
		printf("x = (%f,%f) \n", x(0),x(1) );
		printf("error = %f, True or False = %d \n", error, result);
	#endif
	return result;
}


/*************************************************************************//**
 *
 ****************************************************************************/
bool testVectors (Vector2f vector01, Vector2f vector02) {
	Vector2f delta = vector02 - vector01;
	return delta.norm() < 0.01;
}


/*************************************************************************//**
 *
 ****************************************************************************/
VectorAns polyLeft (vector<Vector2f> poly) {
	VectorAns result;

	if (poly.size() == 0)
		return result;

	// find average of points
	for (unsigned int i=0; i<poly.size(); i++) {
		result.vector += poly[i];
	}
	result.vector = result.vector/poly.size();

	// find direction of points
	Vector2f delta = poly[0] - result.vector;
	double old_theta = atan2 ( delta.y(), delta.x() );
	int left(0);
	for (unsigned int i=1; i<poly.size(); i++) {
		delta = poly[i] - result.vector;
		double new_theta = atan2 ( delta.y(), delta.x() );
		if (new_theta > old_theta)
			left++;
		if (new_theta < old_theta)
			left--;
		old_theta = new_theta;
	}
	if (left > 0)
		result.ans = true;
	
	return result;
}


/*************************************************************************//**
 *
 ****************************************************************************/
bool polyMatch (vector<Vector2f> poly01, vector<Vector2f> poly02) {
	Vector2f min(9999,9999), max(0,0);

	// find min_max
	for (unsigned int i=0; i<poly01.size(); i++) {
		if (poly01[i].x() < min.x())
			min.x() = poly01[i].x();
		if (poly01[i].y() < min.y())
			min.y() = poly01[i].y();
		if (poly01[i].x() > max.x())
			max.x() = poly01[i].x();
		if (poly01[i].y() > max.y())
			max.y() = poly01[i].y();
	}
	for (unsigned int i=0; i<poly02.size(); i++) {
		if (poly02[i].x() < min.x())
			min.x() = poly01[i].x();
		if (poly02[i].y() < min.y())
			min.y() = poly01[i].y();
		if (poly02[i].x() > max.x())
			max.x() = poly02[i].x();
		if (poly02[i].y() > max.y())
			max.y() = poly02[i].y();
	}

	
	// create an positive integer poly
	vector< vector<Point> > contour01;
	contour01.push_back( vector<Point>() );
	for (unsigned int i=0; i<poly01.size(); i++) {
		Vector2f point = poly01[i]-min;
		contour01[0].push_back( Point( point.x()+1, point.y()+1 ) );
	}
	vector< vector<Point> > contour02;
	contour02.push_back( vector<Point>() );
	for (unsigned int i=0; i<poly02.size(); i++) {
		Vector2f point = poly02[i]-min;
		contour02[0].push_back( Point( point.x()+1, point.y()+1 ) );
	}

	int xsize = (max-min).x()+2;
	int ysize = (max-min).y()+2;

	cout << "contour01 " << contour01[0] << "\n";
	cout << "xsize " << xsize << ", ysize " << ysize << "\n";

	Mat image01(ysize, xsize, CV_8UC1, Scalar(0) );
	Mat image02(ysize, xsize, CV_8UC1, Scalar(0) );
	vector<Vec4i> hierarchy; // << (-1,-1,-1,-1);
	Scalar colour = Scalar(255);

	drawContours(image01, contour01, 0, colour, 2, 8, hierarchy, 0, Point(0,0) );
	drawContours(image02, contour02, 0, colour, 2, 8, hierarchy, 0, Point(0,0) );

	double match = matchShapes(image01, image02, 2, 0);
	printf("Match %.2f \n", match);
	if ( match>0 ) {
		return true;
	} else {
		return false;
	}

}


/*************************************************************************//**
 *
 ****************************************************************************/
double calcPathCost(const vector<Vector2f> path) {

	double cost = 0;
	for (unsigned int i=0; i < path.size()-1; i++) {
		Vector2f delta = path[i+1] - path[i];
		
		cost += 1; // corner cost
		cost += delta.norm(); // distance cost
	}
	return cost;

}


/**********************************************************************
 * pointOnSegment
 * @brief   Checks if a point is on a segment (including the endpoints)
 *********************************************************************/
bool pointOnSegment(Vector2f point, Vector2f start, Vector2f finish) {
    // eqn of segment: start + l*(finish-start), l in [0,1]
    Vector2f d = point-start;
    Vector2f seg = finish-start;

    // if d is parallel to seg => d = l*seg
    // check if some multiple of d is equal to seg
    double l = d.norm()/seg.norm();
    if(l < 1+PERCENT_TOL)
    {
        // d is inside circle of center "start" and radius "l"
        // check if d = seg*l
        return ( (d - seg*l).norm() < PERCENT_TOL );
    }
    return false;
}

/**********************************************************************
 * closestPointOnSegment
 * @brief   finds the closest point on the segment
 *********************************************************************/
Vector2f closestPointOnSegment(Vector2f point, Vector2f start, Vector2f finish) {
    // eqn of segment: start + l*(finish-start), l in [0,1]
    Vector2f d = point-start;
    Vector2f seg = finish-start;

    // Find projection of d onto seg
    double projDist = seg.dot(d)/seg.norm();
    // check if projection is beyond or at start point
    if(projDist < 0+PERCENT_TOL)
    	return start;
    // check if projection is beyond or at end point
    if(projDist > seg.norm()-PERCENT_TOL)
    	return finish;
    // projection is on segment
    return ( start + projDist*( seg/seg.norm() ) );
}


/**********************************************************************
 * findObstacleIntersections_boost
 * @brief   Checks if a line intersects an obstacle interior (points on obstacle boundary are allowed).
 * (p1,p2):     vertices of the line
 * obstacle:  list of vertices for the obstacle (OPEN and CLOCKWISE polygon)
 * isEnvBndry:  should the line lie inside (TRUE) or outside (FALSE) the polygon
 * otpt (optional):  if specified, this gets filled with the line segments that lie inside the polygon (default:NULL)
 *********************************************************************/
bool findObstacleIntersections_boost(Vector2f p1, Vector2f p2, const vector< Vector2f > &obstacle, bool isEnvBndry, vector<linestringboost> *otpt)
{
    polygonboost poly;
    linestringboost line; // 'segment' does NOT work
    vector<linestringboost> output;

    // convert line to boost linestring
    line.push_back( point2dboost(p1.x(),p1.y()) );
    line.push_back( point2dboost(p2.x(),p2.y()) );

    // convert obstacle to boost polygon
    poly = convertPolyVector2fToBoost(obstacle);
    // for(unsigned int i = 0; i<obstacle.size(); ++i )
    //     poly.outer().push_back( point2dboost(obstacle[i].x(),obstacle[i].y()) );
    // poly.outer().push_back( point2dboost(obstacle[0].x(),obstacle[0].y()) );

    // ROS_INFO(polygonboost2Str(poly).c_str());

    // find intersections
    output.clear();
    boost::geometry::intersection(poly, line, output);
    ROS_DEBUG("Localplan::findObstacleIntersections_boost: output.size()=%lu", output.size());

    if(isEnvBndry)
    {
        bool result = true;

        // line has to be contained entirely within the polygon ... cannot split up
        if(output.size() == 1)
        {
            // there is a line inside the boundary ... check if it is the same line
            // second condition should never happen but just incase the line gets flipped for whatever reason
            linestringboost line2 = output[0];
            if ( ( (boost::geometry::distance(line[0],line2[0]) < PERCENT_TOL) && (boost::geometry::distance(line[1],line2[1]) < PERCENT_TOL) )
               ||( (boost::geometry::distance(line[0],line2[1]) < PERCENT_TOL) && (boost::geometry::distance(line[1],line2[0]) < PERCENT_TOL) ) )
                result = false;
        }

        // Invert the line so that output is the segments OUTSIDE the polygon
        if(output.size() == 0)
            output.push_back(line);
        else
        {
            vector<point2dboost> points;
            for(unsigned int i=0; i<output.size(); ++i)
            {
                // assume the points are already sorted w.r.t. distance from start vertex
                points.push_back(output[i][0]);
                points.push_back(output[i][1]);
            }
            output.clear();

            // NOTE: points will always have a positive even number of elements
            if(boost::geometry::distance(line[0],points[0]) > PERCENT_TOL) { // first point different from start vertex
                linestringboost line2;
                line2.push_back(line[0]); line2.push_back(points[0]);
                output.push_back(line2);
            }
            for(unsigned int i = 1; i<points.size()-2; i+=2)
            {
                linestringboost line2;
                line2.push_back(points[i]); line2.push_back(points[i+1]);
                output.push_back(line2);
            }
            if(boost::geometry::distance(line[1],points.back()) > PERCENT_TOL) { // last point different from finish vertex
                linestringboost line2;
                line2.push_back(points.back()); line2.push_back(line[1]);
                output.push_back(line2);
            }
        }

        if(otpt != NULL) (*otpt) = output;
        return result;
    }

    if(! (output.empty()) )
    {
        // Check if any of the intersections lie on an edge

        // linestringboost edge;
        // vector<point2dboost> out;
        // bool p1OnEdge, p2OnEdge;
        Vector2f start, finish;

        // for(unsigned int i=0; i<poly.outer().size()-1; ++i) // for each edge
        for(unsigned int i=0; i<obstacle.size(); ++i) // for each edge
        {
            // edge.push_back( poly.outer()[i] );
            // edge.push_back( poly.outer()[i+1] );
            start = obstacle[i];
            finish = obstacle[ (i==obstacle.size()-1)?0:(i+1) ];

            for(unsigned int j=0; j<output.size(); ++j) // for each intersecting line
            {
                // NOTE: technically this is a vector of points but dont actually need a loop since this particular type of intersection
                //        cannot return a line string ... so will only have 2 points

                // out.clear();
                // boost::geometry::intersection(output[j][0], edge, out);
                // p1OnEdge = (out.size()>0);
                // boost::geometry::intersection(output[j][1], edge, out);
                // p2OnEdge = (out.size()>0);

                // if(p1OnEdge && p2OnEdge)
                if( pointOnSegment(Vector2f(output[j][0].x(),output[j][0].y()), start, finish) && pointOnSegment(Vector2f(output[j][1].x(),output[j][1].y()), start, finish) )
                {
                    // ROS_INFO("bad intersect: [%u] of %s on (%f,%f)-(%f,%f)", j, linestringboostvector2Str(output).c_str(), start.x(), start.y(), finish.x(), finish.y() );
                    output.erase(output.begin()+j);
                    --j;
                }
            }
        }
    }
    if(otpt != NULL) (*otpt) = output;
    return !(output.empty());
}


vector<Vector2f> convertPolyBoostToVector2f (polygonboost boostVect)
{
	vector<Vector2f> vector2fVect;
	
	for (unsigned int i=0; i< boostVect.outer().size()-1; i++)
	{
		vector2fVect.push_back(Vector2f(boostVect.outer()[i].x(),boostVect.outer()[i].y()));
		
	}
	
	return vector2fVect;
	
}


polygonboost convertPolyVector2fToBoost (const vector<Vector2f> & vector2fVect)
{
	
	polygonboost boostVect;
	
	for (unsigned int i=0; i< vector2fVect.size(); i++)
		boostVect.outer().push_back( point2dboost(vector2fVect[i].x(),vector2fVect[i].y()) );
	
	boostVect.outer().push_back( point2dboost(vector2fVect[0].x(),vector2fVect[0].y()) );
	boost::geometry::correct(boostVect); 		//magic. 
	return boostVect;
	
}

point2dboost vector2fToBoost(Vector2f p)
{
	return point2dboost(p.x(),p.y());
}

/*****************************************************************************
 * closestBoundaryPoint
 * @brief   Find EDGE and POINT ON EDGE of specified obstacle closest to point p
 ****************************************************************************/
std::pair<unsigned int, Vector2f> closestBoundaryPoint(Vector2f p, const vector<Vector2f> & obsPts)
{
    double dist;
	Vector2f edgeP1, edgeP2, temp;

    double minDist = INFINITY;
    std::pair<unsigned int, Vector2f> closest;
    closest.first = 0;
    for(unsigned int i=0; i< obsPts.size(); ++i)
    {
    	edgeP1 = obsPts[i];
    	edgeP2 = obsPts[ (i < obsPts.size()-1)? (i+1) : 0 ];

        temp = closestPointOnSegment(p, edgeP1, edgeP2);
        dist = (p-temp).norm();
        if( dist < minDist )
        {
            minDist = dist;
            closest.first = i;
            closest.second = temp;
        }
    }

    return closest;
}


vector<vector<Vector2f> > convertPolyWithHolesBoostToVector2f(polygonboost boostVect)
{
	vector<vector<Vector2f> > vector2fPolys;
	vector<Vector2f> boundary;

	// filling in the boundary
	for (unsigned int i=0; i< boostVect.outer().size()-1; i++)
	{
		boundary.push_back(Vector2f(boostVect.outer()[i].x(),boostVect.outer()[i].y()));
	}
	vector2fPolys.push_back(boundary);

	// filling in the holes
	for (unsigned int i=0; i< boostVect.inners().size(); i++)
	{
		vector<Vector2f> eachInner;

		for (unsigned int j=0; j< boostVect.inners()[i].size()-1; j++)
		{
			eachInner.push_back(Vector2f(boostVect.inners()[i][j].x(),boostVect.inners()[i][j].y()));
		}

		vector2fPolys.push_back(eachInner);

	}

	return vector2fPolys;

}


/*************************************************************************//**
 * FOR DEBUGGING
 ****************************************************************************/
string vector2Str(Vector2f v)
{
    stringstream ss;
    ss << "(" << v.x() << "," << v.y() << ")";
    return ss.str();
}

string linestringboostvector2Str(vector<linestringboost> &l)
{
    stringstream ss;
    for(unsigned int j=0; j< l.size(); ++j)
    {
        ss << "[";
        for(unsigned int i =0; i<l[j].size(); ++i)
            ss << ((i>0)?"--":"") << "(" << l[j][i].x() << "," << l[j][i].y() << ")";
        ss << "]";
    }
    return ss.str();
}



//////////////////////////// >>>>>>>>>>>>>>>> ARCHIVE/DEPRECATED <<<<<<<<<<<<<<<<<<<

/**********************************************************************
 * doesLineIntersectObstacle
 * @brief   Checks if a line intersects an obstacle. Points on obstacle boundary are allowed.
 * (p1,p2):     vertices of the line
 * obstacle:  list of vertices for the obstacle (OPEN polygon)
 * isEnvBndry:  should the line lie inside (TRUE) or outside (FALSE) the polygon
 *********************************************************************/
bool doesLineIntersectObstacle(Vector2f p1, Vector2f p2, const vector<Vector2f> &obstacle, bool isEnvBndry)
{
    vector<cv::Point2f> obstacleCV;
    VectorAns colcheck;
    bool test1_belong, test2_belong = false;
    int index1=0, index2=0; 
    unsigned int collision_loops = 20;
    Vector2f edgeP1, edgeP2;
    
    // Convert to CV points    
    obstacleCV.clear();
    for(unsigned int i = 0; i< obstacle.size(); ++i)
        obstacleCV.push_back( cv::Point2f(obstacle[i].x(),obstacle[i].y()) );

    // Special cases
    ROS_DEBUG("Localplan::doesLineIntersectObstacle: Checking (%f,%f)-(%f,%f) for obstacle collision",p1.x(),p1.y(),p2.x(),p2.y());
    if(isEnvBndry)
    {
        // If the obstacle is the environment boundary ... want line to lie INSIDE the polygon
        ROS_DEBUG("Localplan::doesLineIntersectObstacle: Checking intersection with environment");
        // Check if any of the points are outside the polygon
        if( (pointPolygonTest(obstacleCV, cv::Point2f(p1.x(), p1.y()), false) < 0) || (pointPolygonTest(obstacleCV, cv::Point2f(p2.x(), p2.y()), false) < 0) ) {
            return true;
        }
        // Special case: Points are on boundary but line segment is outside
        for (unsigned int m = 1; m < collision_loops; m++){
            if (pointPolygonTest(obstacleCV, cv::Point2f( p1.x()+m*(p2.x()-p1.x() )/collision_loops, p1.y()+m*(p2.y()-p1.y() )/collision_loops), false) < 0) {
            // if (pointPolygonTest(obstacleCV, cv::Point2f( p1.x()+(p1.x()-p1.x() )/(m+1),p1.y()+(p2.y()-p1.y() )/(m+1)), false) < 0){
                return true;
            }       
        }
    }
    else
    {
        // Checks for points along the boundary of a single obstacle
        ROS_DEBUG("Localplan::doesLineIntersectObstacle: Checking intersection with obstacle");
        test1_belong = false; 
        test2_belong = false;
        for (unsigned int i = 0; i < obstacle.size(); i++){
            //prevent collisions along obstacle edge.
            if ( p1 == obstacle[i] ) { 
               test1_belong = true;
               index1 = i;
            } 
            if ( p2 == obstacle[i] ){ 
               test2_belong = true;
               index2 = i;
            }
        }

        //prevent collisions along obstacle edge. 
        if (test1_belong && test2_belong){ 
            if(index2 > index1) swap(index1,index2); // make sure index1 >= index2

            // ROS_DEBUG("Localplan::doesLineIntersectObstacle: ... Are Obstacle Vertices (%u,%u)",index1,index2);
            //check if the two points are actually neighbours via index numbers.
            if (std::abs(index1-index2)==1){
                return false;
            }
            //corner case for wrap-around indices given open polygons
            if ( (index1 ==0 || index2 == 0)){
                if (std::abs((int)index1-index2) == (obstacle.size()-1)){
                    return false;
                }
            }
        }
        ROS_DEBUG("Localplan::doesLineIntersectObstacle: Done checking for obstacle edge");

        //check if the points lay on both sides of an obstacle
        for (unsigned int m = 1; m < collision_loops; m++){
            if (pointPolygonTest(obstacleCV, cv::Point2f( p1.x()+m*(p2.x()-p1.x() )/collision_loops, p1.y()+m*(p2.y()-p1.y() )/collision_loops), false) >0){
            // if (pointPolygonTest(obstacleCV, cv::Point2f( p1.x()+(p2.x()-p1.x() )/(m+1),p1.y()+(p2.y()-p1.y() )/(m+1)), false) >0){
                return true;                
            }
        }
        ROS_DEBUG("Localplan::doesLineIntersectObstacle: Done checking for crossing line");
    }

    //check against edge list.
    //TODO: check if intersects 2 non-adjacent edges (or vertices)
    ROS_DEBUG("Localplan::doesLineIntersectObstacle: Checking intersection with obstacle edge list");
    for(unsigned int l=0; l < obstacle.size(); ++l) // for each vertex
    {
        edgeP1 = obstacle[l];
        if(l == obstacle.size()-1) edgeP2 = obstacle[0];
        else edgeP2 = obstacle[l+1];

        //make sure the lines being checked are not the boundary edges themselves
        // ROS_DEBUG("p1(%f,%f), p2(%f,%f), edgeP1(%f,%f), edgeP2(%f,%f)", p1.x(),p1.y(), p2.x(),p2.y(), edgeP1.x(),edgeP1.y(), edgeP2.x(),edgeP2.y());
        if ( (p1 == edgeP1 && p2 == edgeP2) || (p1 == edgeP2 && p2 == edgeP1) ) {
            colcheck.ans = false;
        }
        else {
            // ROS_DEBUG("calling vectorIntersect for vertex %u ...", l);
            colcheck = vectorIntersect(p1, p2, edgeP1, edgeP2, false); 
            // ROS_DEBUG("done vectorIntersect for vertex %u ...", l);
        }
        if (colcheck.ans == true){
          return true; 
        }
    }

    ROS_DEBUG("Localplan::doesLineIntersectObstacle: Done");
    return false; 

}
