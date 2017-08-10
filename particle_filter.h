/*
 * particle_filter.h
 *
 * 2D particle filter class.
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#ifndef PARTICLE_FILTER_H_
#define PARTICLE_FILTER_H_
#include "map.h"
#include "helper_functions.h"
//#include "Eigen/Dense"
#include <vector>
//using Eigen::MatrixXd;
//using Eigen::VectorXd;

struct Particle {

	int id;
	double x;
	double y;
	double theta;
	double weight;
	std::vector<int> associations;
	std::vector<double> sense_x;
	std::vector<double> sense_y;
	//std::vector<map_landmarks> landmark_list;
	//std::vector<single_landmark_s> landmark_list;
	
};



class ParticleFilter {
	
	// Number of particles to draw
	//int num_particles;
    //int i;
	//std::vector<single_landmark_s> landmark_list;
	
	// Flag, if filter is initialized
	bool is_initialized;
	//bool initialized();
	
	// Vector of weights of all particles
	//std::vector<double> weights;
	//VectorXd weights;
	//std::vector<single_landmark_s> landmark_list;
public:
	
	// Set of current particles
	std::vector<Particle> particles;
	//VectorXd particles;
	std::vector<double> weights;
//added
	//VectorXd pred_landmark;
	//VectorXd error;
	//VectorXd map_landmarks;
	//VectorXd transfx;
	//VectorXd transfy;
	//VectorXd landmark_list;
	std::vector<double>transfx;
	std::vector<double>transfy;
	//std::vector<map_landmarks> landmark_list;
	//std::vector<single_landmark_s> landmark_list;
	std::vector<double> pred_landmark;
	//std::vector<double> error;
	double error;
	double min_error;
	double sumweight;
	int sum;
	int counter;
	int num_particles;
	
	double particletheta;
	double tx;
	double ty;
	std::vector<int>multiples;
	std::vector<Particle>resampled; //or double???
	
	// Constructor
	// @param M Number of particles
	ParticleFilter() : num_particles(0), is_initialized(false) {}

	// Destructor
	~ParticleFilter() {}

	/**
	 * init Initializes particle filter by initializing particles to Gaussian
	 *   distribution around first position and all the weights to 1.
	 * @param x Initial x position [m] (simulated estimate from GPS)
	 * @param y Initial y position [m]
	 * @param theta Initial orientation [rad]
	 * @param std[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 */
	void init(double x, double y, double theta, double std[]);

	/**
	 * prediction Predicts the state for the next time step
	 *   using the process model.
	 * @param delta_t Time between time step t and t+1 in measurements [s]
	 * @param std_pos[] Array of dimension 3 [standard deviation of x [m], standard deviation of y [m]
	 *   standard deviation of yaw [rad]]
	 * @param velocity Velocity of car from t to t+1 [m/s]
	 * @param yaw_rate Yaw rate of car from t to t+1 [rad/s]
	 */
	void prediction(double delta_t, double std_pos[], double velocity, double yaw_rate);
	
	/**
	 * dataAssociation Finds which observations correspond to which landmarks (likely by using
	 *   a nearest-neighbors data association).
	 * @param predicted Vector of predicted landmark observations
	 * @param observations Vector of landmark observations
	 */
	void dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations);
	
	/**
	 * updateWeights Updates the weights for each particle based on the likelihood of the 
	 *   observed measurements. 
	 * @param sensor_range Range [m] of sensor
	 * @param std_landmark[] Array of dimension 2 [standard deviation of range [m],
	 *   standard deviation of bearing [rad]]
	 * @param observations Vector of landmark observations
	 * @param map Map class containing map landmarks
	 */
	void updateWeights(double sensor_range, double std_landmark[], std::vector<LandmarkObs> observations,
			Map map_landmarks);
	
	/**
	 * resample Resamples from the updated set of particles to form
	 *   the new set of particles.
	 */
	void resample();

	/*
	 * Set a particles list of associations, along with the associations calculated world x,y coordinates
	 * This can be a very useful debugging tool to make sure transformations are correct and assocations correctly connected
	 */
	Particle SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y);
	
	std::string getAssociations(Particle best);
	std::string getSenseX(Particle best);
	std::string getSenseY(Particle best);

	/**
	 * initialized Returns whether particle filter is initialized yet or not.
	 */
	const bool initialized() const {
		return is_initialized;
	}
};



#endif /* PARTICLE_FILTER_H_ */
