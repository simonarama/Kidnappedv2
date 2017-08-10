/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <vector>
#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <sstream>
#include <string>
#include <iterator>
#include "map.h"
#include <map>

//#include "Eigen/Dense"

//David Simon lecture code modified and accessing structure code modified from forum

using std::vector;

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
    // TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
    //   x, y, theta and their uncertainties from GPS) and all weights to 1. 
    // Add random Gaussian noise to each particle.
    // NOTE: Consult particle_filter.h for more information about this method (and others in this file).
    //is_initialized = false;
    num_particles = 1;  //had 150 same results
    //cout<<"number of particles"<<endl<<num_particles<<endl<<endl;
    default_random_engine gen;
    
    //initialize
    
    
    //num_particles = weights.size(); ??problem
	
	
    
    double weight = 1.0;
    //cout<<"weight"<<endl<<weight<<endl<<endl;
	
	

    // This line creates a normal (Gaussian) distribution for x
    normal_distribution<double> dist_x(x, std[0]);
    normal_distribution<double> dist_y(y, std[1]);
    normal_distribution<double> dist_theta(theta, std[2]);
    //cout<<"past normal dist"<<endl<<endl;
    //int i;
    for (int i = 0; i < num_particles; i++){
        
        //cout<<"i"<<endl<<i<<endl<<endl;
        
        Particle particle = {};
        particle.id = i;
        particle.x = dist_x(gen);//had particles[i].x
        cout<<"particle x"<<endl<<particle.x<<endl<<endl;
        particle.y = dist_y(gen); 
        cout<<"particle y"<<endl<<particle.y<<endl<<endl;
        particle.theta = dist_theta(gen);
        cout<<"particle theta"<<endl<<particle.theta<<endl<<endl;
        particle.weight = weight;
        cout<<"particle weight"<<endl<<particle.weight<<endl<<endl;
        particles.push_back(particle);
		weights.push_back(weight); //added
        
}
        
	//cout<<"before initialization command"<<endl;
    is_initialized = true;
    
    //cout<<"after initialization command"<<endl;
    
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
    // TODO: Add measurements to each particle and add random Gaussian noise.
    // NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
    //  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
    //  http://www.cplusplus.com/reference/random/default_random_engine/
    //default_random_engine gen;
    //vector<Particle> particles = pf.particles;
    int num_particles = 1;  //had 150 same result
    //cout<<"just before prediction loop"<<endl<<endl;
    for (int i = 0; i < num_particles; ++i) {
        // particles[i].x, particles[i].y, particles[i].theta, particles[i].weight;
        Particle particle = {};
        particle.id = i;
		//cout<<"velocity"<<endl<<velocity<<endl<<endl;
		//cout<<"yaw rate"<<endl<<yaw_rate<<endl<<endl;
		//cout<<"particle x"<<endl<<particles[i].x<<endl<<endl;
		//cout<<"particle y"<<endl<<particles[i].y<<endl<<endl;
		//cout<<"particle theta"<<endl<<particles[i].theta<<endl<<endl;
        particle.x = particles[i].x + (velocity/yaw_rate)*(sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
        particle.y = particles[i].y + (velocity/yaw_rate)*(cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
        particle.theta = particles[i].theta + yaw_rate * delta_t;
        default_random_engine gen;
        //add Gaussian noise
        normal_distribution<double> dist_x(particle.x, std_pos[0]);
        normal_distribution<double> dist_y(particle.y, std_pos[1]);
        normal_distribution<double> dist_theta(particle.theta, std_pos[2]);
     
        // particles[i].x, particles[i].y, particles[i].theta, particles[i].weight;
        particle.x = dist_x(gen);
		cout<<"predictx"<<endl<<particle.x<<endl;
        particle.y = dist_y(gen);
		cout<<"predicty"<<endl<<particle.y<<endl;
        particle.theta = dist_theta(gen);
		cout<<"predicttheta"<<endl<<particle.theta<<endl;
        particles.push_back(particle);}
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
    // TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
    //   observed measurement to this particular landmark.
    // NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
    //   implement this method and use it as a helper during the updateWeights phase.



}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
        std::vector<LandmarkObs> observations, Map map_landmarks) {
    // TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
    //   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
    // NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
    //   according to the MAP'S coordinate system. You will need to transform between the two systems.
    //   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
    //   The following is a good resource for the theory:
    //   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
    //   and the following is a good resource for the actual equation to implement (look at equation 
    //   3.33
    // was  Map map_landmarks
	// landmark_list.size
	double sumweight = 0.0;
	//initialize transfx, transfy
	for(int i = 0; i < observations.size(); i++){
		//transfx[i] = 1.0; //initialize 
		//transfy[i] = 1.0; //initialize
		transfx.push_back(0.0);
		transfy.push_back(0.0);
	}
	
	//reinitialize weights to 1.0 to avoid drift, needed for continuing iterations
	for(int i = 0; i < num_particles; i++){
		weights[i] = 1.0;}
	
	//initialize pred_landmark, error
	for(int m = 0; m < 42; m++){
	    pred_landmark.push_back(70.0);}
	    
	
	//std::vector<single_landmark_s> landmark_list(42);
	//Map::single_landmark_s landmarks = map_landmarks.landmark_list;???? needed
	
	//cout<<"updateweights start"<<endl<<endl;
	int num_particles = 1;  //had 150 same result
	//cout<<"number of particles"<<endl<<num_particles<<endl<<endl;
    for(int i = 0; i < num_particles; ++i) {  
    //transform each particle sensor readings
	    double weight = 1.0;  //test this
        //cout<<"i update"<<endl<<i<<endl<<endl; 
        for(int j = 0; j < observations.size(); j++){
            //cout<<"j update"<<endl<<j<<endl<<endl;
			LandmarkObs obs;
			
            
            //Particle particle = {};
			
			//particle.id = j;
			
		   //particles[j].x, particles[j].y, particles[j].theta; 
		    
		    //cout<<"particle x"<<endl<<particles[i].x<<endl<<endl;
		    //cout<<"particle y"<<endl<<particles[i].y<<endl<<endl;
		    //cout<<"particle theta"<<endl<<particles[i].theta<<endl<<endl;
			//cout<<"observations x"<<endl<<observations[j].x<<endl<<endl;
			//cout<<"observations y"<<endl<<observations[j].y<<endl<<endl;
			//Particle particle = {};
			
			//per internet
			transfx[j] = particles[i].x + ((observations[j].x) * (cos(particles[i].theta))) + ((observations[j].y) * (sin(particles[i].theta)));
			transfy[j] = particles[i].y + (observations[j].x * (-1.0) * sin(particles[i].theta)) + (observations[j].y * cos(particles[i].theta));
            
			//transformation per forum mentor below, no impact on results
			//transfx[j] = particles[i].x + ((observations[j].x) * (cos(particles[i].theta))) - ((observations[j].y) * (sin(particles[i].theta)));
			//transfy[j] = particles[i].y + (observations[j].x * (+1.0) * sin(particles[i].theta)) + (observations[j].y * cos(particles[i].theta));
			//cout<<"transfx"<<endl<<transfx[j]<<endl<<endl;
			//cout<<"transfy"<<endl<<transfy[j]<<endl<<endl;
			
            //transfx[j] = particles[j].x + ((observations[j].x) * (cos(particles[j].theta))) + ((observations[j].y) * (sin(particles[j].theta)));
		    //cout<<"j"<<endl<<j<<endl<<endl;
			
            double min_error = 80.0;
            double error = 90.0;
            for(int k = 0; k < 42; k++){
                //double dist(double x1, double y1, double x2, double y2)
                //pred_landmark(k) = dist(transfx, transfy, map_landmarks, map_landmarks);
				//map_landmarks list;
				
				
				//cout<<"k"<<endl<<k<<endl<<endl;
                pred_landmark[k] = sqrt((map_landmarks.landmark_list[k].x_f - transfx[j])*(map_landmarks.landmark_list[k].x_f - transfx[j])+(map_landmarks.landmark_list[k].y_f - transfy[j])*(map_landmarks.landmark_list[k].y_f - transfy[j]));
                //cout<<"map x"<<endl<<map_landmarks.landmark_list[k].x_f<<endl;
				//cout<<"transfx"<<endl<<transfx[j]<<endl;
				//cout<<"map y"<<endl<<map_landmarks.landmark_list[k].y_f<<endl;
				//cout<<"transfy"<<endl<<transfy[j]<<endl;
				//cout<<"pred_landmark"<<endl<<pred_landmark[k]<<endl<<endl;
				//if (dist <= sensor_range){
				//if(k = 1){
					//min_error = pred_landmark[k];
					
				if(k >= 1){
					if(pred_landmark[k] < pred_landmark[k-1]){
						error = pred_landmark[k];}

                    else if (pred_landmark[k] > pred_landmark[k-1]){
					    error = pred_landmark[k-1];}
						}
				if(error < min_error){
					min_error = error;}
				else{
					min_error = min_error;}
            cout<<"min error"<<endl<<min_error<<endl;
						}
			//weight for one observation
            weights[i] *= (1/(2*M_PI*std_landmark[0]*std_landmark[1]))* (exp((-min_error * min_error)/(2*std_landmark[0]*std_landmark[1]))); 
            
			cout<<"weights[i]"<<endl<<weights[i]<<endl;
			//weights[i] *= weights[i];
			//cout<<"weight"<<endl<<weights[i]<<endl<<endl;
			//if(j = observations.size()){
				//Particle particle = {};
				//particle.id = i;
				//weights.push_back(weight);
				//particle.weight = weights[i];
				//particles.push_back(particle);
				//cout<<"i"<<endl<<i<<endl<<endl;
				//cout<<"particle weight"<<endl<<particles[i].weight<<endl<<endl;
			}
			Particle particle ={};
			weight = weights[i];
			particle.weight = weight;
			sumweight = sumweight + weight;
			cout<<"i"<<endl<<i<<endl;
			cout<<"weight"<<endl<<weight<<endl;
			weights.push_back(weight);
			particles.push_back(particle);
            //sumweight += weight;
			
			} 
            //cout<<"sumweight"<<endl<<sumweight<<endl;
			
			// use i ???
			//for(int i = 0; i < num_particles; ++i){
				//cout<<"i"<<endl<<i<<endl;
				//cout<<"weight"<<endl<<weights[i]<<endl;
				//cout<<"prob"<<endl<<weights[i]/sumweight<<endl;}
			//cout<<"i"<<endl<<i<<endl<<endl;
			//particles[i].weight = weight;
			}
            





void ParticleFilter::resample() {
    // TODO: Resample particles with replacement with probability proportional to their weight. 
    // NOTE: You may find std::discrete_distribution helpful here.
    //   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution
    
	//Particle particle = {};
	int i;
	Particle &particle = particles[i];
	//&weights = weights[i];
	//std::discrete_distribution<>(weights);
	//cout<<"start of resample"<<endl;
	int num_particles = 1;  //had 150 same result
    //default_random_engine gen; //??? needed
    
    //initialize
    //int main(){
    //discrete distribution code modified from internet and forum mentor
    std::random_device rd;
    std::mt19937 gen(rd());
	std::discrete_distribution<>d{particles[0].weight};
	
	//std::discrete_distribution<>d{weights[0]};
	
    //std::discrete_distribution<> d{weights[0],weights[1],weights[2],weights[3],weights[4],weights[5],weights[6],weights[7], 
	//weights[8],weights[9],weights[10],weights[11],weights[12],weights[13],weights[14],weights[15],weights[16],weights[17],
	//weights[18],weights[19],weights[20],weights[21],weights[22],weights[23],weights[24],weights[25],weights[26],weights[27],
	//weights[28],weights[29],weights[30],weights[31],weights[32],weights[33],weights[34],weights[35],weights[36],weights[37],
	//weights[38],weights[39],weights[40],weights[41],weights[42],weights[43],weights[44],weights[45],weights[46],weights[47],
	//weights[48],weights[49],weights[50],weights[51],weights[52],weights[53],weights[54],weights[55],weights[56],weights[57],
	//weights[58],weights[59],weights[60],weights[61],weights[62],weights[63],weights[64],weights[65],weights[66],weights[67],
	//weights[68],weights[69],weights[70],weights[71],weights[72],weights[73],weights[74],weights[75],weights[76],weights[77],
	//weights[78],weights[79],weights[80],weights[81],weights[82],weights[83],weights[84],weights[85],weights[86],weights[87],
	//weights[88],weights[89],weights[90],weights[91],weights[92],weights[93],weights[94],weights[95],weights[96],weights[97],
	//weights[98],weights[99],weights[100],weights[101],weights[102],weights[103],weights[104],weights[105],weights[106],weights[107],
	//weights[108],weights[109],weights[110],weights[111],weights[112],weights[113],weights[114],weights[115],weights[116],weights[117],
	//weights[118],weights[119],weights[120],weights[121],weights[122],weights[123],weights[124],weights[125],weights[126],weights[127],
	//weights[128],weights[129],weights[130],weights[131],weights[132],weights[133],weights[134],weights[135],weights[136],weights[137],
	//weights[138],weights[139],weights[140],weights[141],weights[142],weights[143],weights[144],weights[145],weights[146],weights[147],
	//weights[148],weights[149]};
	
	
	
	//weights.begin(), weights.end()  not working
	
    std::map<int, int> m;
    for(int n = 0; n < num_particles; ++n) {
       // ++m[d(gen)];
		Particle particle = particles[d(gen)];
        particles.push_back(particle);
    }
    //for(auto p : m) {
	//std::cout << p.first << " generated " << p.second << " times\n";}
    
}



Particle ParticleFilter::SetAssociations(Particle particle, std::vector<int> associations, std::vector<double> sense_x, std::vector<double> sense_y)
{
	//particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
	// associations: The landmark id that goes along with each listed association
	// sense_x: the associations x mapping already converted to world coordinates
	// sense_y: the associations y mapping already converted to world coordinates
 
	//Clear the previous associations
	particle.associations.clear();
	particle.sense_x.clear();
	particle.sense_y.clear();

	particle.associations= associations;
 	particle.sense_x = sense_x;
 	particle.sense_y = sense_y;

 	return particle;
}

string ParticleFilter::getAssociations(Particle best)
{
	vector<int> v = best.associations;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<int>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseX(Particle best)
{
	vector<double> v = best.sense_x;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}
string ParticleFilter::getSenseY(Particle best)
{
	vector<double> v = best.sense_y;
	stringstream ss;
    copy( v.begin(), v.end(), ostream_iterator<float>(ss, " "));
    string s = ss.str();
    s = s.substr(0, s.length()-1);  // get rid of the trailing space
    return s;
}