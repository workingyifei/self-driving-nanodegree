/*
 * particle_filter.cpp
 *
 *  Created on: Dec 12, 2016
 *      Author: Tiffany Huang
 */

#include <random>
#include <algorithm>
#include <iostream>
#include <numeric>
#include <math.h> 
#include <iostream>
#include <sstream>
#include <string>
#include <iterator>

#include "particle_filter.h"

using namespace std;

void ParticleFilter::init(double x, double y, double theta, double std[]) {
	// TODO: Set the number of particles. Initialize all particles to first position (based on estimates of 
	//   x, y, theta and their uncertainties from GPS) and all weights to 1. 
	// Add random Gaussian noise to each particle.
	// NOTE: Consult particle_filter.h for more information about this method (and others in this file).
  num_particles = 100;

  std::default_random_engine gen;

  std::normal_distribution<double> N_x(x, std[0]);
  std::normal_distribution<double> N_y(y, std[1]);
  std::normal_distribution<double> N_theta(theta, std[2]);

  for (int i = 0; i < num_particles; i++) {
    Particle particle;
    particle.id = i;
    particle.x = N_x(gen);
    particle.y = N_y(gen);
    particle.theta = N_theta(gen);
    particle.weight = 1;

    particles.push_back(particle);
    weights.push_back(1);
  }

  is_initialized = true;
}

void ParticleFilter::prediction(double delta_t, double std_pos[], double velocity, double yaw_rate) {
	// TODO: Add measurements to each particle and add random Gaussian noise.
	// NOTE: When adding noise you may find std::normal_distribution and std::default_random_engine useful.
	//  http://en.cppreference.com/w/cpp/numeric/random/normal_distribution
	//  http://www.cplusplus.com/reference/random/default_random_engine/

  default_random_engine gen;

  for (int i = 0; i < num_particles; i++) {
    double new_x;
    double new_y;
    double new_theta;

    if (yaw_rate == 0) {
      new_x = particles[i].x + velocity * delta_t * cos(particles[i].theta);
      new_y = particles[i].y + velocity * delta_t * sin(particles[i].theta);
      new_theta = particles[i].theta;
    }
    else {
      new_x = particles[i].x + velocity / yaw_rate * (sin(particles[i].theta + yaw_rate * delta_t) - sin(particles[i].theta));
      new_y = particles[i].y + velocity / yaw_rate * (cos(particles[i].theta) - cos(particles[i].theta + yaw_rate * delta_t));
      new_theta = particles[i].theta + yaw_rate * delta_t;
    }

    normal_distribution<double> N_x(new_x, std_pos[0]);
    normal_distribution<double> N_y(new_y, std_pos[1]);
    normal_distribution<double> N_theta(new_theta, std_pos[2]);

    particles[i].x = N_x(gen);
    particles[i].y = N_y(gen);
    particles[i].theta = N_theta(gen);
  }
}

void ParticleFilter::dataAssociation(std::vector<LandmarkObs> predicted, std::vector<LandmarkObs>& observations) {
	// TODO: Find the predicted measurement that is closest to each observed measurement and assign the 
	//   observed measurement to this particular landmark.
	// NOTE: this method will NOT be called by the grading code. But you will probably find it useful to 
	//   implement this method and use it as a helper during the updateWeights phase.
  size_t T = observations.size();
  for (int i = 0; i < T; i++) {

    LandmarkObs obs = observations[i];

    double minimum_distance = std::numeric_limits<double>::max(); // set minimum_distance to possible maximum value of a double

    for (int j = 0; j < predicted.size(); j++) {

      LandmarkObs predict = predicted[j];

      double distance = dist(predict.x, predict.y, obs.x, obs.y);

      if (distance < minimum_distance) {
        observations[i].id = predict.id;
        minimum_distance = distance;
      }
    }
  }
}

void ParticleFilter::updateWeights(double sensor_range, double std_landmark[], 
		const std::vector<LandmarkObs> &observations, const Map &map_landmarks) {
	// TODO: Update the weights of each particle using a mult-variate Gaussian distribution. You can read
	//   more about this distribution here: https://en.wikipedia.org/wiki/Multivariate_normal_distribution
	// NOTE: The observations are given in the VEHICLE'S coordinate system. Your particles are located
	//   according to the MAP'S coordinate system. You will need to transform between the two systems.
	//   Keep in mind that this transformation requires both rotation AND translation (but no scaling).
	//   The following is a good resource for the theory:
	//   https://www.willamette.edu/~gorr/classes/GeneralGraphics/Transforms/transforms2d.htm
	//   and the following is a good resource for the actual equation to implement (look at equation 
	//   3.33
	//   http://planning.cs.uiuc.edu/node99.html

  double std_x = std_landmark[0];
  double std_y = std_landmark[1];
  double var_x = std_x * std_x;
  double var_y = std_y * std_y;
  auto coeff_normalizer = 1 / (2 * M_PI * std_x * std_y);
  double x_denom = 2 * var_x;
  double y_denom = 2 * var_y;

  size_t T = observations.size();

  for (int i = 0; i < num_particles; ++i) {

    double particle_x = particles[i].x;
    double particle_y = particles[i].y;
    double particle_theta = particles[i].theta;


    // translate landmark from vehicle coordinates to map coordinates
    vector<LandmarkObs> trans_observations;
    LandmarkObs obs;

    for (size_t t = 0; t < T; t++) {
      LandmarkObs trans_obs;

      obs = observations[t];
      double obs_x = obs.x;
      double obs_y = obs.y;

      // perform the space transformation from vehicle to map
      trans_obs.x = particle_x + obs_x * cos(particle_theta) - obs_y * sin(particle_theta);
      trans_obs.y = particle_y + obs_x * sin(particle_theta) + obs_y * cos(particle_theta);
      trans_observations.push_back(trans_obs);
    }

    // filter landmarks
    vector<LandmarkObs> predicted_landmarks;
    for (int j = 0; j < map_landmarks.landmark_list.size(); j++) {

      double landmark_x = map_landmarks.landmark_list[j].x_f;
      double landmark_y = map_landmarks.landmark_list[j].y_f;
      double dist_to_landmark = dist(particle_x, particle_y, landmark_x, landmark_y);

      if (dist_to_landmark <= sensor_range) {
        LandmarkObs prediction;
        prediction.id = map_landmarks.landmark_list[j].id_i;
        prediction.x = map_landmarks.landmark_list[j].x_f;
        prediction.y = map_landmarks.landmark_list[j].y_f;
        predicted_landmarks.push_back(prediction);
      }

    }

    dataAssociation(predicted_landmarks, trans_observations);

    particles[i].weight = 1.0;

    // update weights
    for (int k = 0; k < trans_observations.size(); k++) {
      LandmarkObs obs = trans_observations[k];
      double mu_x;
      double mu_y;

      for (int l = 0; l < predicted_landmarks.size(); l++) {
        LandmarkObs landmark = predicted_landmarks[l];
        if (landmark.id == obs.id) {
          mu_x = landmark.x;
          mu_y = landmark.y;
        }

      }
      double x_num = (obs.x - mu_x) * (obs.x - mu_x);
      double y_num = (obs.y - mu_y) * (obs.y - mu_y);

      particles[i].weight *= coeff_normalizer * exp( -x_num / x_denom - y_num / y_denom);

    }

    weights[i] = particles[i].weight;

  }
}

void ParticleFilter::resample() {
	// TODO: Resample particles with replacement with probability proportional to their weight. 
	// NOTE: You may find std::discrete_distribution helpful here.
	//   http://en.cppreference.com/w/cpp/numeric/random/discrete_distribution

  vector<Particle> new_particles;

  std::random_device randomDevice;
  std::mt19937 gen(randomDevice());

  discrete_distribution<int> discreteDistributionWeights(0, num_particles - 1);
  int weight_index = discreteDistributionWeights(gen);

  default_random_engine generator;

  double max_weight = *max_element(weights.begin(), weights.end());
  uniform_real_distribution<double> beta_dist(0.0, (2 * max_weight));

  double beta = 0.0;

  // resampling
  for (int i = 0; i < num_particles; i++) {

    beta = beta + beta_dist(generator);

    while (weights[weight_index] < beta) {
      beta = beta - weights[weight_index];
      weight_index = (weight_index + 1) % num_particles;
    }

    new_particles.push_back(particles[weight_index]);

  }

  particles = new_particles;
}

Particle ParticleFilter::SetAssociations(Particle& particle, const std::vector<int>& associations, 
                                     const std::vector<double>& sense_x, const std::vector<double>& sense_y)
{
    //particle: the particle to assign each listed association, and association's (x,y) world coordinates mapping to
    // associations: The landmark id that goes along with each listed association
    // sense_x: the associations x mapping already converted to world coordinates
    // sense_y: the associations y mapping already converted to world coordinates

    particle.associations= associations;
    particle.sense_x = sense_x;
    particle.sense_y = sense_y;
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
