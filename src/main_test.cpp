#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>
#include <stdlib.h>
#include "Eigen/Dense"
#include "FusionEKF.h"
#include "measurement_package.h"
#include "tools.h"
#include <iomanip>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

void check_arguments(int argc, char* argv[])
{
  string usage_instructions = "Usage instructions: ";
  usage_instructions += argv[0];
  usage_instructions += " path/to/input.txt output.txt";

  bool has_valid_args = false;

  // make sure the user has provided input and output files
  if (argc == 1)
  {
    has_valid_args = true;
    //cerr << usage_instructions << endl;
  }
  else if (argc > 1)
  {
    cerr << "Too many arguments.\n" << usage_instructions << endl;
  }

  if (!has_valid_args)
  {
    exit(EXIT_FAILURE);
  }
}

void check_files(ifstream& in_file, string& in_name,
                 ofstream& out_file, string& out_name)
{
  if (!in_file.is_open())
  {
    cerr << "Cannot open input file: " << in_name << endl;
    exit(EXIT_FAILURE);
  }

  if (!out_file.is_open())
  {
    cerr << "Cannot open output file: " << out_name << endl;
    exit(EXIT_FAILURE);
  }
}

int main(int argc, char* argv[])
{
  check_arguments(argc, argv);

  string in_file_name_ = "../data/obj_pose-laser-radar-synthetic-input.txt"; //argv[1];
  ifstream in_file_(in_file_name_.c_str(), ifstream::in);

  string out_file_name_ = "../output/RMSE.txt"; //argv[2];
  ofstream out_file_(out_file_name_.c_str(), ofstream::out);

  check_files(in_file_, in_file_name_, out_file_, out_file_name_);

  /**********************************************
   *  Set Measurements                          *
   **********************************************/

  vector<MeasurementPackage> measurement_pack_list;
  vector<VectorXd> gt_pack_list;

  string line;

  // prep the measurement packages (each line represents a measurement at a
  // timestamp)
  while (getline(in_file_, line))
  {
    string sensor_type;
    MeasurementPackage meas_package;
    istringstream iss(line);
    long long timestamp;

    // reads first element from the current line
    iss >> sensor_type;

    if (sensor_type.compare("L") == 0)
    {
      // laser measurement
      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::LASER;
      meas_package.raw_measurements_ = VectorXd(2);
      float px;
      float py;
      iss >> px;
      iss >> py;
      meas_package.raw_measurements_ << px, py;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }
    else if (sensor_type.compare("R") == 0)
    {
      // radar measurement
      // read measurements at this timestamp
      meas_package.sensor_type_ = MeasurementPackage::RADAR;
      meas_package.raw_measurements_ = VectorXd(3);
      float ro;
      float phi;
      float ro_dot;
      iss >> ro;
      iss >> phi;
      iss >> ro_dot;
      meas_package.raw_measurements_ << ro, phi, ro_dot;
      iss >> timestamp;
      meas_package.timestamp_ = timestamp;
      measurement_pack_list.push_back(meas_package);
    }

      // read ground truth data to compare later
      float x_gt;
      float y_gt;
      float vx_gt;
      float vy_gt;
      iss >> x_gt;
      iss >> y_gt;
      iss >> vx_gt;
      iss >> vy_gt;
      VectorXd gt_values = VectorXd(4);
      gt_values << x_gt, y_gt, vx_gt, vy_gt;
      gt_pack_list.push_back(gt_values);
  }

  /**********************************************
   *  Measurements                              *
   **********************************************/

   // Create a Kalman Filter instance
  FusionEKF fusionEKF;

  // used to compute the RMSE later
  Tools tools;
  vector<VectorXd> estimations;
  vector<VectorXd> ground_truth;

  size_t number_of_measurements = measurement_pack_list.size();
  for (size_t k = 0; k < number_of_measurements; ++k)
  {
    //Call ProcessMeasurment(meas_package) for Kalman filter
    fusionEKF.ProcessMeasurement(measurement_pack_list[k]);

    //Push the current estimated x,y positon from the Klaman filter's state vector

    VectorXd estimate(4);

    double p_x = fusionEKF.ekf_.x_(0);
    double p_y = fusionEKF.ekf_.x_(1);
    double v1  = fusionEKF.ekf_.x_(2);
    double v2 = fusionEKF.ekf_.x_(3);

    estimate(0) = p_x;
    estimate(1) = p_y;
    estimate(2) = v1;
    estimate(3) = v2;

    estimations.push_back(estimate);
    ground_truth.push_back(gt_pack_list[k]);
  }

  VectorXd rmse = tools.CalculateRMSE(estimations, ground_truth);

  cout << "RMSE: "
      << std::fixed << std::setprecision(6) << rmse[0] << " "
      << std::fixed << std::setprecision(6) << rmse[1] << " "
      << std::fixed << std::setprecision(6) << rmse[2] << " "
      << std::fixed << std::setprecision(6) << rmse[3] << endl;

  // close files
  if (out_file_.is_open())
  {
    out_file_.close();
  }

  if (in_file_.is_open())
  {
    in_file_.close();
  }

  cout << "Done!" << endl;
  return 0;
}
