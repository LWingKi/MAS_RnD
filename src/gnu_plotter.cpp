#include "gnu_plotter.hpp"
#include "robif2b/functions/kinova_gen3.h"

GNUPlotter::GNUPlotter(std::string logs_dir,bool plot_data, bool save_data) : logs_dir_(logs_dir), save_data_(save_data) {
    // create the logs directory if it doesn't exist
    if (!std::filesystem::exists(logs_dir)) {
        std::filesystem::create_directory(logs_dir);
    }

    // get the number of subdirectories
    int n_subdirs = std::distance(std::filesystem::directory_iterator(logs_dir), std::filesystem::directory_iterator{});

    // create the new subdirectory: 0 if no subdirectories exist, else n_subdirs + 1
    std::string subdir = logs_dir + "/" + std::to_string(n_subdirs ? n_subdirs : 0);

    // create the subdirectory
    std::filesystem::create_directory(subdir);

    // set the logs dir to the new subdirectory
    logs_dir_ = subdir;
}

GNUPlotter::~GNUPlotter() {}

std::string GNUPlotter::getNewFileName(std::string filename) {
    // get current time
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);

    // convert to string
    std::stringstream ss;
    ss << std::put_time(std::localtime(&now_c), "%d_%m_%Y_%H_%M_%S");
    std::string time_str = ss.str();

    // create filename
    filename = logs_dir_ + "/" + filename + "_" + time_str + ".csv";

    return filename;
}

void GNUPlotter::saveDataToCSV(const std::vector<std::array<double, 3>>& positions, const std::array<double, 3>& target_pos){
    // get the file
    std::string filename = getNewFileName("xyz_positions");

    // create the file
    std::ofstream file(filename);

    file << "Index,X,Y,Z,Target_X,Target_Y,Target_Z\n";
    for (size_t i = 0; i < positions.size(); ++i) {
        file << i << ","
             << positions[i][0] << ","
             << positions[i][1] << ","
             << positions[i][2] << ","
             << target_pos[0] << ","
             << target_pos[1] << ","
             << target_pos[2] << "\n";
    }

    file.close();
}

void GNUPlotter::testPlot() {
    Gnuplot gp;

    // Generate test data
    std::vector<std::pair<double, double>> data;
    for (double x = 0; x <= 2 * M_PI; x += 0.1) {
        double y = std::sin(x);
        data.emplace_back(x, y);
    }

    // Plot the sine function
    gp << "set xlabel 'x'\n";
    gp << "set ylabel 'sin(x)'\n";
    gp << "plot '-' with lines title 'Sine function'\n";
    gp.send1d(data);
}



void GNUPlotter::plotXYZ(const std::vector<std::array<double, 3>>& positions, const std::array<double, 3>& target_pos) {

    // Save data to csv if save_data_ is true
    if (save_data_) {
        saveDataToCSV(positions, target_pos);
    }

    Gnuplot gp;
    gp << "set multiplot\n";
    gp << "set xlabel 'time'\n";

    // Prepare target_pos data
    std::vector<double> target_x(positions.size(), target_pos[0]);
    std::vector<double> target_y(positions.size(), target_pos[1]);
    std::vector<double> target_z(positions.size(), target_pos[2]);
    // std::vector<double> target_x(current_val.size(), target_val[0]);
    // std::vector<double> target_y(current_val.size(), target_val[1]);
    // std::vector<double> target_z(current_val.size(), target_val[2]);

    // Separate x, y, and z values from positions vector
    std::vector<double> x_values, y_values, z_values;
    for (const auto& pos : positions) {
        x_values.push_back(pos[0]);
        y_values.push_back(pos[1]);
        z_values.push_back(pos[2]);
    }

    // Plot X values
    gp << "set origin 0,0.66\n";
    gp << "set size 1,0.33\n";
    gp << "set ylabel 'X'\n";
    gp << "set ytics 0.002\n";
    gp << "plot '-' with lines title 'X values', '-' with lines title 'Target X'\n";
    gp.send1d(x_values);
    gp.send1d(target_x);

    // Plot Y values
    gp << "set origin 0,0.33\n";
    gp << "set size 1,0.33\n";
    gp << "set ylabel 'Y'\n";
    gp << "set ytics 0.003\n";
    gp << "plot '-' with lines title 'Y values', '-' with lines title 'Target Y'\n";
    gp.send1d(y_values);
    gp.send1d(target_y);

    // Plot Z values
    gp << "set origin 0,0\n";
    gp << "set size 1,0.33\n";
    gp << "set ylabel 'Z'\n";
    gp << "set ytics 0.05\n";
    gp << "plot '-' with lines title 'Z values', '-' with lines title 'Target Z'\n";
    gp.send1d(z_values);
    gp.send1d(target_z);

    gp << "unset multiplot\n";
}

//use with KDL
void GNUPlotter::saveDataToCSV(const std::vector<KDL::Vector>& current_val, const std::vector<KDL::Vector>& target_val,std::string logname){
    // get the filefilename
    std::string filename = getNewFileName(logname);

    // create the file
    std::ofstream file(filename);

    file << "Index,X,Y,Z,Target_X,Target_Y,Target_Z\n";
    for (size_t i = 0; i < current_val.size(); ++i) {
        file << i << ","
             << current_val[i].x() << ","
             << current_val[i].y() << ","
             << current_val[i].z() << ","
             << target_val[i].x() << ","
             << target_val[i].y() << ","
             << target_val[i].z() << "\n";
    }

    file.close();
}

void GNUPlotter::plotXYZ(const std::vector<KDL::Vector>& current_val, const std::vector<KDL::Vector>& target_val,std::string title,double ytick){

    std::vector<double> x_values, y_values, z_values;
    std::vector<double> target_x,target_y,target_z;
    if (save_data_) {
        saveDataToCSV(current_val, target_val,title+"_log");
    }

    for (const auto& curr : current_val) {
        x_values.push_back(curr.x());
        y_values.push_back(curr.y());
        z_values.push_back(curr.z());
    }
    for (const auto& target : target_val) {
        target_x.push_back(target.x());
        target_y.push_back(target.y());
        target_z.push_back(target.z());
    }

    Gnuplot gp;
    gp << "set multiplot\n";
    gp << "set xlabel 'time'\n";

    // Plot X values
    gp << "set origin 0,0.66\n";
    gp << "set size 1,0.33\n";
    gp << "set ylabel 'X'\n";
    gp << "set ytics "<< ytick <<"\n";
    gp << "plot '-' with lines title 'X values', '-' with lines title '"<< title <<"X'\n";
    gp.send1d(x_values);
    gp.send1d(target_x);

    // Plot Y values
    gp << "set origin 0,0.33\n";
    gp << "set size 1,0.33\n";
    gp << "set ylabel 'Y'\n";
    gp << "set ytics 0.01\n";
    gp << "plot '-' with lines title 'Y values', '-' with lines title '"<< title <<"Y'\n";
    gp.send1d(y_values);
    gp.send1d(target_y);

    // Plot Z values
    gp << "set origin 0,0\n";
    gp << "set size 1,0.33\n";
    gp << "set ylabel 'Z'\n";
    gp << "set ytics 0.01\n";
    gp << "plot '-' with lines title 'Z values', '-' with lines title '"<< title <<"Z'\n";
    gp.send1d(z_values);
    gp.send1d(target_z);

    gp << "unset multiplot\n";
}


void GNUPlotter::saveDataToCSV(const std::vector<KDL::JntArray>& q,
                     const std::vector<KDL::JntArray>& qdot,
                     const std::vector<KDL::JntArray>& qddot,
                     const std::vector<KDL::JntArray>& constraint_tau,
                     const std::vector<KDL::Vector>& current_vel, 
                     const std::vector<KDL::Vector>& target_vel,
                     const std::vector<KDL::Vector>& current_pos, 
                     const std::vector<KDL::Vector>& target_pos,
                     const std::vector<KDL::Twist>& control_signal,
                     const std::vector<std::array<double, 7>>& joint_torque,
                     const std::vector<std::array<double, 7>>& actuar_current,
                     const std::vector<std::array<double, 7>>& actuar_voltage,
                     std::string logname){

                        // get the filefilename
                std::string filename = getNewFileName(logname);

                // create the file
                std::ofstream file(filename);
                std::string str = "Index,";
                std::string str1 = "joint_angle_0,joint_angle_1,joint_angle_2,joint_angle_3,joint_angle_4,joint_angle_5,joint_angle_6,";
                std::string str2 = "joint_velocity_0,joint_velocity_1,joint_velocity_2,joint_velocity_3,joint_velocity_4,joint_velocity_5,joint_velocity_6,";
                std::string str3 = "joint_accel_0,joint_accel_1,joint_accel_2,joint_accel_3,joint_accel_4,joint_accel_5,joint_accel_6,";
                std::string str4 = "constraint_tau_0,constraint_tau_1,constraint_tau_2,constraint_tau_3,constraint_tau_4,constraint_tau_5,constraint_tau_6,";
                std::string str5 = "current_pos_x,current_pos_y,current_pos_z,";
                std::string str6 = "target_pos_x,target_pos_y,target_pos_z,";
                std::string str7 = "current_vel_x,current_vel_y,current_vel_z,";
                std::string str8 = "target_vel_x,target_vel_y,target_vel_z,";
                std::string str9 = "control_signal_linear_x,control_signal_linear_y,control_signal_linear_z,";
                std::string str10 = "control_signal_angular_x,control_signal_angular_y,control_signal_angular_z,";
                std::string str11 = "joint_torque_0,joint_torque_1,joint_torque_2,joint_torque_3,joint_torque_4,joint_torque_5,joint_torque_6,";
                std::string str12 = "actuar_current_0,actuar_current_1,actuar_current_2,actuar_current_3,actuar_current_4,actuar_current_5,actuar_current_6,";
                std::string str13 = "actuar_voltage_0,actuar_voltage_1,actuar_voltage_2,actuar_voltage_3,actuar_voltage_4,actuar_voltage_5,actuar_voltage_6\n";
                // std::string str8 =
                std::string result = str+str1 + str2+str3+str4+str5+str6+str7+str8+str9+str10+str11+str12+str13;
                file << result;
            
                for (size_t i = 0; i < current_vel.size(); ++i) {
                    file << i << ","
                        << q[i](0) << ","
                        << q[i](1) << ","
                        << q[i](2) << ","
                        << q[i](3) << ","
                        << q[i](4) << ","
                        << q[i](5) << ","
                        << q[i](6) << ","
                        << qdot[i](0) << ","
                        << qdot[i](1) << ","
                        << qdot[i](2) << ","
                        << qdot[i](3) << ","
                        << qdot[i](4) << ","
                        << qdot[i](5) << ","
                        << qdot[i](6) << ","
                        << qddot[i](0) << ","
                        << qddot[i](1) << ","
                        << qddot[i](2) << ","
                        << qddot[i](3) << ","
                        << qddot[i](4) << ","
                        << qddot[i](5) << ","
                        << qddot[i](6) << ","
                        << constraint_tau[i](0) << ","
                        << constraint_tau[i](1) << ","
                        << constraint_tau[i](2) << ","
                        << constraint_tau[i](3) << ","
                        << constraint_tau[i](4) << ","
                        << constraint_tau[i](5) << ","
                        << constraint_tau[i](6) << ","
                        << current_pos[i].x() << ","
                        << current_pos[i].y() << ","
                        << current_pos[i].z() << ","
                        << target_pos[i].x() << ","
                        << target_pos[i].y() << ","
                        << target_pos[i].z() << ","
                        << current_vel[i].x() << ","
                        << current_vel[i].y() << ","
                        << current_vel[i].z() << ","
                        << target_vel[i].x() << ","
                        << target_vel[i].y() << ","
                        << target_vel[i].z() << ","
                        << control_signal[i].vel.x() << ","
                        << control_signal[i].vel.y() << ","
                        << control_signal[i].vel.z() << ","
                        << control_signal[i].rot.x() << ","
                        << control_signal[i].rot.y() << ","
                        << control_signal[i].rot.z() << ","
                        << joint_torque[i][0] <<","
                        << joint_torque[i][1] <<","
                        << joint_torque[i][2] <<","
                        << joint_torque[i][3] <<","
                        << joint_torque[i][4] <<","
                        << joint_torque[i][5] <<","
                        << joint_torque[i][6] <<","
                        << actuar_current[i][0] <<","
                        << actuar_current[i][1] <<","
                        << actuar_current[i][2] <<","
                        << actuar_current[i][3] <<","
                        << actuar_current[i][4] <<","
                        << actuar_current[i][5] <<","
                        << actuar_current[i][6] <<","
                        << actuar_voltage[i][0] <<","
                        << actuar_voltage[i][1] <<","
                        << actuar_voltage[i][2] <<","
                        << actuar_voltage[i][3] <<","
                        << actuar_voltage[i][4] <<","
                        << actuar_voltage[i][5] <<","
                        << actuar_voltage[i][6] <<","
                        <<"\n";
                }

                file.close();
                     }
