#include <gnuplot-iostream.h>

#include <array>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <vector>

#include "robif2b/functions/kinova_gen3.h"
#include "kdl/chain.hpp"
#include "kdl/frames_io.hpp"
#include "kdl/framevel_io.hpp"
#include "kdl/kinfam_io.hpp"
#include "kdl/frames.hpp"
#include "kdl/chainfksolver.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"
#include "kdl/chainfksolvervel_recursive.hpp"
#include "kdl/chainhdsolver_vereshchagin.hpp"
#include "kdl_parser/kdl_parser.hpp"
#include "robot.hpp"
#include "gnu_plotter.hpp"
#include "tf_utils.hpp"
#include "logger.hpp"

namespace fs = std::filesystem;
int main() {
    std::filesystem::path path = __FILE__;
    std::fstream file;
    // std::string line;
    const std::string csvFilePath = (path.parent_path()/"log/us2/contactless/").string();
    int folderCount = 0;

    for (const auto& entry : fs::directory_iterator(csvFilePath)) {
        if (entry.is_directory()) {
            folderCount++;
        }
    }

    // Define the column names you want to extract
    const std::vector<std::string> desiredColumns = {
            "joint_angle_0", "joint_angle_1", "joint_angle_2",
            "joint_angle_3", "joint_angle_4", "joint_angle_5",
            "joint_angle_6"
        };

    
    for (int i = 0; i <= folderCount; ++i) {
        // Create an input file stream for reading the CSV file
        std::string folder_path = csvFilePath + std::to_string(i);
        if (fs::exists(folder_path) && fs::is_directory(folder_path)) {
            if (fs::exists(folder_path) && fs::is_directory(folder_path)) {
                for (const auto& entry : fs::directory_iterator(folder_path)) {
                    if (entry.is_regular_file()) {
                        std::string file_path = entry.path();
                        if (file_path.size() >= 4 && file_path.compare(file_path.size() - 4, 4, ".csv") == 0) {
                            std::ifstream csvFile(file_path);
                            // Check if the file is open and ready for reading
                            if (!csvFile.is_open()) {
                                std::cerr << "Error: Unable to open the CSV file." << std::endl;
                                return 1;
                            }
                            //ROBOT
                            //Create robot chain
                            KDL::Tree my_tree;
                            KDL::Chain robot;

                            // get current file path
                            std::filesystem::path path2 = __FILE__;
                            std::string robot_urdf = (path2.parent_path().parent_path()/ "gen3_robotiq_2f_85.urdf").string();
                            // std::string robot_urdf = (path.parent_path().parent_path()/ "gen3_7dof_vision_arm_urdf_v12.urdf").string();

                            if (!kdl_parser::treeFromFile(robot_urdf, my_tree)){
                                std::cout << "Failed to construct kdl tree" << std::endl;
                                return -1;
                            }
                            // set the base and tool frames
                            std::string base_link = "base_link";
                            std::string tool_frame = "bracelet_link";

                            // create the kdl chain
                            if (!my_tree.getChain(base_link, tool_frame, robot)){
                                std::cout << "Failed to get kdl chain" << std::endl;
                                return -1;
                            }

                            KDL::JntArray q(robot.getNrOfJoints());     
                            KDL::ChainFkSolverPos_recursive fksolver = KDL::ChainFkSolverPos_recursive(robot);
                            KDL::Frame currPos;
                            std::vector<KDL::Vector> current_position;
                        // Read the header line to determine the column indices
                            std::string headerLine;
                            if (std::getline(csvFile, headerLine)) {
                                std::istringstream headerStream(headerLine);
                                std::string columnHeader;

                                // Create a map to store the mapping of column names to column indices
                                std::vector<int> columnIndexMap(desiredColumns.size(), -1);

                                int columnIndex = 0;
                                while (std::getline(headerStream, columnHeader, ',')) {
                                    // Check if the current column header is one of the desired columns
                                    for (int i = 0; i < desiredColumns.size(); i++) {
                                        if (columnHeader == desiredColumns[i]) {
                                            columnIndexMap[i] = columnIndex;
                                        }
                                    }
                                    columnIndex++;
                                }

                                // Read and print data from the desired columns, skipping the "index" column
                                std::string line;
                                while (std::getline(csvFile, line)) {
                                    std::istringstream lineStream(line);
                                    std::string cell;
                                    int outputColumnIndex = 0;

                                    while (std::getline(lineStream, cell, ',')) {
                                        // Skip the "index" column (first cell)
                                        if (outputColumnIndex == 0) {
                                            outputColumnIndex++;
                                            continue;
                                        }

                                        // Check if the current column index is one of the desired indices
                                        if (outputColumnIndex - 1 < columnIndexMap.size() &&
                                            columnIndexMap[outputColumnIndex - 1] != -1) {
                                            // std::cout << "Data for Column \"" << desiredColumns[outputColumnIndex - 1] << "\": " << cell << std::endl;
                                            // std::cout << "INDEX: " << outputColumnIndex-1 << std::endl;
                                            q(outputColumnIndex-1) = std::stod(cell);
                                        }
                                        outputColumnIndex++;
                                    }
                                    // std::cout << "q: "<< q << std::endl;
                                    fksolver.JntToCart(q, currPos);
                                    current_position.push_back(currPos.p);
                                    // std::cout << "currPos: " << currPos.p << std::endl;
                                }
                            } else {
                                std::cerr << "Error: Unable to read the header line of the CSV file." << std::endl;
                                csvFile.close();
                                return 1;
                            }
                            // Close the CSV file stream
                            csvFile.close();

                            GNUPlotter plot_graph(path.parent_path()/"log/us2/contactlessos",false,false);
                            plot_graph.saveDataToCSV(current_position,"sos_log");
                        }
                    }
                }
            }
        }
    }
    // const std::string csvFilePath2 = (path.parent_path()/"log/us2/contact/0/sos.csv").string();
}