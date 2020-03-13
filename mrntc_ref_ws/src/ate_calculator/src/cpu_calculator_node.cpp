// node that logs the CPU usage and timestamp into a file. 
// the code for caluclating cpu usage comes from https://rosettacode.org/wiki/Linux_CPU_utilization#C.2B.2B

#include <fstream>
#include <iostream>
#include <numeric>
#include <unistd.h>
#include <vector>
#include <ros/ros.h>
 
std::vector<size_t> get_cpu_times() {
    std::ifstream proc_stat("/proc/stat");
    proc_stat.ignore(5, ' '); // Skip the 'cpu' prefix.
    std::vector<size_t> times;
    for (size_t time; proc_stat >> time; times.push_back(time));
    return times;
}
 
bool get_cpu_times(size_t &idle_time, size_t &total_time) {
    const std::vector<size_t> cpu_times = get_cpu_times();
    if (cpu_times.size() < 4)
        return false;
    idle_time = cpu_times[3];
    total_time = std::accumulate(cpu_times.begin(), cpu_times.end(), 0);
    return true;
}
 
int main(int argc, char** argv) {
  ros::init(argc, argv, "cpu_calculator");
  ros::NodeHandle node;

  //#####################################PARAMS################################################################### 
  std::string out_file;
  node.param<std::string>("/cpu_calculator/out_file_path", out_file, "cpu_output.txt");
  double frequency; 
  node.param("/cpu_calculator/freq_rate", frequency, 5.0);
  
  //#############################################################################################################

  ROS_INFO("Writing results to file: %s with frequency: %lf", out_file.c_str(), frequency);
  // set the log file
  std::ofstream myfile(out_file.c_str());

  // innit the vars used in the program
  std::string output_text;


  size_t previous_idle_time=0, previous_total_time=0;
  size_t idle_time, total_time;

  // main loop
  ros::Rate rate(frequency);
  while (node.ok()){
    get_cpu_times(idle_time, total_time);
    const float idle_time_delta = idle_time - previous_idle_time;
    const float total_time_delta = total_time - previous_total_time;
    const float utilization = 100.0 * (1.0 - idle_time_delta / total_time_delta);
    std::cout << utilization << '%' << std::endl;
    
    // logging difference and 
    myfile << utilization << "\t";
    myfile << ros::Time::now();
    myfile << std::endl;

    previous_idle_time = idle_time;
    previous_total_time = total_time;
    rate.sleep();
  }


  // for (size_t idle_time, total_time; get_cpu_times(idle_time, total_time); sleep(1)) {
  //     const float idle_time_delta = idle_time - previous_idle_time;
  //     const float total_time_delta = total_time - previous_total_time;
  //     const float utilization = 100.0 * (1.0 - idle_time_delta / total_time_delta);
  //     std::cout << utilization << '%' << std::endl;
  //     previous_idle_time = idle_time;
  //     previous_total_time = total_time;
  // }
  return 0;
}