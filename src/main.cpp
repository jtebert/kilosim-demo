#include "BayesBot.cpp"
#include "ProgressBar.hpp"

#include <math.h>
#include <iomanip>
#include <sstream>
#include <kilosim/World.h>
#include <kilosim/ConfigParser.h>
#include <kilosim/Viewer.h>
#include <kilosim/Logger.h>

// AGGREGATORS

std::vector<double> robot_light_count(std::vector<Kilosim::Robot *> &robots)
{
    // Pull each robot's count of light color observations
    std::vector<double> light_counts(robots.size());
    for (int i = 0; i < robots.size(); i++)
    {
        // Downcast to get to custom variables
        Kilosim::BayesBot *bb = (Kilosim::BayesBot *)robots[i];
        light_counts[i] = bb->light_count;
    }
    return light_counts;
}

std::vector<double> robot_dark_count(std::vector<Kilosim::Robot *> &robots)
{
    // Pull each robot's count of dark color observations
    std::vector<double> dark_counts(robots.size());
    for (int i = 0; i < robots.size(); i++)
    {
        // Downcast to get to custom variables
        Kilosim::BayesBot *bb = (Kilosim::BayesBot *)robots[i];
        dark_counts[i] = bb->dark_count;
    }
    return dark_counts;
}

std::vector<double> robot_decision(std::vector<Kilosim::Robot *> &robots)
{
    // Pull each robot's decision
    std::vector<double> decisions(robots.size());
    for (int i = 0; i < robots.size(); i++)
    {
        // Downcast to get to custom variables
        Kilosim::BayesBot *bb = (Kilosim::BayesBot *)robots[i];
        decisions[i] = bb->decision;
    }
    return decisions;
}

std::vector<double> robot_observation_count(std::vector<Kilosim::Robot *> &robots)
{
    // Pull each robot's count of light color observations
    std::vector<double> observation_counts(robots.size());
    for (int i = 0; i < robots.size(); i++)
    {
        // Downcast to get to custom variables
        Kilosim::BayesBot *bb = (Kilosim::BayesBot *)robots[i];
        observation_counts[i] = bb->observation_ind;
    }
    return observation_counts;
}

bool all_robots_decided(std::vector<Kilosim::BayesBot *> &robots)
{
    // Add the ability to stop the simulation early if all the robots
    // decided before the end
    for (int i = 0; i < robots.size(); i++)
    {
        if (robots[i]->decision == -1)
            return false;
    }
    return true;
}

// HACKY STUFF

nlohmann::json get_val(Kilosim::ConfigParser config, std::string key,
                       std::string compare_param, int compare_ind)
{
    // Hacky approach to getting the indexed value if this is the compare_param,
    // or otherwise just returning the scalar value
    if (compare_param == key)
    {
        return config.get(key)[compare_ind];
    }
    else
    {
        return config.get(key);
    }
}

// MAIN STUFF

int main(int argc, char *argv[])
{
    // To avoid showing a bunch of slow/unnecessary progress bars on AWS
    bool show_progress = false;

    // Get config file name
    std::vector<std::string> args(argv, argv + argc);
    if (args.size() < 2)
    {
        std::cout << "ERROR: You must provide a config file name" << std::endl;
        exit(1);
    }
    Kilosim::ConfigParser config(args[1]);

    // Get configuration values
    const std::string compare_param = config.get("compare_param");
    nlohmann::json compare_vals = config.get(compare_param);
    const uint num_compare_vals = compare_vals.size();

    const uint start_trial = config.get("start_trial");
    const uint num_trials = config.get("num_trials");
    const double trial_duration = config.get("trial_duration"); // seconds
    const std::string light_img_src = config.get("light_img_src");
    const std::string log_dir = config.get("log_dir");
    const std::vector<double> fill_ratios = config.get("fill_ratios");
    const double world_width = config.get("world_width");
    const double world_height = config.get("world_height");
    const unsigned long seed_base = config.get("seed_base");

    // Constant values used for configuring initial robot positions
    const double grid_cover = 0.8; // cover 90% of width/height
    const double x_pos_offset = world_width * (1.0 - grid_cover) / 2;
    const double y_pos_offset = world_height * (1.0 - grid_cover) / 2;

    // Length of time covered by a single dot in the progress bar
    const int progress_update_freq = 60;
    const int limit = trial_duration / progress_update_freq;

    // Loop through the parameter under investigation (compare_param)
    for (uint compare_ind = 0; compare_ind < num_compare_vals; compare_ind++)
    {

        // These are things that can possibly be varied between conditions
        uint use_positive_feedback = get_val(config, "use_positive_feedback", compare_param, compare_ind);
        uint log_freq = get_val(config, "log_freq", compare_param, compare_ind); // seconds
        uint num_robots = get_val(config, "num_robots", compare_param, compare_ind);
        double credible_thresh = get_val(config, "credible_thresh", compare_param, compare_ind);
        uint allow_simultaneity = get_val(config, "allow_simultaneity", compare_param, compare_ind);
        uint observe_step_time = get_val(config, "observe_step_time", compare_param, compare_ind); // seconds
        uint both_prior = get_val(config, "both_prior", compare_param, compare_ind);
        // uint dark_prior = get_val(config, "dark_prior", compare_param, compare_ind);
        // uint light_prior = get_val(config, "light_prior", compare_param, compare_ind);
        json compare_val = config.get(compare_param)[compare_ind];

        // uint log_freq = config.get("log_freq"); // kiloticks
        // uint num_robots = config.get("num_robots");
        // double credible_thresh = config.get("credible_thresh");
        // uint allow_simultaneity = config.get("allow_simultaneity");
        // uint observe_step_time = config.get("observe_step_time");
        // std::vector<uint> dark_prior = config.get("dark_prior");
        // uint light_prior = config.get("light_prior");

        // Computed values for initializing robot positions
        // (This goes here because num_robots is possibly changeable)
        uint num_rows = ceil(sqrt(num_robots));
        double x_spacing = grid_cover * world_width / num_rows;
        double y_spacing = grid_cover * world_height / num_rows;

        // Loop through the fill ratios
        for (int f_ind = 0; f_ind < fill_ratios.size(); f_ind++)
        {

            seed_rand(seed_base); // For random number configuration

            // Set fill ratio and logging file name
            double fill_ratio = fill_ratios[f_ind];
            std::ostringstream ss;
            ss << std::fixed << std::setprecision(2) << fill_ratio;
            std::string fill_ratio_str = ss.str();
            std::ostringstream cs;
            cs << compare_val;
            std::string compare_val_str = cs.str();
            //std::vector<std::string> compare_vals = config.get(compare_param);
            //std::string compare_val = compare_vals[0];
            std::string log_filename = log_dir + compare_param + '=' + compare_val_str + '-' + fill_ratio_str + ".h5";

            // Run all trials for this parameter set
            for (uint trial = start_trial; trial < (num_trials + start_trial); trial++)
            {
                if (show_progress)
                {
                    printf("\n\n");
                    printf("-------------------------------------------------------\n");
                    std::cout << "    TRIAL " << trial << "    [" << fill_ratio << "]    " << compare_param << " = " << compare_val << std::endl;
                    printf("-------------------------------------------------------\n");
                }
                ProgressBar progress_bar(limit, 50);

                // Configure light image filename
                std::string light_img_filename = light_img_src + "rect-" + fill_ratio_str + "-" + std::to_string(trial) + ".png";

                // Initialize World (and Viewer)
                Kilosim::World world(
                    world_width,
                    world_height,
                    light_img_filename);

                Kilosim::Viewer viewer(world);

                // Create robots and initialize in grid
                std::vector<Kilosim::BayesBot *> robots(num_robots);
                for (int n = 0; n < num_robots; n++)
                {
                    // Set any implementation-specific config that comes from config file
                    robots[n] = new Kilosim::BayesBot();
                    robots[n]->credible_thresh = credible_thresh;
                    robots[n]->allow_simultaneity = allow_simultaneity;
                    robots[n]->use_positive_feedback = use_positive_feedback;
                    robots[n]->observe_step_time = observe_step_time;
                    robots[n]->dark_prior = both_prior;
                    robots[n]->light_prior = both_prior;
                    world.add_robot(robots[n]);
                    // robots[n]->robot_init(n * 50 + 100, 100, PI / 2);
                    robots[n]->robot_init((n / num_rows + 0.5) * x_spacing + x_pos_offset,
                                          (n % num_rows + 0.5) * y_spacing + y_pos_offset,
                                          uniform_rand_real(0, 2 * PI));
                }
                // Verify that robots are within World bounds and not overlapping
                world.check_validity();

                // Set up logging
                Kilosim::Logger logger(
                    world,
                    log_filename,
                    trial,
                    false);
                logger.add_aggregator("light_count", robot_light_count);
                logger.add_aggregator("dark_count", robot_dark_count);
                logger.add_aggregator("decision", robot_decision);
                logger.add_aggregator("observation_count", robot_observation_count);
                logger.log_config(config, false);
                // Log the fill_ratio separately because it's not in the config
                logger.log_param("fill_ratio", fill_ratio);
                // Add logging of compare_param
                logger.log_param(compare_param, compare_val);

                while (world.get_time() < trial_duration)
                {
                    // Run a simulation step
                    // This automatically increments the tick
                    world.step();

                    viewer.draw();

                    if ((world.get_tick() % (log_freq * world.get_tick_rate())) == 0)
                    {
                        // Log the state of the world every 5 seconds
                        // This works because the tickRate (ticks/sec) must be an integer
                        logger.log_state();

                        // End trial early if all of the robots have decided
                        // And only allow this after the decisions have been logged!
                        if (all_robots_decided(robots))
                            break;
                    }
                    if (show_progress && (world.get_tick() % (progress_update_freq * world.get_tick_rate())) == 0)
                    {
                        ++progress_bar;
                        progress_bar.display();
                    }
                }

                // Print out statistics when trial is finished.
                if (show_progress)
                {
                    progress_bar.done();
                }
                int acc_decision = 0;
                int undecided_count = 0;
                for (int i = 0; i < robots.size(); i++)
                {
                    if (robots[i]->decision == -1)
                        undecided_count++;
                    else
                        acc_decision += robots[i]->decision;
                }
                double decision_accuracy = (double)acc_decision / robots.size();
                int time = world.get_time();
                std::cout << "Simulated duration:\t"
                          << std::setfill('0') << std::setw(2) << (int)(time / 3600) << ":"
                          << std::setfill('0') << std::setw(2) << (int)((time % 3600) / 60) << ":"
                          << std::setfill('0') << std::setw(2) << time % 60 << std::endl;
                std::cout << "Decision accuracy:\t" << decision_accuracy * 100 << "%" << std::endl;
                std::cout << "Undecided robots:\t" << undecided_count << "/" << robots.size() << std::endl;
            }
        }
    }

    printf("\n\nSimulations complete\n\n");

    return 0;
}
