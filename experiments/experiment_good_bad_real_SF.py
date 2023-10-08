from exp_utils import *
import os
import itertools



def main():
    parent_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))

    dataset_name = "real"
    experiment_name = "good_bad_real_q20"

    if not os.path.isdir(os.path.join(parent_dir, "results_and_plots/ShopFacade", experiment_name)):
        os.makedirs(os.path.join(parent_dir, "results_and_plots/ShopFacade", experiment_name), exist_ok=True)
    
    datasets_folder = os.path.join(parent_dir, "models/ShopFacade/", dataset_name)
    results_folder = os.path.join(parent_dir, "results_and_plots/ShopFacade", experiment_name, "results")
    plots_folder = os.path.join(parent_dir, "results_and_plots/ShopFacade", experiment_name, "plots")    

    # list of [noise_x, noise_y]
    noises = [0.1, 1, 2]

    # # list of [min, max] to set the inlier ratios, [1,1] for this experiment without outliers
    # mins = [0.1, 0.3, 0.5]
    # maxs = [0.7, 0.9, 1.0]
    # inliers_min_max =  list(itertools.product(mins,maxs))
    inliers_min_max = [[0.1, 0.9], [0.3, 0.7], [0.5, 0.7]]

    # other parameters
    solver_type = "0"   # hybrid solver
    num_iters = "100"
    sampler_types = ["2", "1"]  # 2 - inlier sampling, 1 - random sampling
    early_model_rejection = "0"
    experiment = "good_bad"
    experiment_param = "3"
    modify_data = "1"   # always yes in these experiments, since we want to control the inlier ratios and noise levels
    inlier_threshold_2D = "10.0"
    inliers_init = 0    # if 1, then "oracle" experiment is run - the inlier ratios for the cameras is initialized to their true values

    sampler_types_and_inliers_inits = [["2","1"], ["2","0"], ["1","0"]] # oracle hybrid, hybrid, random
    query_size = 1  # must be one of [1, 10, 20, 50, 100, 500, 1000, 2000] - the query is repeated query_size times to get more RANSAC runs

    run_experiments(datasets_folder, results_folder, inliers_min_max, num_iters, sampler_types_and_inliers_inits, 
                        early_model_rejection, experiment, experiment_param, modify_data, solver_type, inlier_threshold_2D, noises, query_size, 32)

    title = "ShopFacade, "
    plot_all([experiment], noises, results_folder, inliers_min_max, plots_folder, [experiment_param], title)


if __name__ == "__main__":
    main()