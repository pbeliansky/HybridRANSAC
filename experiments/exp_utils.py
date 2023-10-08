import os

from plot_ransac_scores import *

import subprocess as sub    
import errno

def plot_all(experiments, noises, results_folder, inliers_min_max, plots_folder, experiment_params, title, to=100):
    for experiment in experiments:
            if experiment == "dist":
                exp_params = [0, 4, 8] #list(range(9))
            else:
                exp_params = experiment_params
            for n in noises:
                for experiment_param in exp_params:
                    plot_3_experiments(results_folder, inliers_min_max, plots_folder, n, experiment, experiment_param, title, to)

                if experiment == "dist" or experiment == "dist_uniform":
                    plot_3_dist_experiments(results_folder, inliers_min_max, plots_folder, n, experiment, title, to)


def get_plotting_strings(experiment, min_inl, max_inl, noise, experiment_param, results_folder, save_folder):
    if experiment == "dist":
        order_suf = "-th"
        if str(experiment_param) == "1":
            order_suf = "-st"
        elif str(experiment_param) == "2":
            order_suf = "-nd"
        elif str(experiment_param) == "3":
            order_suf = "-rd"

        hybrid_res_dir = results_folder + "/inlier/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_" + str(experiment_param) + "/res"
        random_res_dir = results_folder + "/random/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_" + str(experiment_param) + "/res"
        oracle_hybrid_res_dir = results_folder + "/inlieroracle/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_" + str(experiment_param) + "/res"
        save_dir = save_folder + "/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_" + str(experiment_param)
        figure_name = "inlier ratio " + str(max_inl) + " for " + str(experiment_param) + order_suf + " two cameras, " + str(min_inl) + " for others, "+str(noise)+"px noise"
        inliers_name = "inlier ratios " + str(max_inl) + " & " + str(min_inl) + ", "+str(noise)+"px noise"

    elif experiment == "uniform":
        hybrid_res_dir = results_folder + "/inlier/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "/res"
        random_res_dir = results_folder + "/random/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "/res"
        oracle_hybrid_res_dir = results_folder + "/inlieroracle/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "/res"
        save_dir = save_folder + "/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise)
        figure_name = "inlier ratios " + str(min_inl) + "-" + str(max_inl) + " and "+str(noise)+"px noise"
        inliers_name = "inlier ratios " + str(min_inl) + " - " + str(max_inl) + ", "+str(noise)+"px noise"

    elif experiment == "good_bad":
        hybrid_res_dir = results_folder + "/inlier/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "/res"
        random_res_dir = results_folder + "/random/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "/res"
        oracle_hybrid_res_dir = results_folder + "/inlieroracle/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "/res"
        save_dir = save_folder + "/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise)
        figure_name = "inlier ratio " + str(max_inl) + " for " + experiment_param + " cameras, " + str(min_inl) + " for others, "+str(noise)+"px noise"
        inliers_name = "inlier ratios " + str(max_inl) + " & " + str(min_inl) + ", "+str(noise)+"px noise"

    elif experiment == "dist_uniform":
        if str(experiment_param) == "0":
            order = "ascending"
        elif str(experiment_param) == "1":
            order = "descending"

        hybrid_res_dir = results_folder + "/inlier/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_uniform_" + str(experiment_param) + "/res"
        random_res_dir = results_folder + "/random/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_uniform_" + str(experiment_param) + "/res"
        oracle_hybrid_res_dir = results_folder + "/inlieroracle/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_uniform_" + str(experiment_param) + "/res"
        save_dir = save_folder + "/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_uniform_" + str(experiment_param)
        figure_name = "inlier ratios " + str(min_inl) + "-" + str(max_inl) + " in " + order + " order by distance, "+str(noise)+"px noise"
        inliers_name = "inlier ratios " + str(min_inl) + " - " + str(max_inl) + ", "+str(noise)+"px noise"
    # else throw

    return hybrid_res_dir, random_res_dir, save_dir, oracle_hybrid_res_dir, inliers_name, figure_name


def plot_experiments(results_folder, inlier_min_max, save_folder, noise, experiment, experiment_param):

    if not os.path.isdir(save_folder):
        os.makedirs(save_folder, exist_ok=True)


    for [min_inl, max_inl] in inlier_min_max:
        fr = 0

        hybrid_res_dir, random_res_dir, save_dir, oracle_hybrid_res_dir, inliers_name, figure_name = get_plotting_strings(experiment, min_inl, max_inl, noise, experiment_param, results_folder, save_folder)            

        if not os.path.isdir(save_dir):
            os.makedirs(save_dir, exist_ok=True)

        try:
            # scores
            plot_scores(hybrid_res_dir, random_res_dir, "Mean and median Ransac scores for " + inliers_name, save_dir, True, True, figure_name+"_score", fr)
            plot_scores(hybrid_res_dir, random_res_dir, "Mean Ransac scores for " + inliers_name, save_dir, True, False, figure_name+"_score_mean", fr)
            plot_scores(hybrid_res_dir, random_res_dir, "Median Ransac scores for " + inliers_name, save_dir, False, True, figure_name+"_score_median", fr)

            # orientations
            plot_scores(hybrid_res_dir, random_res_dir, "Mean orientation errors for " + inliers_name, save_dir, True, False, figure_name+"_orientation_mean", fr,
                        "_mean_orientation_errors_.txt",
                        "_median_orientation_errors_.txt",
                        "_mean_orientation_errors_.txt",
                        "_median_orientation_errors_.txt",
                        'orientation error')
            plot_scores(hybrid_res_dir, random_res_dir, "Median orientation errors for " + inliers_name, save_dir, False, True, figure_name+"_orientation_median", fr,
                        "_mean_orientation_errors_.txt",
                        "_median_orientation_errors_.txt",
                        "_mean_orientation_errors_.txt",
                        "_median_orientation_errors_.txt",
                        'orientation error')

            # positions
            plot_scores(hybrid_res_dir, random_res_dir, "Mean position errors for " + inliers_name, save_dir, True, False, figure_name+"_position_mean", fr,
                        "_mean_position_errors_.txt",
                        "_median_position_errors_.txt",
                        "_mean_position_errors_.txt",
                        "_median_position_errors_.txt",
                        'position error')
            plot_scores(hybrid_res_dir, random_res_dir, "Median position errors for " + inliers_name, save_dir, False, True, figure_name+"_position_median", fr,
                        "_mean_position_errors_.txt",
                        "_median_position_errors_.txt",
                        "_mean_position_errors_.txt",
                        "_median_position_errors_.txt",
                        'position error')
        except:
            print(hybrid_res_dir, random_res_dir, " failed")



                    

def plot_3_experiments(results_folder, inlier_min_max, save_folder, noise, experiment, experiment_param, title, to):

    if not os.path.isdir(save_folder):
        os.makedirs(save_folder, exist_ok=True)


    for [min_inl, max_inl] in inlier_min_max:

        hybrid_res_dir, random_res_dir, save_dir, oracle_hybrid_res_dir, inliers_name, figure_name = get_plotting_strings(experiment, min_inl, max_inl, noise, experiment_param, results_folder, save_folder)    
            

        if not os.path.isdir(save_dir):
            os.makedirs(save_dir, exist_ok=True)


        try:
            # scores
            plot_3_scores(hybrid_res_dir, random_res_dir, oracle_hybrid_res_dir, title + inliers_name, save_dir, True, True, figure_name+"_score", to,
                        "_mean_scores_.txt",
                        "_median_scores_.txt",
                        "_mean_scores_.txt",
                        "_median_scores_.txt",
                        'Best model score')
            plot_3_scores(hybrid_res_dir, random_res_dir, oracle_hybrid_res_dir, title + inliers_name, save_dir, True, False, figure_name+"_score_mean", to,
                        "_mean_scores_.txt",
                        "_median_scores_.txt",
                        "_mean_scores_.txt",
                        "_median_scores_.txt",
                        'Mean best model score')
            plot_3_scores(hybrid_res_dir, random_res_dir, oracle_hybrid_res_dir, title + inliers_name, save_dir, False, True, figure_name+"_score_median", to,
                        "_mean_scores_.txt",
                        "_median_scores_.txt",
                        "_mean_scores_.txt",
                        "_median_scores_.txt",
                        'Median best model score')


            # orientations
            plot_3_scores(hybrid_res_dir, random_res_dir, oracle_hybrid_res_dir, title + inliers_name, save_dir, True, False, figure_name+"_orientation_mean", to,
                        "_mean_orientation_errors_.txt",
                        "_median_orientation_errors_.txt",
                        "_mean_orientation_errors_.txt",
                        "_median_orientation_errors_.txt",
                        'Mean orientation error')
            plot_3_scores(hybrid_res_dir, random_res_dir, oracle_hybrid_res_dir, title + inliers_name, save_dir, False, True, figure_name+"_orientation_median", to,
                        "_mean_orientation_errors_.txt",
                        "_median_orientation_errors_.txt",
                        "_mean_orientation_errors_.txt",
                        "_median_orientation_errors_.txt",
                        'Median orientation error')

            # positions
            plot_3_scores(hybrid_res_dir, random_res_dir, oracle_hybrid_res_dir, title + inliers_name, save_dir, True, False, figure_name+"_position_mean", to,
                        "_mean_position_errors_.txt",
                        "_median_position_errors_.txt",
                        "_mean_position_errors_.txt",
                        "_median_position_errors_.txt",
                        'Mean position error')
            plot_3_scores(hybrid_res_dir, random_res_dir, oracle_hybrid_res_dir, title + inliers_name, save_dir, False, True, figure_name+"_position_median", to,
                        "_mean_position_errors_.txt",
                        "_median_position_errors_.txt",
                        "_mean_position_errors_.txt",
                        "_median_position_errors_.txt",
                        'Median position error')
        except:
            print(hybrid_res_dir, random_res_dir, oracle_hybrid_res_dir, " failed")







def plot_3_dist_experiments(results_folder, inlier_min_max, save_folder, noise, experiment, title, to):

    if not os.path.isdir(save_folder):
        os.makedirs(save_folder, exist_ok=True)


    for [min_inl, max_inl] in inlier_min_max:

        if experiment == "dist":
            hybrid_res_dir = results_folder + "/inlier/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_"
            random_res_dir = results_folder + "/random/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_"
            oracle_hybrid_res_dir = results_folder + "/inlieroracle/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_"
            save_dir = save_folder + "/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist"
            figure_name = "inlier ratio " + str(max_inl) + " for i-th two cameras, " + str(min_inl) + " for others, "+str(noise)+"px noise"
            inliers_name = "inlier ratios " + str(max_inl) + " & " + str(min_inl) + ", "+str(noise)+"px noise"
            N = 9
        elif experiment == "dist_uniform":
            hybrid_res_dir = results_folder + "/inlier/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_uniform_"
            random_res_dir = results_folder + "/random/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_uniform_"
            oracle_hybrid_res_dir = results_folder + "/inlieroracle/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_uniform_"
            save_dir = save_folder + "/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_uniform"
            figure_name = "inlier ratios " + str(min_inl) + "-" + str(max_inl) + " in ascending(0) or descending(1) order by distance, "+str(noise)+"px noise"
            inliers_name = "inlier ratios " + str(min_inl) + " - " + str(max_inl) + ", "+str(noise)+"px noise"
            N = 2

        if not os.path.isdir(save_dir):
            os.makedirs(save_dir, exist_ok=True)


        try:
            # scores
            plot_3_dist_scores(hybrid_res_dir, random_res_dir, oracle_hybrid_res_dir, title + inliers_name, N, save_dir, True, True, figure_name+"_score", to,
                        "_mean_scores_.txt",
                        "_median_scores_.txt",
                        "_mean_scores_.txt",
                        "_median_scores_.txt",
                        'Best model score')
            plot_3_dist_scores(hybrid_res_dir, random_res_dir, oracle_hybrid_res_dir, title + inliers_name, N, save_dir, True, False, figure_name+"_score_mean", to,
                        "_mean_scores_.txt",
                        "_median_scores_.txt",
                        "_mean_scores_.txt",
                        "_median_scores_.txt",
                        'Mean best model score')
            plot_3_dist_scores(hybrid_res_dir, random_res_dir, oracle_hybrid_res_dir, title + inliers_name, N, save_dir, False, True, figure_name+"_score_median", to,
                        "_mean_scores_.txt",
                        "_median_scores_.txt",
                        "_mean_scores_.txt",
                        "_median_scores_.txt",
                        'Median best model score')


            # orientations
            plot_3_dist_scores(hybrid_res_dir, random_res_dir, oracle_hybrid_res_dir, title + inliers_name, N, save_dir, True, False, figure_name+"_orientation_mean", to,
                        "_mean_orientation_errors_.txt",
                        "_median_orientation_errors_.txt",
                        "_mean_orientation_errors_.txt",
                        "_median_orientation_errors_.txt",
                        'Mean orientation error')
            plot_3_dist_scores(hybrid_res_dir, random_res_dir, oracle_hybrid_res_dir, title + inliers_name, N, save_dir, False, True, figure_name+"_orientation_median", to,
                        "_mean_orientation_errors_.txt",
                        "_median_orientation_errors_.txt",
                        "_mean_orientation_errors_.txt",
                        "_median_orientation_errors_.txt",
                        'Median orientation error')

            # positions
            plot_3_dist_scores(hybrid_res_dir, random_res_dir, oracle_hybrid_res_dir, title + inliers_name, N, save_dir, True, False, figure_name+"_position_mean", to,
                        "_mean_position_errors_.txt",
                        "_median_position_errors_.txt",
                        "_mean_position_errors_.txt",
                        "_median_position_errors_.txt",
                        'Mean position error')
            plot_3_dist_scores(hybrid_res_dir, random_res_dir, oracle_hybrid_res_dir, title + inliers_name, N, save_dir, False, True, figure_name+"_position_median", to,
                        "_mean_position_errors_.txt",
                        "_median_position_errors_.txt",
                        "_mean_position_errors_.txt",
                        "_median_position_errors_.txt",
                        'Median position error')
        except:
            print(hybrid_res_dir, random_res_dir, oracle_hybrid_res_dir, " failed")



#*
def get_saving_strings(sampler_type, inliers_init, experiment, min_inl, max_inl, noise, experiment_param,
                       datasets_save_folder, results_save_folder):
    if sampler_type == "1" or sampler_type == 1:
        sampling_strategy = "random"
    elif sampler_type == "2" or sampler_type == 2:
        sampling_strategy = "inlier"
    else:
        sampling_strategy = ""
        
    if inliers_init == "1" or inliers_init == 1:
        sampling_strategy += "oracle"

    results_save_folder = os.path.join(results_save_folder, sampling_strategy)
    if not os.path.isdir(results_save_folder):
        try:
            os.makedirs(results_save_folder, exist_ok=True)
        except OSError as exc:
            if exc.errno != errno.EEXIST:
                raise
            pass

    if experiment == "dist":
        match_dir =  datasets_save_folder + "/matches/dist_inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_" + str(experiment_param)
        out_dir = results_save_folder + "/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_" + str(experiment_param)
    elif experiment == "dist_uniform":
        match_dir = datasets_save_folder + "/matches/dist_uniform_inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_uniform_" + str(experiment_param)
        out_dir = results_save_folder + "/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) + "_dist_uniform_" + str(experiment_param)
    else:
        match_dir = datasets_save_folder + "/matches/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise)
        out_dir = results_save_folder + "/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise) + "_" + str(noise) 
    
    match_dir =  datasets_save_folder + "/matches/inl_1_1_noise_0_0/"

    return sampling_strategy, match_dir, out_dir


#*
def run(datasets_save_folder, results_save_folder, min_inl, max_inl, executable, query_list, colmap_dir, pairs_file, inlier_threshold_2D, 
            solver_type, num_iters, sampler_type, early_model_rejection, experiment, experiment_param, noise, modify_data, inliers_init):
    print("run ", min_inl, max_inl)

    sampling_strategy, match_dir, out_dir = get_saving_strings(sampler_type, inliers_init, experiment, min_inl, max_inl, noise, experiment_param,
                       datasets_save_folder, results_save_folder)

    if not os.path.isdir(out_dir):
        try:
            os.makedirs(out_dir, exist_ok=True)
        except OSError as exc:
            if exc.errno != errno.EEXIST:
                raise
            pass
        # return       # maybe don't run if exists
    out_file = out_dir + "/res"

    print("running    ", executable, query_list, colmap_dir, match_dir, pairs_file, inlier_threshold_2D, out_file, 
                solver_type, num_iters, sampler_type, early_model_rejection, experiment, experiment_param, min_inl, max_inl, noise, modify_data, inliers_init)

    p = sub.Popen([executable, query_list, colmap_dir, match_dir, pairs_file, inlier_threshold_2D, out_file, 
                solver_type, num_iters, sampler_type, early_model_rejection, experiment, experiment_param, min_inl, max_inl, noise, modify_data, inliers_init])
    while True:
        ret = p.wait()
        if ret:   # anything other than zero will evaluate to True here
            print(" failed")
            print(p.stdin)
            print(p.stdout)
            print(p.stderr)
            print()
            break#p = sub.Popen([executable, query_list, colmap_dir, match_dir, pairs_file, inlier_threshold_2D, out_file, solver_type, num_iters])
        else:
            print("done ", min_inl, max_inl, noise)
            break

    print("done ", min_inl, max_inl)





#*
def run_experiments(datasets_save_folder, results_save_folder, inlier_min_max, num_iters, sampler_types_and_inliers_inits, 
                    early_model_rejection, experiment, experiment_param, modify_data, solver_type, inlier_threshold_2D, noises, q_size = 20, max_pool_size=36):
    if not os.path.isdir(results_save_folder):
        os.makedirs(results_save_folder, exist_ok=True)

    exp_dir = os.path.dirname(os.path.realpath(__file__))
    executable = os.path.join(exp_dir, os.pardir, "SemiGeneralizedHomography/build/hybrid_ransac_eval/eval_localization_calibrated")
    query_list = datasets_save_folder + "/queries/queries" + str(q_size) + ".txt"
    colmap_dir = datasets_save_folder + "/model/"
    pairs_file = datasets_save_folder + "/pairs.txt"
    
    if experiment == "dist":
        params = [0, 4, 8]#list(range(9))
    elif experiment == "dist_uniform":
        params = ["0", "1"]
    else:
        params = [experiment_param]

    from multiprocessing import Pool

    jobs = len(params) * len(noises) * len(inlier_min_max) * len(sampler_types_and_inliers_inits)

    with Pool(min(max_pool_size, jobs)) as p:
        p.starmap(run, [(datasets_save_folder, results_save_folder, str(min_inl), str(max_inl), executable, query_list, colmap_dir, pairs_file, inlier_threshold_2D, 
                            solver_type, num_iters, str(sampler_type), early_model_rejection, experiment, str(expar), str(noise), modify_data, str(inliers_init)) 
                            for [sampler_type, inliers_init] in sampler_types_and_inliers_inits for [min_inl, max_inl] in inlier_min_max for noise in noises for expar in params])