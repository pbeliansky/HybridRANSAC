from matches_from_model import *
import os





def create_datasets(scene, experiment, save_folder, original_model_folder, list_db_path, list_query_path, pairs_path, inlier_min_max, noises, multiples = [1, 10, 20, 50, 100, 500, 1000, 2000]):
    if not os.path.isdir(save_folder):
        os.makedirs(save_folder, exist_ok=True)
    

    # load or create model
    model_folder = save_folder + "/model"
    if not os.path.isdir(model_folder):
        os.makedirs(model_folder, exist_ok=True) 
        print("...creating dataset")
        
        cameras, images, points3D = read_model(original_model_folder)

        # create model with only inliers
        new_images, new_points3D = get_inlier_images(cameras, images, points3D)

        write_model(cameras, new_images, new_points3D, model_folder, ext=".txt")
    else:
        print("...loading dataset")
        cameras, new_images, new_points3D = read_model(model_folder)



    # load pairs so that they can be used to make matches
    pairs = []
    with open(pairs_path, "r") as f:
        for line in f:
            pairs.append(line.strip().split())
            
    # load queries so that they can be used to make matches
    list_query = []
    with open(list_query_path, "r") as f:
        for line in f:
            list_query.append(line.strip())



    # create matches
    matches_folder = save_folder + "/matches"
    if not os.path.isdir(matches_folder):
        os.makedirs(matches_folder, exist_ok=True) 

    print("...creating matches")
    # creating matches for all noises
    for [noise_x, noise_y] in noises:
        noise_images = add_noise_to_images(new_images, noise_x, noise_y)

        if scene == "planar":
            all_matches = get_pair_matches_modify_pairs_queries(noise_images, pairs, list_query) # removes pairs with bad matches
        else:
            all_matches = get_pair_matches(noise_images, pairs)

        for [min_inl, max_inl] in inlier_min_max:
            if experiment == "dist":
                match_folder = matches_folder + "/dist_inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise_x) + "_" + str(noise_y)
            else:
                match_folder = matches_folder + "/inl_" + str(min_inl) + "_" + str(max_inl) + "_noise_" + str(noise_x) + "_" + str(noise_y)

            if not os.path.isdir(match_folder):
                os.makedirs(match_folder, exist_ok=True)

            if experiment == "dist":
                create_and_save_matches_for_dist_exp(all_matches, noise_images, pairs, match_folder, min_inl, max_inl)
            else:
                create_and_save_matches(all_matches, pairs, match_folder, min_inl, max_inl)
        
        print("   created matches with noise", noise_x, noise_y)


    # save possibly modified pairs
    pf = os.path.join(save_folder, 'pairs.txt')
    np.savetxt(pf, pairs, delimiter=' ', fmt="%s")

    # now we check if the queries are already created
    query_folder = save_folder + "/queries"
    if not os.path.isdir(query_folder):
        os.makedirs(query_folder, exist_ok=True)

    # and finally create the queries:
    print("...creating queries")
    create_and_save_queries(list_query, cameras, new_images, query_folder, multiples)


    




def main():
    parent_dir = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir))

    # The following parameters can be ignored - the evaluation program has been modified and now it can add outliers and noise by itself based on the input parameters

    # list of [noise_x, noise_y]
    noises = [[0, 0]]

    # list of [min, max] to set the inlier ratios, [1,1] for the experiments without outliers
    inliers_min_max = [[1, 1]] 



    #################### create datasets for experiments with a planar scene
    name = "planar"
    save_folder = os.path.join(parent_dir, "models/KingsCollege/", name)    
    original_model_folder = os.path.join(parent_dir, "models/original_models/KingsCollege")
    
    original_model_path = original_model_folder + "/planar_model"
    list_db_path = original_model_folder + "/list_db.txt"
    list_query_path = original_model_folder + "/list_query.txt"
    pairs_path = original_model_folder + "/densevlad_top10.txt"
    
    create_datasets("planar", "", save_folder, original_model_path, list_db_path, list_query_path, pairs_path, inliers_min_max, noises)



    #################### create datasets for experiments with a real scene KingsCollege
    name = "real"
    save_folder = os.path.join(parent_dir, "models/KingsCollege/", name)    
    original_model_folder = os.path.join(parent_dir, "models/original_models/KingsCollege")
    
    original_model_path = original_model_folder + "/retriangulated_model"
    list_db_path = original_model_folder + "/list_db.txt"
    list_query_path = original_model_folder + "/list_query.txt"
    pairs_path = original_model_folder + "/densevlad_top10.txt"
    
    create_datasets("real", "", save_folder, original_model_path, list_db_path, list_query_path, pairs_path, inliers_min_max, noises)
    


    #################### create datasets for experiments with a real scene ShopFacade
    name = "real"
    save_folder = os.path.join(parent_dir, "models/ShopFacade/", name)    
    original_model_folder = os.path.join(parent_dir, "models/original_models/ShopFacade")
    
    original_model_path = original_model_folder + "/retriangulated_model"
    list_db_path = original_model_folder + "/list_db.txt"
    list_query_path = original_model_folder + "/list_query.txt"
    pairs_path = original_model_folder + "/densevlad_top10.txt"
    
    create_datasets("real", "", save_folder, original_model_path, list_db_path, list_query_path, pairs_path, inliers_min_max, noises)


if __name__ == "__main__":
    main()