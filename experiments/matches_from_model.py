from hashlib import new

from soupsieve import match
from read_write_model import *
import numpy as np
import random
import os
import copy


def get_K(camera):
    if camera.model == 'SIMPLE_RADIAL':
        [f,cx,cy,_] = camera.params
        return np.array([[f, 0, cx], 
                        [0, f, cy], 
                        [0, 0, 1]])

def project_point(K, R, t, xyz, nx, ny):
    proj = K @ np.hstack((R, t[:, np.newaxis])) @ np.concatenate([xyz, [1]])

    noise_x = (random.random() - 0.5) * nx
    noise_y = (random.random() - 0.5) * ny
    
    return np.array([proj[0]/proj[2] + noise_x, proj[1]/proj[2] + noise_y])


def add_noise_to_images(images, nx, ny):
    noise_images =  copy.deepcopy(images)
    if nx == 0 and ny == 0:
        return noise_images

    for im in noise_images:
        for idx in range(len(noise_images[im].xys)):
            noise_x = (random.random() - 0.5) * nx
            noise_y = (random.random() - 0.5) * ny
            noise_images[im].xys[idx][0] += noise_x
            noise_images[im].xys[idx][1] += noise_y

    return noise_images


# camera fields:  ('id', 'model', 'width', 'height', 'params')
# image fields:  ('id', 'qvec', 'tvec', 'camera_id', 'name', 'xys', 'point3D_ids')
# point fields:  ('id', 'xyz', 'rgb', 'error', 'image_ids', 'point2D_idxs')
def get_inlier_images(cameras, images, points3D, noise_x=0, noise_y=0):


    new_images = {}
    new_points3D = points3D.copy()

    for img_id, val in images.items():
        K = get_K(cameras[val.camera_id])
        R = qvec2rotmat(val.qvec)
        t = val.tvec

        new_xys = []
        new_point3D_ids = []

        for index, id3D in enumerate(val.point3D_ids):
            if id3D != -1 and id3D in new_points3D:
                new_point3D_ids.append(id3D)
                index_in_3D = np.where(new_points3D[id3D].image_ids == img_id)[0][0]

                new_points3D[id3D].point2D_idxs[index_in_3D] = len(new_xys)

                new_xys.append(project_point(K, R, t, new_points3D[id3D].xyz, noise_x, noise_y))
                
        new_images[img_id] = BaseImage(img_id, val.qvec, val.tvec, val.camera_id, val.name, np.array(new_xys), np.array(new_point3D_ids))
        
    return new_images, new_points3D


# used for planar scene, because moving the points to a plane can move matching points outside of the images
def get_pair_matches_modify_pairs_queries(images, pairs, list_query):
    n_matches = []
    kept_matches = []
    qx = []
    qy = []
    dbx = []
    dby = []

    img_name_to_id = {}
    for i in images.values():
        img_name_to_id[i.name] = i.id

    all_matches = {}
    copy_pairs = pairs.copy()
    for [q,db] in copy_pairs:
        q_id = img_name_to_id[q]
        db_id = img_name_to_id[db]

        points_seen_from_q = set(images[q_id].point3D_ids)
        points_seen_from_db = set(images[db_id].point3D_ids)

        matching_points = points_seen_from_q & points_seen_from_db

        n_matches.append(len(matching_points))
        kept_matches.append(0)

        # this approach is not very smart, but it was a quick way to make the experiment
        for point3D_id in matching_points:
            # find index of the point in image.xys 
            index_q = np.where(images[q_id].point3D_ids == point3D_id)[0][0]
            index_db = np.where(images[db_id].point3D_ids == point3D_id)[0][0]
            
            
            qx.append(images[q_id].xys[index_q][0])
            qy.append(images[q_id].xys[index_q][1])
            dbx.append(images[db_id].xys[index_db][0])
            dby.append(images[db_id].xys[index_db][1])

            if 0 < qx[-1] < 800 and 0 < qy[-1] < 450 and 0 < dbx[-1] < 800 and 0 < dby[-1] < 450:   # hardcoded image size
                kept_matches[-1] += 1

        if kept_matches[-1] < 500:
            pairs.remove([q,db])
            if q in list_query:
                list_query.remove(q)
            continue


        matches = []
        for point3D_id in matching_points:
            # find index of the point in image.xys 
            index_q = np.where(images[q_id].point3D_ids == point3D_id)[0][0]
            index_db = np.where(images[db_id].point3D_ids == point3D_id)[0][0]            
            
            q_x = images[q_id].xys[index_q][0]
            q_y = images[q_id].xys[index_q][1]
            db_x = images[db_id].xys[index_db][0]
            db_y = images[db_id].xys[index_db][1]

            if 0 < q_x < 800 and 0 < q_y < 450 and 0 < db_x < 800 and 0 < db_y < 450:
                s = [images[q_id].xys[index_q], images[db_id].xys[index_db]]
                matches.append(s)

        
        match_name = "matches_" + q.replace("/", "_") + "_" + db.replace("/", "_" ) + ".txt"
        
        all_matches[match_name] = matches
    
    return all_matches



def get_pair_matches(images, pairs):
    img_name_to_id = {}
    for i in images.values():
        img_name_to_id[i.name] = i.id

    all_matches = {}
    for [q,db] in pairs:
        q_id = img_name_to_id[q]
        db_id = img_name_to_id[db]

        points_seen_from_q = set(images[q_id].point3D_ids)
        points_seen_from_db = set(images[db_id].point3D_ids)

        matching_points = points_seen_from_q & points_seen_from_db

        matches = []
        for point3D_id in matching_points:
            # find index of the point in image.xys 
            index_q = np.where(images[q_id].point3D_ids == point3D_id)[0][0]
            index_db = np.where(images[db_id].point3D_ids == point3D_id)[0][0]


            try:
                s = [images[q_id].xys[index_q], images[db_id].xys[index_db]]
            except:
                print("q_id", q_id)
                print("len(new_images[q_id].xys)", len(images[q_id].xys))
                print("index_q", index_q)
                print("db_id", db_id)
                print("len(new_images[db_id].xys)", len(images[db_id].xys))
                print("index_db", index_db)
                return
            
            matches.append(s)

        
        match_name = "matches_" + q.replace("/", "_") + "_" + db.replace("/", "_" ) + ".txt"
        
        all_matches[match_name] = matches

    return all_matches




def get_all_matches(images, points3D):
    all_matches = {}

    nn = len(points3D)
    cur = 0

    for pt_id, point3D in points3D.items():
        cur += 1
        if cur % 5000 == 0:
            print(cur, "/", nn)

        for ind1, q_id in enumerate(point3D.image_ids):
            index_q = point3D.point2D_idxs[ind1]
            for ind2, db_id in enumerate(point3D.image_ids):
                index_db = point3D.point2D_idxs[ind2]
                
                try:
                    s = [images[q_id].xys[index_q], images[db_id].xys[index_db]]
                except:
                    print("q_id", q_id)
                    print("len(new_images[q_id].xys)", len(images[q_id].xys))
                    print("index_q", index_q)
                    print("db_id", db_id)
                    print("len(new_images[db_id].xys)", len(images[db_id].xys))
                    print("index_db", index_db)
                    return
                
                match_name = "matches_" + images[q_id].name.replace("/", "_") + "_" + images[db_id].name.replace("/", "_" ) + ".txt"
                
                if match_name in all_matches:
                    all_matches[match_name].append(s)
                else:
                    all_matches[match_name] = [s]

    return all_matches


def randomize_match(match): # better solution is to generate random point until it's far enough
    min_d = 10#pixels
    dev = 200#pixels
    new_match = copy.deepcopy(match)

    new_match[1][0] += (random.random()*2 - 1) * dev + min_d
    new_match[1][1] += (random.random()*2 - 1) * dev + min_d

    return new_match


def add_outliers(matches, inl_ratio):
    out_matches = []
    for m in matches:
        if random.random() > inl_ratio:
            out_matches.append(np.concatenate(randomize_match(m)))
        else:
            out_matches.append(np.concatenate(m))
    return out_matches



def create_and_save_queries(list_query, cameras, images, folder, multiples):
    img_name_to_id = {}
    for i in images.values():
        img_name_to_id[i.name] = i.id

    queries = []
    for query in list_query:
        q_id = img_name_to_id[query]

        R = qvec2rotmat(images[q_id].qvec)
        t = images[q_id].tvec
        c = -R.transpose() @ t

        queries.append([query, cameras[images[q_id].camera_id].model,
           cameras[images[q_id].camera_id].width,
           cameras[images[q_id].camera_id].height,
           cameras[images[q_id].camera_id].params[0],
           cameras[images[q_id].camera_id].params[1],
           cameras[images[q_id].camera_id].params[2],
           cameras[images[q_id].camera_id].params[3],
           images[q_id].qvec[0],
           images[q_id].qvec[1],
           images[q_id].qvec[2],
           images[q_id].qvec[3],
           c[0],
           c[1],
           c[2]])
    
    for m in multiples:
        q = []
        q.extend(queries * m)

        qf = os.path.join(folder, 'queries' + str(m) + '.txt')
        np.savetxt(qf, q, delimiter=' ', fmt="%s")    



def create_and_save_matches_for_dist_exp(all_matches, images, pairs, folder, min_inl, max_inl):
    from operator import itemgetter
    img_name_to_id = {}
    for i in images.values():
        img_name_to_id[i.name] = i.id

    pairs_dic = {}
    for [q,db] in pairs:
        if q not in pairs_dic:
            pairs_dic[q] = [db]
        else:
            pairs_dic[q].append(db)


    miss = 0
    count = 0

    for query in pairs_dic:
        db_num = len(pairs_dic[query])
        inlier_ratios = np.full((db_num), min_inl)
        inlier_ratios[0]

        q_c = -qvec2rotmat(images[img_name_to_id[query]].qvec).T * images[img_name_to_id[query]].tvec

        sorted_by_dist = []
        for i, db in enumerate(pairs_dic[query]):
            db_c = -qvec2rotmat(images[img_name_to_id[db]].qvec).T * images[img_name_to_id[db]].tvec            
            dist = np.linalg.norm(q_c - db_c)
            sorted_by_dist.append((db, dist))

        sorted_by_dist = sorted(sorted_by_dist,key=itemgetter(1))

        for dist_ord in range(len(sorted_by_dist)):
            inlier_ratios = np.full((db_num), min_inl)
            inlier_ratios[dist_ord] = max_inl

            match_folder = folder+'_dist_'+str(dist_ord)
                    
            if not os.path.isdir(match_folder):
                os.mkdir(match_folder)

            for i, (db, dist) in enumerate(sorted_by_dist):
                inlier_ratios = np.full((db_num), min_inl)
                inlier_ratios[i] = max_inl

                count += 1

                filename = "matches_" + query.replace("/", "_") + "_" + db.replace("/", "_" ) + ".txt"

                if filename not in all_matches:
                    miss += 1
                else:
                        outfile = os.path.join(match_folder, filename)                        

                        tmp_matches = add_outliers(all_matches[filename], inlier_ratios[i])
                        np.savetxt(outfile, tmp_matches, delimiter=' ')          
        
    print("missed", miss,"out of",  count)





def create_and_save_matches(all_matches, pairs, folder, min_inl, max_inl):

    pairs_dic = {}
    for [q,db] in pairs:
        if q not in pairs_dic:
            pairs_dic[q] = [db]
        else:
            pairs_dic[q].append(db)

    miss = 0
    count = 0

    for query in pairs_dic:
        db_num = len(pairs_dic[query])
        inlier_ratios = np.linspace(min_inl, max_inl, db_num)
        np.random.shuffle(inlier_ratios)
            
        for i, db in enumerate(pairs_dic[query]):
            count += 1

            filename = "matches_" + query.replace("/", "_") + "_" + db.replace("/", "_" ) + ".txt"

            if filename not in all_matches:
                miss += 1
            else:
                    outfile = os.path.join(folder, filename)

                    tmp_matches = add_outliers(all_matches[filename], inlier_ratios[i])
                    np.savetxt(outfile, tmp_matches, delimiter=' ')          
    
    print("missed", miss,"out of",  count)



def create_and_save_matches_pairs_queries(cameras, images, points3D, list_query, list_db, folder):
    all_matches = get_all_matches(images, points3D)
    
    img_name_to_id = {}
    for i in images.values():
        img_name_to_id[i.name] = i.id

    pairs = []
    queries = []

    miss = 0
    count = 0
    for query in list_query:
        q_id = img_name_to_id[query]

        R = qvec2rotmat(images[q_id].qvec)
        t = images[q_id].tvec
        c = -R.transpose() @ t

        queries.append([query, cameras[images[q_id].camera_id].model,
           cameras[images[q_id].camera_id].width,
           cameras[images[q_id].camera_id].height,
           cameras[images[q_id].camera_id].params[0],
           cameras[images[q_id].camera_id].params[1],
           cameras[images[q_id].camera_id].params[2],
           cameras[images[q_id].camera_id].params[3],
           images[q_id].qvec[0],
           images[q_id].qvec[1],
           images[q_id].qvec[2],
           images[q_id].qvec[3],
           c[0],
           c[1],
           c[2]])
            
        for db in list_db:
            count += 1

            #probability that this camera will have inliers
            p = 0.5
            inl_ratio = 0.98
            
            rand = random.random()
            if rand <= 0.3:
                inl_ratio = 0.85
            elif rand <= 0.5:
                inl_ratio = 0.70
            elif rand <= 0.6:
                inl_ratio = 0.55
            

            filename = "matches_" + query.replace("/", "_") + "_" + db.replace("/", "_" ) + ".txt"

            if filename not in all_matches:
                miss += 1
            else:
                if len(all_matches[filename]) < 20:
                    miss += 1
                else:
                    pairs.append([query, db])
                    outfile = os.path.join(folder, "matches", filename)
                    np.savetxt(outfile, add_outliers(all_matches[filename], inl_ratio), delimiter=' ')
    
    print(miss, count)
    
    pf = os.path.join(folder, 'pairs.txt')
    np.savetxt(pf, pairs, delimiter=' ', fmt="%s")

    qf = os.path.join(folder, 'queries.txt')
    np.savetxt(qf, queries, delimiter=' ', fmt="%s")



def main():
    cameras, images, points3D = read_model("ShopFacade\model_train")

    new_images, new_points3D = get_inlier_images(cameras, images, points3D, 1, 1)

    folder = "generated_models\\ShopFacade\\mixinliers"
    modelfolder = "generated_models\\ShopFacade\\mixinliers\\model"

    write_model(cameras, new_images, new_points3D, modelfolder, ext=".txt")

    # create queries only from known db images
    list_db = []
    with open("ShopFacade\\list_db.txt", "r") as f:
        for line in f:
            list_db.append(line.strip())
    
    np.random.shuffle(list_db)
    list_query, list_db = list_db[:int(len(list_db)/3)], list_db[int(len(list_db)/3):]

    create_and_save_matches_pairs_queries(cameras, new_images, new_points3D, list_query, list_db, folder)



if __name__ == "__main__":
    main()