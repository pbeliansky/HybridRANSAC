#ifndef RANSACLIB_RANSACLIB_DATA_HANDLING_H_
#define RANSACLIB_RANSACLIB_DATA_HANDLING_H_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <limits>
#include <random>
#include <regex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector>


#include <ransac_solvers/ransac_solvers_common.h>
#include <hybrid_ransac_eval/eval_utils.h>



namespace ransac_lib {
    namespace utils
    {
        // At the moment, we are ignoring every model that is too complicated.
        struct ColmapCamera {
            std::string camera_model;
            int camera_id;
            int width;
            int height;
            std::vector<double> parameters;
        };

        struct ColmapObservation {
            Eigen::Vector2d x;
            int point_id;
        };

        struct ColmapImage {
            std::string image_name;
            int image_id;
            int camera_id;
            Eigen::Quaterniond q;
            Eigen::Vector3d t;
            std::vector<ColmapObservation> observations;
        };

        struct ColmapTrack {
            int image_id;
            int feature_id;
        };

        struct ColmapPoint {
            int point_id;
            Eigen::Vector3d X;
            Eigen::Vector3i color;
            double error;
            std::vector<ColmapTrack> track;
        };

        struct QueryData {
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
                std::string name;

            ColmapCamera camera;

            Eigen::Quaterniond q;
            Eigen::Vector3d t;
        };

        typedef std::vector<QueryData, Eigen::aligned_allocator<QueryData>> Queries;

        typedef std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>> Points2D;
        typedef std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>> Points3D;

        using pairs_u_map = std::unordered_map<std::string, std::vector<std::string>>;

        inline void LoadCamera(const std::string& line, bool load_camera_id,
            ColmapCamera* camera) {
            std::stringstream s_stream(line);

            ColmapCamera& cam = *camera;
            if (load_camera_id) s_stream >> cam.camera_id;
            s_stream >> cam.camera_model >> cam.width >> cam.height;
            if (cam.camera_model.compare("SIMPLE_RADIAL") == 0 ||
                cam.camera_model.compare("VSFM") == 0 ||
                cam.camera_model.compare("PINHOLE") == 0) {
                cam.parameters.resize(4);
                s_stream >> cam.parameters[0] >> cam.parameters[1] >> cam.parameters[2]
                    >> cam.parameters[3];
            }
            else if (cam.camera_model.compare("SIMPLE_PINHOLE") == 0) {
                cam.parameters.resize(3);
                s_stream >> cam.parameters[0] >> cam.parameters[1] >> cam.parameters[2];
            }
            else {
                std::cout << " camera model " << cam.camera_model << " not supported" << std::endl;
                exit(EXIT_FAILURE);
            }
        }

        inline bool LoadColmapCamera(const std::string& filename,
            std::vector<ColmapCamera>* cameras) {
            cameras->clear();
            std::ifstream ifs(filename.c_str(), std::ios::in);
            if (!ifs.is_open()) {
                std::cout << " cannot read from " << filename << std::endl;
                return false;
            }
            std::string line;

            // Skips the first three lines containing the header.
            for (int i = 0; i < 3; ++i) {
                std::getline(ifs, line);
            }

            while (std::getline(ifs, line)) {
                ColmapCamera cam;
                LoadCamera(line, true, &cam);
                cameras->push_back(cam);
            }

            ifs.close();
            return true;
        }

        inline bool LoadColmapImages(const std::string& filename,
            std::vector<ColmapImage>* images) {
            images->clear();
            std::ifstream ifs(filename.c_str(), std::ios::in);
            if (!ifs.is_open()) {
                std::cout << " cannot read from " << filename << std::endl;
                return false;
            }
            std::string line;

            // Skips the first four lines containing the header.
            for (int i = 0; i < 4; ++i) {
                std::getline(ifs, line);
            }

            int count = 0;
            while (std::getline(ifs, line)) {
                std::stringstream s_stream(line);

                ColmapImage img;
                s_stream >> img.image_id >> img.q.w() >> img.q.x() >> img.q.y() >> img.q.z()
                    >> img.t[0] >> img.t[1] >> img.t[2] >> img.camera_id
                    >> img.image_name;
                img.q.normalize();
                {
                    // Skips feature associations.
                    std::getline(ifs, line);
                    // std::stringstream s_stream2(line);
                    // while (!s_stream2.eof()) {
                    //   ColmapObservation obs;
                    //   s_stream2 >> obs.x[0] >> obs.x[1] >> obs.point_id;
                    //   if (obs.point_id != -1) {
                    //     img.observations.push_back(obs); 
                    //   }
                    // }
                }

                images->push_back(img);
                ++count;
            }

            ifs.close();
            return true;
        }

        template <typename T>
        inline double ComputeMedian(std::vector<T>* data, T* mean) {
            *mean = static_cast<T>(0.0);
            for (size_t i = 0; i < data->size(); ++i) {
                *mean += (*data)[i];
            }
            *mean /= static_cast<T>(data->size());

            std::sort(data->begin(), data->end());
            if (data->size() % 2u == 1u) {
                return static_cast<double>((*data)[data->size() / 2]);
            }
            else {
                double a = static_cast<double>((*data)[data->size() / 2 - 1]);
                double b = static_cast<double>((*data)[data->size() / 2]);
                return (a + b) * 0.5;
            }
        }


        // Loads the list of query images together with their extrinsics and intrinsics.
        inline bool LoadQueries(const std::string& filename,
            Queries* query_images) {
            std::ifstream ifs(filename.c_str(), std::ios::in);
            if (!ifs.is_open()) {
                std::cerr << " ERROR: Cannot read the image list from " << filename
                    << std::endl;
                return false;
            }
            std::string line;

            query_images->clear();

            while (std::getline(ifs, line)) {
                std::stringstream s_stream(line);

                QueryData q;
                s_stream >> q.name;
                s_stream >> q.camera.camera_model >> q.camera.width >> q.camera.height;
                if (q.camera.camera_model.compare("SIMPLE_RADIAL") == 0 ||
                    q.camera.camera_model.compare("VSFM") == 0 ||
                    q.camera.camera_model.compare("PINHOLE") == 0) {
                    q.camera.parameters.resize(4);
                    s_stream >> q.camera.parameters[0] >> q.camera.parameters[1]
                        >> q.camera.parameters[2] >> q.camera.parameters[3];
                }
                else {
                    std::cout << " camera model " << q.camera.camera_model << " not supported" << std::endl;
                    exit(EXIT_FAILURE);
                }
                s_stream >> q.q.w() >> q.q.x() >> q.q.y() >> q.q.z() >> q.t[0] >> q.t[1] >> q.t[2];
                q.q.normalize();
                // std::string line;
                // std::getline(s_stream, line);
                // LoadCamera(line, false, &(q.camera));
                query_images->push_back(q);
            }

            ifs.close();

            return true;
        }



        // Loads 2D-2D matches for two given image names.
        inline bool LoadMatches2D2D(const std::string& match_dir, const std::string& q_name,
            const std::string& db_name,
            Points2D* points2D_q, Points2D* points2D_ref) {
            points2D_q->clear();
            points2D_ref->clear();

            std::stringstream s_stream;
            s_stream << "matches_" << q_name << "_" << db_name << ".txt";
            std::string ending(s_stream.str());
            std::string filename(match_dir);
            filename.append(std::regex_replace(ending, std::regex("/"), "_"));

            std::ifstream ifs(filename.c_str(), std::ios::in);
            if (!ifs.is_open()) {
                std::cerr << " ERROR: Cannot read the matches from " << filename
                    << std::endl;
                return false;
            }
            std::string line;

            while (std::getline(ifs, line)) {
                std::stringstream s_stream(line);

                Eigen::Vector2d p_q;
                Eigen::Vector2d p_db;
                s_stream >> p_q[0] >> p_q[1] >> p_db[0] >> p_db[1];

                points2D_q->push_back(p_q);
                points2D_ref->push_back(p_db);
            }

            return true;
        }


        inline bool LoadPairs(const std::string& filename,
            std::unordered_map<std::string, std::vector<std::string>>* pairs) {
            pairs->clear();

            std::ifstream ifs(filename.c_str(), std::ios::in);
            if (!ifs.is_open()) {
                std::cerr << " ERROR: Cannot read the matches from " << filename
                    << std::endl;
                return false;
            }
            std::string line;

            while (std::getline(ifs, line)) {
                std::stringstream s_stream(line);

                std::string q_name, db_name;
                s_stream >> q_name >> db_name;

                (*pairs)[q_name].push_back(db_name);
            }

            return true;
        }


        // Prepare the keypoints.
        inline void GetKeypoints(Eigen::Vector2d& pp_q, Eigen::Vector2d& pp_db, double& q_fy, const QueryData& query, const ColmapCamera& camera)
        {
            if (query.camera.camera_model.compare("PINHOLE") == 0) {
                q_fy = query.camera.parameters[1];
                pp_q << query.camera.parameters[2],
                    query.camera.parameters[3];
            }
            else {
                pp_q << query.camera.parameters[1],
                    query.camera.parameters[2];
            }
            if (camera.camera_model.compare("PINHOLE") == 0) {
                pp_db << camera.parameters[2], camera.parameters[3];
            }
            else {
                pp_db << camera.parameters[1], camera.parameters[2];
            }
        }

        inline void GetCameraDistances(std::vector<size_t>& dist_indexes, const std::vector<std::string>& pairs_i, const std::string& q_name,
            std::unordered_map<std::string, int>& map_image_name_to_idx, std::unordered_map<int, int>& map_cam_id_to_idx,
            std::vector<ColmapImage>& images, solvers::Cameras& h_cameras)
        {
            const int queryImgIdx = map_image_name_to_idx[q_name];

            std::vector<double> dists_from_query;
            for (const std::string& p : pairs_i)
            {
                const int kImgIdx = map_image_name_to_idx[p];
                const int kCamIdx = map_cam_id_to_idx[images[kImgIdx].camera_id];

                double dist = (h_cameras[queryImgIdx].c - h_cameras[kImgIdx].c).norm();
                dists_from_query.push_back(dist);
            }

            // sort indexes
            dist_indexes.resize(dists_from_query.size());
            std::iota(dist_indexes.begin(), dist_indexes.end(), 0);
            std::stable_sort(dist_indexes.begin(), dist_indexes.end(), [&dists_from_query](size_t a, size_t b) { return dists_from_query[a] < dists_from_query[b]; });

        }

        template< class Generator >
        inline void RandomizePoint(Eigen::Vector2d& point, std::uniform_real_distribution<double>& w_distribution, std::uniform_real_distribution<double>& h_distribution, 
            Generator& g, double min_dist = 75.0)
        {
            Eigen::Vector2d new_point;
            while (true)
            {
                new_point[0] = w_distribution(g);
                new_point[1] = h_distribution(g);

                double dist = (new_point - point).norm();

                if (dist >= min_dist)
                {
                    point = new_point;
                    break;
                }
            }
        }

        template< class Generator >
        inline void AddNoise(Eigen::Vector2d& point, std::normal_distribution<double>& noise_distribution, Generator& g)
        {
            point[0] += noise_distribution(g);
            point[1] += noise_distribution(g);
        }

        // set inlier ratios to corresponding values and add noise
        template< class Generator >
        inline void ModifyMatches(solvers::Matches2D2DWithCamID& matches2D2D, double noise_px, double inlier_ratio, Generator& g,
            Points2D& points2d_q, Points2D& points2d_db, int w = 800, int h = 450)//,
            //const QueryData& query, std::unordered_map<std::string, int>& map_image_name_to_idx)
        {
            std::uniform_real_distribution<double> inl_distribution(0.0, 1.0);
            std::normal_distribution<double> noise_distribution(0.0, noise_px / 2);
            std::uniform_real_distribution<double> w_distribution(0.0, w);
            std::uniform_real_distribution<double> h_distribution(0.0, h);


            // add noise to the points in the query image
            for (size_t i = 0; i < points2d_q.size(); ++i)
                AddNoise(points2d_q[i], noise_distribution, g);

            // make outlier match or add noise to the points in db images
            for (size_t i = 0; i < points2d_db.size(); ++i)
            {
                // with probability inlier_ratio keep inlier and add noise, otherwise make db point an outlier
                double number = inl_distribution(g);
                if (number <= inlier_ratio)
                    AddNoise(points2d_db[i], noise_distribution, g);
                else
                    RandomizePoint(points2d_db[i], w_distribution, h_distribution, g);
            }
        }

        inline void PrepareMatches(std::vector<std::pair<int, int>>& match_ranges, solvers::Matches2D2DWithCamID& matches2D2D, std::vector<int>& selected_images,
            Points2D& points2d_q, Points2D& points2d_db, std::vector<std::pair< Eigen::Vector2d, Eigen::Vector2d >> &points2D_q_db, 
            const QueryData& query, ColmapCamera& camera, solvers::Camera& h_camera,
            double& q_fx, double& q_fy, double kErrorThresh, const int kImgIdx, int& num_valid)
        {
            match_ranges.push_back(std::make_pair(
                static_cast<int>(matches2D2D.size()), 0));


            const int kNumMatches = static_cast<int>(points2d_q.size());

            // Prepare the keypoints.
            Eigen::Vector2d pp_q, pp_db;
            GetKeypoints(pp_q, pp_db, q_fy, query, camera);

            double threshold = kErrorThresh * kErrorThresh;
            Eigen::Matrix3d R_gt(query.q);

            for (int j = 0; j < kNumMatches; ++j)
            {
                solvers::Match2D2DWithCamID m;
                m.p2D_query = points2d_q[j] - pp_q;
                m.p2D_query[0] /= q_fx; 
                m.p2D_query[1] /= q_fy;
                m.p2D_db = points2d_db[j] - pp_db;
                m.ref_ray_dir = h_camera.R.transpose() * h_camera.K.inverse() * m.p2D_db.homogeneous();
                m.ref_ray_dir.normalize();
                m.camera_id = kImgIdx;

                matches2D2D.push_back(m);
                points2D_q_db.emplace_back(points2d_q[j], points2d_db[j]);
            }
            match_ranges.back().second = static_cast<int>(matches2D2D.size() - 1);

            if (kNumMatches > 0) ++num_valid;
        }


        inline int LoadData(Queries& query_data, std::vector<ColmapImage>& images, std::unordered_map<std::string, int>& map_image_name_to_idx, std::unordered_map<int, int>& map_cam_id_to_idx, pairs_u_map& pairs, solvers::Cameras& h_cameras,
            std::vector<ColmapCamera>& cameras, char** argv)
        {
            {
                std::string image_file(argv[2]);
                image_file.append("images.txt");
                if (!LoadColmapImages(image_file, &images)) {
                    std::cerr << " ERROR: Cannot load images from " << image_file << std::endl;
                    return -1;
                }
            }
        
            std::unordered_map<int, int> map_image_id_to_idx;
            
            std::vector<Eigen::Vector3d> camera_centers(images.size());
            for (int i = 0; i < static_cast<int>(images.size()); ++i) {
                map_image_id_to_idx[images[i].image_id] = i;
                map_image_name_to_idx[images[i].image_name] = i;
                camera_centers[i] = -Eigen::Matrix3d(images[i].q).transpose() * images[i].t;
            }
        
            
            {
                std::string cam_file(argv[2]);
                cam_file.append("cameras.txt");
                if (!LoadColmapCamera(cam_file, &cameras)) {
                    std::cerr << " ERROR: Cannot load cameras from " << cam_file << std::endl;
                    return -1;
                }
            }
            
            for (int i = 0; i < static_cast<int>(cameras.size()); ++i) {
                map_cam_id_to_idx[cameras[i].camera_id] = i;
                double scaling = 800.0 / static_cast<double>(std::max(cameras[i].width, cameras[i].height));
                scaling = 1.0;
                cameras[i].width = static_cast<int>(static_cast<double>(cameras[i].width) * scaling);
                cameras[i].height = static_cast<int>(static_cast<double>(cameras[i].height) * scaling);
                // Assuming SIMPLE_RADIAL or VSFM
        
                cameras[i].parameters[0] *= scaling;
                cameras[i].parameters[1] *= scaling;
                cameras[i].parameters[2] *= scaling;
                cameras[i].parameters[3] *= scaling;
                // We don't scale the distortion parameter as we are ignoring it.
            }
        
            //   std::cout << "  -> done loading COLMAP data" << std::endl;
        
            std::string pairs_file(argv[4]);
            
            if (!LoadPairs(pairs_file, &pairs)) {
                std::cerr << " ERROR: Could not read the pairs from " << pairs_file
                    << std::endl;
                return -1;
            }        
            
            std::string list(argv[1]);
        
            if (!LoadQueries(list, &query_data)) {
                std::cerr << " ERROR: Could not read the data from " << list << std::endl;
                return -1;
            }
        
        
            h_cameras = solvers::Cameras(images.size());
            for (size_t i = 0; i < images.size(); ++i) {
                h_cameras[i].R = images[i].q;
                h_cameras[i].c = -h_cameras[i].R.transpose() * images[i].t;
                h_cameras[i].K = Eigen::Matrix3d::Identity();
                int cam_id = map_cam_id_to_idx[images[i].camera_id];
                h_cameras[i].K(0, 0) = cameras[cam_id].parameters[0];
                if (cameras[cam_id].camera_model.compare("PINHOLE") == 0) {
                    h_cameras[i].K(1, 1) = cameras[cam_id].parameters[1];
                }
                else {
                    h_cameras[i].K(1, 1) = h_cameras[i].K(0, 0);
                }
            }
            return 0;
        } 
    }  // namespace utils
}  // namespace ransac_lib

#endif  // RANSACLIB_RANSACLIB_DATA_HANDLING_H_