//#include "minimal_h50_311.h"

#include <chrono> 
#include <vector>
#include <iostream>
#include <fstream>

#include <Eigen/Eigen>


#include "test_sh45_2.h"
#include "test_sh45_3.h"
#include "test_sh5_2.h"
#include "test_sh5_3.h"
#include "test_sh5f_2.h"
#include "test_sh5f_3.h"

#include "util.h"

namespace tests
{

    typedef void (*solver_type)(Eigen::Matrix<double, 3, 5>&, Eigen::Matrix<double, 3, 5>&, Eigen::Matrix<double, 3, 5>&, std::vector<Eigen::Matrix3d>&, std::vector<Eigen::Vector3d>&, std::vector<double>&, double);

    solver_type get_solver(const std::string& solver_name, const std::string& solver_implementation)
    {
        if (solver_name == "sh5_2")
        {
            if (solver_implementation == "paper")
                return &test_sh5_2::run_solver<0>;
            else if (solver_implementation == "companion_matrix")
                return &test_sh5_2::run_solver<1>;
        }
        else if (solver_name == "sh5_3")
        {
            if (solver_implementation == "paper")
                return &test_sh5_3::run_solver<0>;
            else if (solver_implementation == "closed_form")
                return &test_sh5_3::run_solver<1>;
            else if (solver_implementation == "companion_matrix")
                return &test_sh5_3::run_solver<2>;
        }
        else if (solver_name == "sh5f_2")
        {
            if (solver_implementation == "paper")
                return &test_sh5f_2::run_solver<0>;
            else if (solver_implementation == "companion_matrix")
                return &test_sh5f_2::run_solver<1>;
        }
        else if (solver_name == "sh5f_3")
        {
            if (solver_implementation == "paper")
                return &test_sh5f_3::run_solver<0>;
            else if (solver_implementation == "closed_form")
                return &test_sh5f_3::run_solver<1>;
            else if (solver_implementation == "companion_matrix")
                return &test_sh5f_3::run_solver<2>;
        }
        else if (solver_name == "sh45_2")
        {
            if (solver_implementation == "paper")
                return &test_sh45_2::run_solver<0>;
            else if (solver_implementation == "optimized")
                return &test_sh45_2::run_solver<1>;
            else if (solver_implementation == "danilevsky")
                return &test_sh45_2::run_solver<2>;
        }
        else if (solver_name == "sh45_3")
        {
            if (solver_implementation == "paper")
                return &test_sh45_3::run_solver<0>;
            else if (solver_implementation == "optimized")
                return &test_sh45_3::run_solver<1>;
            else if (solver_implementation == "danilevsky")
                return &test_sh45_3::run_solver<2>;
        }

        throw std::domain_error("Unknown solver");
        return nullptr;
    }

    /// <summary>
    /// Total number of measured runs will be dataset_size * iterations
    /// </summary>
    void run_test(const std::string& solver_name, const std::string& solver_implementation, size_t dataset_size, size_t warmup_iterations, size_t iterations, std::ostream& out_stream, std::ostream& out_stream_brief, double scale = 10.0)
    {
        std::vector<qpc> rand_qpc(dataset_size);
        fill_with_random(rand_qpc, dataset_size, scale);

        std::vector<Eigen::Matrix3d> Hs(200);//could be much less
        std::vector<Eigen::Vector3d> Nss(200);
        std::vector<double> fss(200);
        double solver_tol = 1e-20;

        solver_type solver = get_solver(solver_name, solver_implementation);

        // warmup
        for (size_t j = 0; j < dataset_size; ++j)
            for (size_t i = 0; i < warmup_iterations; ++i)
                solver(rand_qpc[j].q, rand_qpc[j].p, rand_qpc[j].c, Hs, Nss, fss, solver_tol);

        // measuring time of "dataset_size * iterations" runs
        auto start = std::chrono::high_resolution_clock::now();

        for (size_t j = 0; j < dataset_size; ++j)
            for (size_t i = 0; i < iterations; ++i)
                solver(rand_qpc[j].q, rand_qpc[j].p, rand_qpc[j].c, Hs, Nss, fss, solver_tol);

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);

        out_stream << "\n// " << dataset_size * iterations << " runs of " << solver_name << "_" << solver_implementation << " takes " << std::chrono::duration_cast<std::chrono::seconds>(duration).count() << " seconds\n";
        out_stream << "// one iteration takes on average " << (double)duration.count() / (dataset_size * iterations) << " microseconds\n";

        out_stream_brief << solver_name << " " << solver_implementation << " " << dataset_size * iterations << " " << (double)duration.count() / (dataset_size * iterations) << "\n";
    }
}


int main(int argc, char** argv)
{
    using namespace tests;

    size_t dat_size = 1000;
    size_t w_iters = 10;
    size_t iters = 10000;
    double scale = 10;
    std::string output_file_name = "tests_output.txt";

    std::cout << " usage: " << argv[0] << "   dataset_size   warmup_iterations   iterations   scale   output_file_name" << std::endl; // 1000 10 10000

    if (argc >= 4)
    {
        dat_size = atoi(argv[1]);
        w_iters = atoi(argv[2]);
        iters = atoi(argv[3]);
    }
    if (argc >= 5)
    {
        scale = atof(argv[4]);
    }
    if (argc == 6)
    {
        output_file_name = atof(argv[5]);
    }

    std::ofstream out_file(output_file_name, std::ios::out | std::ios::trunc);

    out_file << "# solver_name    solver_implementation    dataset_size * iterations    time in microseconds\n";

    run_test("sh5_2", "paper", dat_size, w_iters, iters, std::cout, out_file, scale);
    run_test("sh5_2", "companion_matrix", dat_size, w_iters, iters, std::cout, out_file, scale);

    run_test("sh5_3", "paper", dat_size, w_iters, iters, std::cout, out_file, scale);
    run_test("sh5_3", "closed_form", dat_size, w_iters, iters, std::cout, out_file, scale);
    run_test("sh5_3", "companion_matrix", dat_size, w_iters, iters, std::cout, out_file, scale);

    run_test("sh5f_2", "paper", dat_size, w_iters, iters, std::cout, out_file, scale);
    run_test("sh5f_2", "companion_matrix", dat_size, w_iters, iters, std::cout, out_file, scale);

    run_test("sh5f_3", "paper", dat_size, w_iters, iters, std::cout, out_file, scale);
    run_test("sh5f_3", "closed_form", dat_size, w_iters, iters, std::cout, out_file, scale);
    run_test("sh5f_3", "companion_matrix", dat_size, w_iters, iters, std::cout, out_file, scale);

    run_test("sh45_2", "paper", dat_size, w_iters, iters, std::cout, out_file, scale);
    run_test("sh45_2", "optimized", dat_size, w_iters, iters, std::cout, out_file, scale);
    run_test("sh45_2", "danilevsky", dat_size, w_iters, iters, std::cout, out_file, scale);

    run_test("sh45_3", "paper", dat_size, w_iters, iters, std::cout, out_file, scale);
    run_test("sh45_3", "optimized", dat_size, w_iters, iters, std::cout, out_file, scale);
    run_test("sh45_3", "danilevsky", dat_size, w_iters, iters, std::cout, out_file, scale);

    return 0;
}










