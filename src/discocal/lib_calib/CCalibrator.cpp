#include "CCalibrator.h"
#include <cstdlib>
#include <chrono>

// Opt-in phase timing (DISC_TIME=1). With nothing set, no output is produced and
// the numerics are untouched -- only the wall clock of each stage is observed.
static bool disc_time_on(){
    static const bool on = []{
        const char *s = std::getenv("DISC_TIME");
        return s && std::atoi(s) != 0;
    }();
    return on;
}
static std::chrono::steady_clock::time_point disc_mark(){ return std::chrono::steady_clock::now(); }
static void disc_phase(const char *name, std::chrono::steady_clock::time_point t0){
    if(!disc_time_on()) return;
    double s = std::chrono::duration<double>(std::chrono::steady_clock::now()-t0).count();
    printf("[time] %-24s %8.2f s\n", name, s);
}

// Block-diagonal uncertainty accumulation (DISC_BLOCK_COV=1) instead of the
// dense n_row x n_row M_cov. Same arithmetic, but the dense matrix and its
// O(n_row^3) inverse are never formed -- see DETECTION_ROBUSTNESS.md 3.1.
static bool disc_block_cov(){
    static const bool on = []{
        const char *s = std::getenv("DISC_BLOCK_COV");
        return s && std::atoi(s) != 0;
    }();
    return on;
}

// Full-precision sigma dump (DISC_UNC_DUMP=1). The yaml carries only ~6
// significant digits and the 2sigma table 3 decimals, both too coarse to prove
// the block-diagonal path reproduces the dense one -- agreement at digit 6 would
// look the same if the whitening were subtly wrong. This prints the same numbers
// at 17 digits so an A/B diff is a real test.
static bool disc_unc_dump(){
    static const bool on = []{
        const char *s = std::getenv("DISC_UNC_DUMP");
        return s && std::atoi(s) != 0;
    }();
    return on;
}

// Sparse (Tier 2) uncertainty accumulation (DISC_SPARSE_INFO=1): take J' W J
// straight from the Ceres CRS jacobian, one blob at a time, instead of building
// the dense J/J_real and sweeping them. Supersedes DISC_BLOCK_COV (it whitens
// the same way) and is linear in the frame count -- see
// DETECTION_ROBUSTNESS.md 3.4.
static bool disc_sparse_info(){
    static const bool on = []{
        const char *s = std::getenv("DISC_SPARSE_INFO");
        return s && std::atoi(s) != 0;
    }();
    return on;
}
// Ceres worker threads (DISC_CERES_THREADS). 0/unset keeps Ceres' default,
// which is the number of hardware threads; the node pins this to 1 by default
// so a solve does not compete with the detector on the same cores.
static int disc_ceres_threads(){
    static const int n = []{
        const char *s = std::getenv("DISC_CERES_THREADS");
        return s ? std::atoi(s) : 0;
    }();
    return n > 0 ? n : 0;  // 0 = leave Ceres' own default
}

// Linear-solver choice. The default DENSE_SCHUR builds a dense Schur complement
// whose size grows with the number of pose blocks, so memory is ~O(n_frames^2)
// and time ~O(n_frames^3): 48 frames solve in seconds, 275 frames need >6 GB and
// do not finish. SPARSE_SCHUR is algebraically the same problem and scales
// linearly in the frame count. Selectable so the stock path stays byte-identical.
static ceres::LinearSolverType disc_linear_solver(){
    const char *s = std::getenv("DISC_SPARSE_SCHUR");
    return (s && std::atoi(s) != 0) ? ceres::SPARSE_SCHUR : ceres::DENSE_SCHUR;
}

Calibrator::Calibrator(int n_x, int n_y, bool asymmetric, int n_d, double r,double distance, int max_scene, string results_path){
    this->n_x = n_x;
    this->n_y = n_y;
    this->is_asymmetric = asymmetric;
    this->n_d = n_d;
    this->max_scene = max_scene;
    this->num_scene = 0;
    this->thr_homography = 0;
    if(endsWith(results_path, "/")) this-> root_dir = results_path;
    else this-> root_dir= results_path+"/";


    this->original_r = r;
    this->distance = distance;
    this->width = 0;
    this->height = 0;

    // origin_conics.reserve(n_x*n_y);
    origin_target.reserve(n_x*n_y);
    targets.reserve(max_scene);
    ud_targets.reserve(max_scene);
    Hs.reserve(max_scene);
    Es.reserve(max_scene);

    set_origin_target();
}
int Calibrator::get_num_scene(){
    return this->num_scene;
}

void Calibrator::init(){
    //Todo: for continous calibration
    // targets.clear();
    Es.clear();
    // curr_r= original_r;
    // for(int i=0;i<num_scene;i++){
    //     rs[i]=original_r;
    // }
    // Hs.clear();
}

void Calibrator::set_image_size(int _width, int _height){
    this->width = _width;
    this->height = _height;
}

void Calibrator::set_origin_target(){
    if(is_asymmetric){
        for(int i=0;i<n_x;i++){
            double offset =0;
            if(i%2==0) offset= distance/2.0;
            for(int j=0;j<n_y;j++){
                double x,y;
                x = distance*i/2.0;
                y = distance*j+offset;
                Shape target(x,y, M_PI*original_r*original_r);
                // target.radius = original_r;
                origin_target.push_back(target);
            }
        }
    }
    else{
        for(int j=0;j<n_y;j++){
            for(int i=0;i<n_x;i++){
                double x,y;
                x = distance*(i+1);
                y = distance*(n_y-j);
                Shape target(x,y, M_PI*original_r*original_r);
                // target.radius = original_r;
                origin_target.push_back(target);
            }
        }
    }


    ori_ms = get_mean_sigma(origin_target);
}


void Calibrator::inputTarget(vector<Shape> target){
    // vector<Point> target2;
    // int n = target.size();
    // target2.reserve(n);
    // for(int i=0;i<n;i++){
    //     target2.push_back(Point(target[i].x,target[i].y));
    // }
    // targets.push_back(target2);
    // Hs.push_back(get_H(target2));

    targets.push_back(target);
    Hs.push_back(get_H(target));
    num_scene++;
    // printf("%dth scene stored\n",num_scene);
}

void Calibrator::printParams(Params params){
    cout<<"camera parameters:"<<endl;;
    cout<< params.to_string_detailed()<<endl;
}

void Calibrator::set_inital_Es(Params params){
    Eigen::Matrix3d K, E, R;
    
    K << params.fx(), params.skew(), params.cx(),
        0,  params.fy(),  params.cy(),
        0,  0,  1;

    if(K.determinant()==0){
        printf("intrinsic matrix should be invertable\n");
        Eigen::Vector3d rot{M_PI,0,0}, trans{0,0,1.0};
        for(int i=0; i<num_scene;i++){
            Es.push_back(se3(rot, trans));
        }
    }
    else{
        Eigen::Vector3d rot, trans, R1, R2, R3;
        for(int i=0; i<num_scene;i++){
            E = K.inverse()* Hs.at(i).first;
            Es.push_back(LieAlgebra::to_se3(E));
        }
    }
    normalize_Es();
    
}

Params Calibrator::batch_optimize(std::vector<int> sample, Params initial_params, int projection_mode, bool save_jacob, bool fix_intrinsic){

    // double fcs[5]={initial_params.fx, initial_params.fy, initial_params.cx, initial_params.cy, initial_params.skew};
    // double distorsion[4]={initial_params.d[0],initial_params.d[1],initial_params.d[2],initial_params.d[3]};
    Params results = initial_params;

    bool use_weight= true;
    Problem problem; 
    ceres::LossFunction* loss_function;
    loss_function = new ceres::TrivialLoss();

    for(int i=0;i<sample.size();i++){
        int index =sample.at(i);
        int num_residual= targets.at(index).size()*2;

        CostFunction* cost_function = new NumericDiffCostFunction<CalibrationFunctor,ceres::CENTRAL, ceres::DYNAMIC,5,4,3,3>(
            new CalibrationFunctor(origin_target,targets[index],original_r,n_d,projection_mode,use_weight), ceres::TAKE_OWNERSHIP, num_residual
        );

        problem.AddResidualBlock(cost_function, loss_function, results.K, results.d, Es.at(index).rot.data(),Es.at(index).trans.data() );

    }
    if(fix_intrinsic){ 
        problem.SetParameterBlockConstant(results.K);
        problem.SetParameterBlockConstant(results.d);
    }

    
    
    // Run the solver!
    Solver::Options options;
    // printf("original iter : %d\n", options.max_num_iterations);
    // options.max_num_iterations=200;

    // options.line_search_direction_type=ceres::LineSearchDirectionType::BFGS;
    // options.function_tolerance=1.0e-8;
    // options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.linear_solver_type = disc_linear_solver();
    if (disc_ceres_threads() > 0) options.num_threads = disc_ceres_threads();
    options.minimizer_progress_to_stdout = false;
    // if(save_jacob){
    //     options.minimizer_progress_to_stdout = true;
    // }
    Solver::Summary summary;
    

    // summary.FullReport()
    // printf("final error: %f\n",summary.final_cost);
    // std::vector<double*> Problem::EvaluateOptions::parameter_blocks;
    // eval_option.parameter_blocks={};

    auto t_solve = disc_mark();
    Solve(options, &problem, &summary);
    disc_phase("ceres Solve", t_solve);
    normalize_Es();

    // Params results(fcs[0],fcs[1],fcs[2],fcs[3],fcs[4],distorsion[0],distorsion[1],distorsion[2],distorsion[3]);



    double cost = 0.0;
    Problem::EvaluateOptions eval_option;
    eval_option.apply_loss_function=false;
    std::vector<double> residuals;
    std::vector<double> gradient;
    ceres::CRSMatrix jacobian;

    Problem problem2; 
    for(int i=0;i<sample.size();i++){
        int index =sample.at(i);
        int num_residual= targets.at(index).size()*2;
        use_weight =false;

        CostFunction* cost_function = new NumericDiffCostFunction<CalibrationFunctor,ceres::CENTRAL, ceres::DYNAMIC, 5,4,3,3>(
            new CalibrationFunctor(origin_target,targets[index],original_r,n_d,projection_mode,use_weight), ceres::TAKE_OWNERSHIP, num_residual
        );
        problem2.AddResidualBlock(cost_function, new ceres::TrivialLoss(),results.K, results.d, Es.at(index).rot.data(),Es.at(index).trans.data() );

    }
    auto t_eval = disc_mark();
    problem2.Evaluate(eval_option, &cost, &residuals, &gradient, &jacobian);
    disc_phase("problem2.Evaluate", t_eval);
    // cout<<summary.FullReport()<<endl;

    // get uncertainty
    int n_row, n_col;
    n_row=jacobian.num_rows;
    n_col=jacobian.num_cols;
    int n_skip_cols = 4-n_d;
    int n_col_real= n_col-n_skip_cols;
    const bool sparse_info = disc_sparse_info();
    // J and J_real are the dense form of the jacobian. The sparse path reads the
    // CRS directly instead, but save_jacob still writes J_real to disk.
    const bool need_dense_j = !sparse_info || save_jacob;
    Eigen::MatrixXd J, J_real;
    if(need_dense_j){
        auto t_dense = disc_mark();
        J.resize(n_row, n_col);
        J.setZero();
        for(int i=0;i<n_row;i++){
            for(int curr_pos = jacobian.rows[i]; curr_pos <jacobian.rows[i+1]; curr_pos++){
                double value = jacobian.values[curr_pos];
                int col = jacobian.cols[curr_pos];
                J(i,col) = value;
            }
        }
        disc_phase("densify J (CRS->dense)", t_dense);

        auto t_real = disc_mark();
        J_real.resize(n_row, n_col_real);
        J_real.setZero();
        for(int i =0;i <n_row;i++){
            for(int j=0;j<n_col_real;j++){
                if(j<9-n_skip_cols) J_real(i,j) = J(i,j);
                else{
                    J_real(i,j) = J(i,j+n_skip_cols);
                }

            }
        }
        disc_phase("build J_real", t_real);
    }
    auto t_info = disc_mark();
    Eigen::MatrixXd info_matrix(n_col_real, n_col_real);
    if(sparse_info){
        // Tier 2: take J' W J straight from the CRS jacobian. Blob k owns CRS
        // rows 2k and 2k+1 (the pairing the dense M_cov used), and a residual row
        // touches only 5+n_d+6 columns -- so rather than sweep a dense n_col-long
        // row once per blob (n_row passes over an n_col^2 matrix, which is what
        // makes the accumulation memory-bound past ~200 frames) we gather just
        // that blob's columns, whiten in that small basis, and scatter the outer
        // products back. Traffic is O(n_blobs * m^2) with m ~ 13, i.e. ~1e7
        // operations at 275 frames instead of ~1e11.
        auto t_sparse = disc_mark();
        info_matrix.setZero();
        // CRS column -> J_real column. The dense path keeps j < 9-n_skip_cols
        // as-is and shifts the rest down by n_skip_cols, so the columns in
        // [9-n_skip_cols, 9) are the constant distortion terms it drops: those
        // are the only columns that map out of range and must be skipped.
        const int n_keep = 9 - n_skip_cols;
        // Each blob's rows are paired with the same target entry the dense path
        // used, so the number of targets has to match the jacobian's row pairs.
        int available = 0;
        for(int i=0;i<(int)sample.size();i++) available += (int)targets[sample.at(i)].size();
        const int n_blob = n_row/2;
        if(available != n_blob)
            printf("[sparse_info] warning: %d target blobs vs %d jacobian row pairs\n", available, n_blob);
        vector<int> idx;                        // this blob's columns (reused)
        vector<double> v0, v1, wa, wb;
        int blob = 0, bad = 0;
        for(int i=0;i<(int)sample.size() && blob<n_blob;i++){
            const vector<Shape> &target_i = targets[sample.at(i)];
            for(int j=0; j<(int)target_i.size() && blob<n_blob; j++, blob++){
                const int r0 = 2*blob, r1 = r0+1;
                const Shape &b = target_i[j];
                Eigen::Matrix2d cov;
                cov<<b.Kxx, b.Kxy, b.Kxy, b.Kyy;
                Eigen::LLT<Eigen::Matrix2d> llt(cov.inverse());
                if(llt.info()!=Eigen::Success){ // not positive definite: no info
                    bad++; continue;
                }
                const Eigen::Matrix2d L = llt.matrixL();
                idx.clear(); v0.clear(); v1.clear();
                for(int r=r0; r<=r1; r++){
                    for(int p=jacobian.rows[r]; p<jacobian.rows[r+1]; p++){
                        const int c = jacobian.cols[p];
                        if(c >= n_keep && c < 9) continue;   // constant distortion
                        const int rc = (c < n_keep) ? c : c - n_skip_cols;
                        int q = 0;
                        while(q < (int)idx.size() && idx[q] != rc) q++;
                        if(q == (int)idx.size()){
                            idx.push_back(rc); v0.push_back(0.0); v1.push_back(0.0);
                        }
                        (r==r0 ? v0[q] : v1[q]) = jacobian.values[p];
                    }
                }
                const int m = (int)idx.size();
                wa.resize(m); wb.resize(m);
                // [wa; wb] = L^T * [r0; r1], then info += wa wa^T + wb wb^T
                for(int p=0;p<m;p++){
                    wa[p] = L(0,0)*v0[p] + L(1,0)*v1[p];
                    wb[p] = L(1,1)*v1[p];
                }
                for(int p=0;p<m;p++)
                    for(int q=0;q<m;q++)
                        info_matrix(idx[p], idx[q]) += wa[p]*wa[q] + wb[p]*wb[q];
            }
        }
        if(bad)
            printf("[sparse_info] warning: %d non-positive-definite 2x2 blob(s) skipped\n", bad);
        if(blob != n_blob)
            printf("[sparse_info] warning: consumed %d of %d blobs\n", blob, n_blob);
        disc_phase("info from CRS (sparse)", t_sparse);
    } else if(disc_block_cov()){
        // M_cov is block diagonal -- one 2x2 block per blob, and nothing
        // off-diagonal, because measurements of different blobs are
        // independent. So J' M_cov^-1 J can be accumulated one blob at a time:
        // whiten that blob's two residual rows with the Cholesky factor of its
        // own 2x2 block (the same transform CFunctors.h applies to the
        // residuals) and add the two outer products. With W = M^-1 = L L^T and
        // the blob's rows [r0; r1], info += (L^T r0)^T (L^T r0) + (L^T r1)^T
        // (L^T r1) == sum_p J_p' W_p J_p == J' M_cov^-1 J. Same numbers, but no
        // n_row x n_row matrix and no O(n_row^3) inverse: O(n_col^2) memory.
        info_matrix.setZero();
        Eigen::RowVectorXd A(n_col_real), B(n_col_real);
        int row = 0, bad = 0;
        for(int i=0;i<(int)sample.size();i++){
            const vector<Shape> &target_i = targets[sample.at(i)];
            for(int j=0; j<(int)target_i.size(); j++){
                const Shape &blob = target_i[j];
                Eigen::Matrix2d cov;
                cov<<blob.Kxx, blob.Kxy, blob.Kxy, blob.Kyy;
                Eigen::LLT<Eigen::Matrix2d> llt(cov.inverse());
                if(llt.info()!=Eigen::Success){ // not positive definite: no info
                    bad++; row += 2; continue;
                }
                const Eigen::Matrix2d L = llt.matrixL();
                // [A; B] = L^T * [r0; r1]
                A.noalias() = L(0,0)*J_real.row(row) + L(1,0)*J_real.row(row+1);
                B.noalias() = L(1,1)*J_real.row(row+1);
                // info is symmetric, so accumulate the lower triangle only:
                // rankUpdate does half the flops of A^T A and goes through
                // Eigen's blocked selfadjoint path.
                info_matrix.selfadjointView<Eigen::Lower>().rankUpdate(A.transpose());
                info_matrix.selfadjointView<Eigen::Lower>().rankUpdate(B.transpose());
                row += 2;
            }
        }
        // rankUpdate touched only the lower triangle; info_matrix.inverse()
        // below reads the whole matrix, so mirror it before inverting.
        info_matrix = info_matrix.selfadjointView<Eigen::Lower>();
        if(bad)
            printf("[block_cov] warning: %d non-positive-definite 2x2 blob(s) skipped\n", bad);
        if(row != n_row)
            printf("[block_cov] warning: consumed %d of %d jacobian rows\n", row, n_row);
        disc_phase("info by block whitening", t_info);
    } else {
        auto t_mcov = disc_mark();
        Eigen::MatrixXd M_cov(n_row, n_row);
        M_cov.setZero();
        for(int i=0;i<sample.size();i++){
            vector<Shape> target_i = targets[sample.at(i)];
            for(int j=0; j<n_x*n_y;j++){
                int index = (i*n_x*n_y+j)*2;
                Shape blob_ij = target_i[j];
                M_cov(index,index) = blob_ij.Kxx;
                M_cov(index+1,index) = blob_ij.Kxy;
                M_cov(index,index+1) = blob_ij.Kxy;
                M_cov(index+1,index+1) = blob_ij.Kyy;
            }
        }
        disc_phase("assemble M_cov", t_mcov);
        //TODO: convert to sparse matrix form
        auto t_minv = disc_mark();
        Eigen::MatrixXd M_cov_inv = M_cov.inverse();
        disc_phase("M_cov.inverse()", t_minv);

        // Own mark: reusing t_info (which starts before the M_cov assembly)
        // reported assemble+inverse+GEMM as "J' Minv J" and inflated this phase
        // by the whole of M_cov.inverse().
        auto t_gemm = disc_mark();
        info_matrix = J_real.transpose() *M_cov_inv* J_real;
        disc_phase("J' Minv J", t_gemm);
    }

    auto t_iinv = disc_mark();
    Eigen::MatrixXd  cov_matrix = info_matrix.inverse();
    disc_phase("info_matrix.inverse()", t_iinv);
    array<double, 9> params_cov{0,};
    for(int i=0; i<5+n_d;i++){
        params_cov[i] = sqrt(cov_matrix(i,i));
    }
    if(disc_unc_dump()){
        printf("[unc]");
        for(int i=0; i<5+n_d; i++)
            printf(" %.17g", params_cov[i]);
        printf("\n");
    }
    results.update_unc(params_cov);

    if(save_jacob){
        //*************save files****************//
        string path;

        path = root_dir+"jacob.txt";
        std::ofstream writeFile2(path.data());
        if(writeFile2.is_open()){
            for(int i =0;i <n_row;i++){
                for(int j=0;j<n_col_real;j++){
                    writeFile2<< J_real(i,j);
                    if(j==n_col_real-1) writeFile2 << "\n";
                    else writeFile2 << "\t";
                }
            }
        }
        writeFile2.close();

        path = root_dir+"measurements.txt";
        std::ofstream writeFile3(path.data());
        writeFile3<<"x\ty\tkxx\tkxy\tkyy\n";
        if(writeFile3.is_open()){
            for(int i : sample){
                vector<Shape> target_i = targets[i];
                for(int j=0; j<target_i.size();j++){
                     writeFile3<< target_i[j].x << "\t" << target_i[j].y <<"\t"<< target_i[j].Kxx << "\t" << target_i[j].Kxy << "\t" << target_i[j].Kyy  << "\n";
                }
            }
        }
        writeFile3.close();
    }


    return results;
}

void Calibrator::visualize_rep(string path,Params params, int mode){
    std::vector<int> sample;
    for(int i=0;i<num_scene;i++){
        sample.push_back(i);
    }
    Eigen::Matrix3d E, E_inv, Qn;

    MomentsTracker* tracker = new MomentsTracker(n_d);

    int n = sample.size();
    vector<array<double,4>> buffer;
    for(int i=0; i<n;i++){
        int index = sample[i];
        E= LieAlgebra::to_E(Es[index]);

        for(int j=0;j<origin_target.size();j++){
            double wx = origin_target[j].x;
            double wy = origin_target[j].y;
            Point p_i = tracker->project(wx,wy,original_r,params,E,mode);
            double u_o = targets[index][j].x;
            double v_o = targets[index][j].y;
            buffer.push_back(array<double,4>{u_o,v_o,p_i.x,p_i.y});
        }
    }
    delete tracker;

    // cv::imwrite(path,img);
    std::ofstream writeFile(path.data());
    for(int i=0; i<buffer.size();i++){
        if(writeFile.is_open()){
            writeFile<<buffer[i][0]<<"\t"<<buffer[i][1]<<"\t"<<buffer[i][2]<<"\t"<<buffer[i][3]<<"\n";  
        }      
    }
    writeFile.close();
}

void Calibrator::save_data_for_gpr(std::vector<int> sample,Params params, int mode){
    // double fx{params.fx}, fy{params.fy}, cx{params.cx}, cy{params.cy}, skew{params.skew};
    vector<double> ds = {1, params.d[0], params.d[1],params.d[2],params.d[3]};
    Eigen::Matrix3d E, E_inv, Qn;

    MomentsTracker* tracker = new MomentsTracker(n_d);

    std::string path = root_dir + "gpr.txt";

    std::ofstream writeFile(path.data());
    writeFile << "sample\t"<<"order\t"<<""<< "x_d\t"<<"y_d\n";

    double error_total=0;
    int n = sample.size();
    for(int i=0; i<n;i++){
        int index = sample[i];
        E= LieAlgebra::to_E(Es[index]);
        double t_error=0;
        for(int j=0;j<origin_target.size();j++){
            double wx = origin_target[j].x;
            double wy = origin_target[j].y;

            Point dp = tracker->project( wx, wy, original_r, params, E, mode, false);

            // Eigen::Matrix3d Cw;
            // Cw <<   1.0, 0.0, -wx,
            //         0.0, 1.0, -wy,
            //         -wx, -wy, pow(wx,2)+pow(wy,2)-pow(original_r,2);

            // Point dp(0,0);
            // if(mode ==0){
            //     Eigen::Matrix3d E_inv=  E.inverse();
            //     Eigen::Matrix3d Qn = E_inv.transpose()*Cw*E_inv;
            //     dp = tracker->ne2dp(Qn,ds);
            // }
            // else if(mode == 1){
            //     Eigen::Matrix3d E_inv=  E.inverse();
            //     Eigen::Matrix3d Qn = E_inv.transpose()*Cw*E_inv;
            //     array<double,5> ellipse_n= tracker->ellipse2array(Qn);
            //     Point pn(ellipse_n[0],ellipse_n[1]);
            //     dp = tracker->distort_Point(pn,ds);
            // }
            // else if(mode == 2){
            //     Eigen::Vector3d Pw{wx,wy,1};
            //     Eigen::Vector3d Pn = E*Pw;
            //     Point pn(Pn[0]/Pn[2],Pn[1]/Pn[2]);
            //     dp = tracker->distort_Point(pn,ds);
            // }
            // else if(mode == 3){
            //     Eigen::Matrix3d E_inv=  E.inverse();
            //     Eigen::Matrix3d Qn = E_inv.transpose()*Cw*E_inv;
            //     dp = tracker->ne2dp_Numerical(Qn,ds);
            // }
            // else if(mode == 4){
            //     dp = tracker->wc2dp_Numerical(Cw,E,ds);
            // }
            // else{
            //     throw MomentsTrackerError();
            // }

            if(writeFile.is_open()){
                writeFile << index<<'\t'<< j<<'\t'<< dp.x<<'\t'<< dp.y<<"\n";
            }
        }
    }

    delete tracker;

    writeFile.close();

}

double Calibrator::cal_reprojection_error(std::vector<int> sample,Params params, int mode){
    // double fx{params.fx}, fy{params.fy}, cx{params.cx}, cy{params.cy}, skew{params.skew};
    // vector<double> ds = {1, params.d[0], params.d[1],params.d[2],params.d[3]};
    Eigen::Matrix3d E;

    MomentsTracker* tracker = new MomentsTracker(n_d);

    double error_total=0;
    int n = sample.size();
    for(int i=0; i<n;i++){
        int index = sample[i];
        E= LieAlgebra::to_E(Es[index]);
        double t_error=0;
        for(int j=0;j<origin_target.size();j++){
            double wx = origin_target[j].x;
            double wy = origin_target[j].y;
            Point p_i = tracker->project(wx,wy,original_r,params,E,mode);

            double u_e = p_i.x; 
            double v_e = p_i.y;

            double u_o = targets[index][j].x;
            double v_o = targets[index][j].y;

            double tt_error= sqrt(pow(u_e-u_o,2)+pow(v_e-v_o,2));  
            t_error += tt_error;
        }
        error_total += t_error/origin_target.size();
    }

    delete tracker;
    return error_total/n;
}

double Calibrator::cal_calibration_quality(std::vector<int> sample,Params params, int mode){
    // To be developed
    // double fx{params.fx}, fy{params.fy}, cx{params.cx}, cy{params.cy}, skew{params.skew};
    // vector<double> ds = {1, params.d[0], params.d[1],params.d[2],params.d[3]};
    Eigen::Matrix3d E;

    MomentsTracker* tracker = new MomentsTracker(n_d);

    double prob_total=0;
    int n = sample.size();
    for(int i=0; i<n;i++){
        int index = sample[i];
        E= LieAlgebra::to_E(Es[index]);
        double t_prop=0;
        for(int j=0;j<origin_target.size();j++){
            double wx = origin_target[j].x;
            double wy = origin_target[j].y;
            Point p_i = tracker->project(wx,wy,original_r,params,E,mode);

            double u_e = p_i.x; 
            double v_e = p_i.y;

            Shape measurement = targets[index][j];
            double u_o = measurement.x;
            double v_o = measurement.y;

            double e_x = u_o-u_e;
            double e_y = v_o-v_e;
            double det = measurement.Kxx*measurement.Kyy-pow(measurement.Kxy,2);
            double scale_factor = measurement.n;
            double info_xx = measurement.Kyy/det/scale_factor;
            double info_xy = -measurement.Kxy/det/scale_factor;
            double info_yy = measurement.Kxx/det/scale_factor;
            double cdf_x = sqrt((info_xx*pow(e_x,2)+2*info_xy*e_x*e_y+info_yy*pow(e_y,2)));
            double tt_prob = 1-cdf(cdf_x);
            t_prop += tt_prob;           
        }
        prob_total += t_prop/origin_target.size();
    }

    delete tracker;
    return prob_total/n;
}
void Calibrator::normalize_Es(){
    for(int i=0; i<Es.size();i++){
        Es[i].rot = LieAlgebra::normalize_so3(Es[i].rot);
        if(Es[i].trans(2)<0){
            Es[i].trans = -Es[i].trans;
            Eigen::Matrix3d R = LieAlgebra::to_SO3(Es[i].rot);
            R.col(0) = -R.col(0);
            R.col(1) = -R.col(1);
            Es[i].rot= LieAlgebra::to_so3(R);
        }
    }

}

void Calibrator::save_extrinsic(){
    save_extrinsic(root_dir+"est_pose.txt");
}
void Calibrator::save_extrinsic(string path){
    std::ofstream writeFile(path.data());
    for(int i=0; i<Es.size();i++){
        se3 E = Es[i];
        if(writeFile.is_open()){
            writeFile<<E.rot(0)<<"\t"<<E.rot(1)<<"\t"<<E.rot(2)<<"\t"<<E.trans(0)<<"\t"<<E.trans(1)<<"\t"<<E.trans(2)<<"\n";  
        }      
    }
    writeFile.close();
}
vector<se3> Calibrator::get_extrinsic(){
    return Es;
}

void Calibrator::update_Es(Params intrinsic, int mode){
    init();
    set_inital_Es(intrinsic);
    std::vector<int> sample = sorted_random_sampling(num_scene,num_scene);
    // double fcs[5]={intrinsic.fx, intrinsic.fy, intrinsic.cx, intrinsic.cy, intrinsic.skew};
    // double distorsion[4]={intrinsic.d[0],intrinsic.d[1],intrinsic.d[2],intrinsic.d[3]};
    bool use_weight= true;
    Problem problem; 
    ceres::LossFunction* loss_function;
    loss_function = new ceres::TrivialLoss();
    for(int i=0;i<sample.size();i++){
        int index =sample.at(i);
        int num_residual= targets.at(index).size()*2;
        CostFunction* cost_function = new NumericDiffCostFunction<CalibrationFunctor,ceres::CENTRAL, ceres::DYNAMIC,5,4,3,3>(
            new CalibrationFunctor(origin_target,targets[index],original_r,n_d,mode,use_weight), ceres::TAKE_OWNERSHIP, num_residual
        );
        problem.AddResidualBlock(cost_function, loss_function, intrinsic.K, intrinsic.d, Es.at(index).rot.data(),Es.at(index).trans.data() );

    }
    problem.SetParameterBlockConstant(intrinsic.K);
    problem.SetParameterBlockConstant(intrinsic.d);

    Solver::Options options;
    options.linear_solver_type = disc_linear_solver();
    if (disc_ceres_threads() > 0) options.num_threads = disc_ceres_threads();
    options.minimizer_progress_to_stdout = false;
    Solver::Summary summary;
    Solve(options, &problem, &summary);
    normalize_Es();
}


Params Calibrator::calibrate(int mode, bool save_jacob){
    init();
    bool debug = true;
    double alpha;
    std::cout.precision(12);
    Params initial_params;
    Params final_params;
    string path = root_dir + "intrinsic_parameters.yaml";
    std::ofstream writeFile(path.data());
   
    bool result= cal_initial_params(&initial_params);
    if(!result){
        initial_params.fx() = initial_params.cx() = this->width/2;
        initial_params.fy() = initial_params.cy() = this->height/2;
    }
    //  if(debug) cout<< initial_params.to_string_detailed()<<endl;
    // if(debug) printParams(initial_params,true);
    
    printf("radius: %f, distance : %f, optimization mode: %d\n",original_r, distance, mode);

    // 0: all, 2: w/o r
    struct timeval  tv;
	double begin, end;
    gettimeofday(&tv, NULL);
	begin = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;

    set_inital_Es(initial_params);
    std::vector<int> sample = sorted_random_sampling(num_scene,num_scene);

    if(mode==0){
        // initial_params.skew=0;
        printf(">>>Coarse results\n");
        final_params= batch_optimize(sample, initial_params,2);
        cout << final_params.to_string_detailed()<<endl;

        printf(">>>Fine results (It may take few minutes.....)\n");
        final_params= batch_optimize(sample, final_params,mode,save_jacob);
    }
    else{
        final_params= batch_optimize(sample, initial_params,mode,save_jacob);
        
    }

    cout<<final_params.to_string_detailed() <<endl;

    double rperror = cal_reprojection_error(sample,final_params,mode);
    printf("Reprojection error: %f\n",rperror);

    
    if(writeFile.is_open()){
        writeFile<<"results_dir: \""<<root_dir <<"\"\n";  
        writeFile<<"width: "<<width <<"\n";  
        writeFile<<"height: "<<height <<"\n";  
        writeFile<<"n_d: "<<n_d <<"\n";  
        writeFile<<"mode: "<<mode <<"\n";
        writeFile<<"radius: " << original_r << "\n";
        writeFile<<"distance: " << distance << "\n";

        writeFile<<"fx: " << final_params.K[0] << "\n";
        writeFile<<"fy: " << final_params.K[1]<< "\n";
        writeFile<<"cx: " << final_params.K[2] << "\n";
        writeFile<<"cy: " << final_params.K[3] << "\n";
        writeFile<<"skew: " << final_params.K[4] << "\n";

        writeFile<<"d1: " << final_params.d[0] << "\n";
        writeFile<<"d2: " << final_params.d[1] << "\n";
        writeFile<<"d3: " << final_params.d[2] << "\n";
        writeFile<<"d4: " << final_params.d[3] << "\n";

        writeFile<<"sfx: " << final_params.s_K[0] << "\n";
        writeFile<<"sfy: " << final_params.s_K[1] << "\n";
        writeFile<<"scx: " << final_params.s_K[2] << "\n";
        writeFile<<"scy: " << final_params.s_K[3] << "\n";

        writeFile<<"sd1: " << final_params.s_d[0] << "\n";
        writeFile<<"sd2: " << final_params.s_d[1] << "\n";
        writeFile<<"sd3: " << final_params.s_d[2] << "\n";
        writeFile<<"sd4: " << final_params.s_d[3] << "\n";
        writeFile<<"reprojection_error: " << rperror;
    } 

    writeFile.close();
    cout << "calibration results are saved at: "<< path<<endl;

    // undistort_total_images(final_params);
    gettimeofday(&tv, NULL);
    end = (tv.tv_sec) * 1000 + (tv.tv_usec) / 1000 ;
    double duration =(end - begin) / 1000;
    printf("finishied. Runtime: %.2f\n", duration);

    return final_params;
}


//---------------------Zhang's method-----------------------------//

array<double, 4> Calibrator::get_mean_sigma(const vector<Shape> &target){
    double m_x{0}, m_y{0}, s_x{0},s_y{0};
    int n = target.size();
    for(int i=0; i<n;i++){
        m_x+= target[i].x;
        m_y+= target[i].y;
        s_x+= pow(target[i].x,2);
        s_y+= pow(target[i].y,2);
    }
    m_x = m_x/n;
    m_y = m_y/n;
    s_x = sqrt(s_x/n - pow(m_x,2));
    s_y = sqrt(s_y/n - pow(m_y,2));
    array<double, 4> result{m_x, m_y, s_x, s_y};
    return result;
}

pair<Eigen::Matrix3d,double> Calibrator::get_H(const vector<Shape> &target){
    
    int n_points = target.size();
    array<double,4> ms = get_mean_sigma(target);
    double m_u{ms[0]}, m_v{ms[1]}, s_u{ms[2]}, s_v{ms[3]};

    double m_x{ori_ms[0]}, m_y{ori_ms[1]}, s_x{ori_ms[2]}, s_y{ori_ms[3]};

    Eigen::Matrix3d H, Kt, Ko;
    double singular_value_ratio =1;

    // (n>4) -> DLT, n==4 -> exact solution
    if(n_points>4){
        Eigen::MatrixXd X(2*n_points,9);
        X.setZero();
        for (int i=0;i<n_points;i++){
            double x,y,u,v;

            u = (target[i].x-m_u)/s_u;
            v = (target[i].y-m_v)/s_v;
            x = (origin_target[i].x-m_x)/s_x;
            y = (origin_target[i].y-m_y)/s_y;

            X(2*i, 0)=x;
            X(2*i,1) = y;
            X(2*i,2) = 1;
            X(2*i,6) = -u*x;
            X(2*i,7) = -u*y;
            X(2*i,8) = -u;

            X(2*i+1,3)=x;
            X(2*i+1,4) = y;
            X(2*i+1,5) = 1;
            X(2*i+1,6) = -v*x;
            X(2*i+1,7) = -v*y;
            X(2*i+1,8) = -v;

        }
        Eigen::BDCSVD<Eigen::MatrixXd> svd(X,Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::MatrixXd V = svd.matrixV();
        H << V(0,8),V(1,8),V(2,8),
            V(3,8),V(4,8),V(5,8),
            V(6,8),V(7,8),V(8,8);
        Eigen::VectorXd singular_values=svd.singularValues();
        double singular_value_ratio = singular_values(7)/singular_values(8);
    }
    else if(n_points==4){
        Eigen::MatrixXd X(2*n_points,8);
        Eigen::VectorXd b(2*n_points);
        X.setZero();
        for (int i=0;i<n_points;i++){
            double x,y,u,v;

            u = (target[i].x-m_u)/s_u;
            v = (target[i].y-m_v)/s_v;
            x = (origin_target[i].x-m_x)/s_x;
            y = (origin_target[i].y-m_y)/s_y;

            X(2*i, 0)=x;
            X(2*i,1) = y;
            X(2*i,2) = 1;
            X(2*i,6) = -u*x;
            X(2*i,7) = -u*y;
            X(2*i+1,3)=x;
            X(2*i+1,4) = y;
            X(2*i+1,5) = 1;
            X(2*i+1,6) = -v*x;
            X(2*i+1,7) = -v*y;

            b(2*i) = u;
            b(2*i+1) = v;
        }
        Eigen::VectorXd V = X.fullPivLu().solve(b);
        H << V(0),V(1),V(2),
            V(3),V(4),V(5),
            V(6),V(7),1;
    }
    else throw exception();    

    Ko << 1/s_x, 0, -m_x/s_x,
            0,  1/s_y, -m_y/s_y,
            0,  0,  1;

    Kt << 1/s_u, 0, -m_u/s_u,
            0,  1/s_v, -m_v/s_v,
            0,  0,  1;

    // std::cout<<"singular value ratio: "<<singular_value_ratio<<std::endl;

    H = Kt.inverse() * H * Ko;
    H = normalize_matrix(H);
    pair<Eigen::Matrix3d,double> result(H,singular_value_ratio);
    
    return result;
}

Eigen::MatrixXd Calibrator::normalize_matrix(Eigen::MatrixXd matrix){
    double norm = matrix.norm();
    return matrix/norm;
}

Eigen::VectorXd Calibrator::get_Vij(const Eigen::Matrix3d &H, int i, int j){
    Eigen::Vector3d h_i,h_j;
    h_i= H.col(i);
    h_j= H.col(j);

    Eigen::VectorXd v_ij(6);
    v_ij << h_i(0)*h_j(0), 
            h_i(0)*h_j(1)+h_i(1)*h_j(0),
            h_i(1)*h_j(1),
            h_i(2)*h_j(0)+h_i(0)*h_j(2),
            h_i(2)*h_j(1)+h_i(1)*h_j(2),
            h_i(2)*h_j(2);

    return v_ij;
}

Eigen::VectorXd Calibrator::get_absolute_conic(){
    Eigen::VectorXd b(6);
    Eigen::MatrixXd X(2*this->num_scene,6);
    X.setZero();
    // std::cout<<"num_scene: "<<this->num_scene<<std::endl;

    double h_mean=0;
    for(int i=0;i<num_scene;i++){
        h_mean+= Hs.at(i).second;
    }
    h_mean = h_mean/num_scene;
    // printf("h_sigma: %f\n",h_sigma);
    
    for(int i=0;i<num_scene;i++){
        pair<Eigen::Matrix3d,double> result  = Hs.at(i);
        Eigen::Matrix3d H = result.first;
        // cout<<H<<endl;
        double h_ratio =result.second/h_mean;
        // std::cout<<"singular value ratio: "<<h_ratio<<std::endl;
        H*= h_ratio;
        if (h_ratio>this->thr_homography){
            Eigen::VectorXd v_01 = get_Vij(H,0,1);
            Eigen::VectorXd v_00 = get_Vij(H,0,0);
            Eigen::VectorXd v_11 = get_Vij(H,1,1);
            X.row(2*i)=v_01;
            X.row(2*i+1) = v_00-v_11;
        }
    }
    // cout<<X<<endl;
    Eigen::BDCSVD<Eigen::MatrixXd> svd(X,Eigen::ComputeThinU | Eigen::ComputeThinV);
    b = svd.matrixV().col(5);
    return b;
}





bool Calibrator::cal_initial_params(Params* inital_params){
    if(targets.size()<5){
        printf("Error: algorithm need at leat 5 images");
        return false;
    }
    else{
        Eigen::VectorXd b = get_absolute_conic();
        // cout<<b<<endl;
        // std::cout<<"absolute_conic:\n"<<b<<std::endl;
        double cx,cy,fx,fy,fx2,fy2,skew,lamda;
        cy = (b(1)*b(3)-b(0)*b(4))/(b(0)*b(2)-b(1)*b(1));
        lamda = b(5) - (b(3)*b(3)+cy*(b(1)*b(3)-b(0)*b(4))) /b(0);
        fx2= lamda/b(0);
        fy2= lamda*b(0)/(b(0)*b(2)-b(1)*b(1));
        if(fx2>0 && fy2>0){
            fx = sqrt(fx2);
            fy = sqrt(fy2);
            skew = -b(1)*fx*fx*fy/lamda;
            cx = skew*cy/fx - b(3)*fx*fx/lamda;

            if(fx <0 || fy <0 || cx <0 || cy < 0){
                return false;
            }
            else{
                inital_params->fx()= fx;
                inital_params->fy()= fy;
                inital_params->cx() = cx;
                inital_params->cy()= cy;
                inital_params->skew() = skew;
                return true;
            }
        }
        else{
            // printf("Not enough images collected");
            return false;
        }

    }

}

//---------------------Zhang's method-----------------------------//