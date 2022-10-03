#include <panda_lgp/KOMO/komo_objective.h>



namespace KOMO_k2
{

void GetStateNodes(const std::vector<double> &vector_set, Eigen::MatrixXd  &internal_rep_set)
{
    size_t idx = 0;
    for (size_t j=0; j<internal_rep_set.cols(); ++j)
    {
        for (size_t i=0; i<internal_rep_set.rows(); ++i)
        {
            internal_rep_set(i, j) = vector_set[idx];
            idx++;
        }
    }
}


double GetCost(const std::vector<double> &x, std::vector<double> &grad, void *data)
{
    ObjectiveData *d = reinterpret_cast<ObjectiveData*>(data);

    // if (!grad.empty()) {
    //     std::fill(grad.begin(), grad.end(), 0.0);
    //     FillJacobianBlock(x, grad, d);
    // }
    // Eigen::MatrixXd x_t(d->num_phase_variables, d->num_phases);

    // GetStateNodes(x, x_t);
    // double cost = 0.0;
    // // First col is zero
    // // -----

    // // Second column
    // size_t idx = 1;
    // auto temp = x_t.col(idx) - x_t.col(idx-1);
    // cost += temp.transpose()*temp;

    // // Rest of columns
    // // idx++;
    // // for (idx; idx<d->num_phase_variables; idx++)
    // // {
    // //     auto temp = x_t.col(idx) - 2*x_t.col(idx-1) + x_t.col(idx-2);
    // //     cost += temp.transpose()*temp;
    // // }
    // return cost;    
    Eigen::VectorXd test(3);

   

    Eigen::Map<const Eigen::Matrix<double, 10, 2>> x_matrix(x.data());


    VectorXreal x_diff = Eigen::Map<const Eigen::VectorXd>(x.data(), x.size());
    VectorXreal F; 

    MatrixXreal 
    // Objective(x_diff);
    std::cout << "velkost x_diff: " << x_diff.size() << "\n";
    Eigen::Map<Eigen::MatrixXd> x_mat(x_diff,10,2);
    // Eigen::MatrixXd J = jacobian(Objective, wrt(x), at(x), F); 

    // std::cout << "F = \n" << F << std::endl;    // print the evaluated output vector F
    // std::cout << "J = \n" << J << std::endl;    // print the evaluated Jacobian matrix dF/dx

    return 0.0;

    

}

void FillJacobianBlock(const std::vector<double> &x, std::vector<double> &jac, ObjectiveData *data)
{
    if (data->num_phases == 2)
    {
        uint half_x_dim = x.size()/2;
        for (uint idx=0; idx<half_x_dim; ++idx)
        {
            jac[idx] = 2*x[idx] - 2*x[idx + half_x_dim];
            jac[idx + half_x_dim] = 2*x[idx+half_x_dim] - 2*x[idx];
        }
    }
    
}


VectorXreal Objective(const VectorXreal& x)
{
    // Eigen::Map<const Eigen::MatrixXd> x_mat(x,10,2);

}

}

