#include <ifopt_test/test_objective.h>
#include <chrono>
namespace motion_planning {

// Objective::Objective()
//     : Objective("k_order=2", 7, 20)
// {
// }

Objective::Objective(const std::string& name, const int num_variables, const int num_time_slices)
    : ifopt::CostTerm(name)
    , _num_variables(num_variables)
    , _num_time_slices(num_time_slices)
{
}


void Objective::GetStateNodes(VectorXd &sym_set, Eigen::MatrixXd &internal_set) const
{
    size_t idx = 0;
    for (size_t j=0; j<internal_set.cols(); j++)
    {
        for (size_t i=0; i<internal_set.rows(); i++)
        {
            internal_set(i, j) = sym_set(idx);
            idx++;
        }
    }
}

double Objective::GetCost() const
{
    // std::cout.precision(5);
    // auto my_set = std::dynamic_pointer_cast<DecisionVariables>(GetVariables()->GetComponent("x"));
    // my_set->SetVariables
    VectorXd x = GetVariables()->GetComponent("x")->GetValues();
    // std::cout << "Vector:\n" << x.transpose() << "\n" << std::endl;

    Eigen::MatrixXd x_t(_num_variables, _num_time_slices);
    GetStateNodes(x, x_t);
    // std::cout << "Matrix:\n" << x_t << "\n" << std::endl;
    
    double cost = 0.0;
    // First col is zero
    // -----
    // Second column
    size_t idx = 1;
    auto temp = x_t.col(idx) - x_t.col(idx-1);
    cost += temp.transpose()*temp;

    // Rest of columns
    idx++;
    for (idx; idx<_num_time_slices; idx++)
    {
        auto temp = x_t.col(idx) - 2*x_t.col(idx-1) + x_t.col(idx-2);
        cost += temp.transpose()*temp;
        // std::cout << "cost: " << cost << " ";

    }

    // std::cout << "temp is:\n" << temp << std::endl;

            // for (size_t i=0; i<_; i++)
    // {
    // }

    // for (int i=0; i<_num_variables; i++)
    // {
    //     cost += std::pow(x(i+_num_variables) - x(i),2); 
    //     idx++;
        
    // }
    // int remaining_iter = x.size() - _num_variables;

    // for (int i=_num_variables; i<remaining_iter; i++)
    // {
    //     cost += std::pow((x(idx) - 2*x(idx-_num_variables) + x(2*_num_variables)), 2);
    //     std::cout << "idx" << idx << "  ";
    //     idx++;
    // }


    // Eigen::MatrixXd x_t(_num_variables, _num_time_slices);
    // GetStateNodes(x, x_t);
    // // std::cout << "x_t\n"<< x_t << std::endl;
    
    // Eigen::MatrixXd x_t_minus1(_num_variables, _num_time_slices);
    // x_t_minus1.col(0) = x_t.col(0);
    // x_t_minus1.block(0, 1, _num_variables, _num_time_slices-1) << x_t.block(0,0,_num_variables,_num_time_slices-1);
    // // std::cout << "\nx_t_minus1\n" << x_t_minus1 << std::endl;


    // Eigen::MatrixXd x_t_minus2(_num_variables, _num_time_slices);
    // x_t_minus2.col(0) = x_t.col(0);
    // x_t_minus2.col(1) = x_t.col(0);
    // x_t_minus2.block(0, 2, _num_variables, _num_time_slices-2) << x_t.block(0,0,_num_variables,_num_time_slices-2);
    // // std::cout << "\nx_t_minus2\n" << x_t_minus2 << std::endl;
 

    // double cost = 0;
    // // std::cout << "\ncost:\n";
    // for (int i=0; i<_num_time_slices; i++)
    // {
    //     // std::cout << "x_t(i): " << x_t.col(i) << "\t -2*x_t_minus1: " << -2*x_t_minus1.col(i) << "\t x_t_minus2: " << x_t_minus2.col(i) << "\n";
    //     Eigen::VectorXd f = (x_t.col(i) - (2*x_t_minus1.col(i)) + x_t_minus2.col(i));
    //     // Eigen::VectorXd f = x_t.col(i) - x_t_minus1.col(i);
    //     // std::cout << "f : "<< f << std::endl;

    //     cost += (f.transpose() * f);
    //     // std::cout << "i:" << i << "\t cost: " << cost << "\t f: " << f << "\t transpose: " << f.transpose() << "\n\n";
    // }
    // // std::cout << "cost " << cost << std::endl;

    return cost;    
}

void Objective::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
    if (var_set == "x") {
        VectorXd x = GetVariables()->GetComponent("x")->GetValues();

        for (int idx=0; idx<_num_variables; idx++)
        {
            jac.coeffRef(0, idx)                  = 4*x(idx) - 6*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
            jac.coeffRef(0, idx+_num_variables)   = 12*x(idx+_num_variables) - 6*x(idx) - 8*x(idx + 2*_num_variables) + 2*x(idx + 3*_num_variables);
            jac.coeffRef(0, idx+2*_num_variables) = 2*x(idx)                     - 8*x(idx + _num_variables)    + 12*x(idx+ 2*_num_variables)  - 8*x(idx+3*_num_variables)  + 2*x(idx+4*_num_variables);
            jac.coeffRef(0, idx+3*_num_variables) = 2*x(idx+ _num_variables)     - 8*x(idx + 2*_num_variables)  + 12*x(idx+ 3*_num_variables)  - 8*x(idx+4*_num_variables)  + 2*x(idx+5*_num_variables);
            jac.coeffRef(0, idx+4*_num_variables) = 2*x(idx+ 2*_num_variables)   - 8*x(idx + 3*_num_variables)  + 12*x(idx+ 4*_num_variables)  - 8*x(idx+5*_num_variables)  + 2*x(idx+6*_num_variables);
            jac.coeffRef(0, idx+5*_num_variables) = 2*x(idx+ 3*_num_variables)   - 8*x(idx + 4*_num_variables)  + 12*x(idx+ 5*_num_variables)  - 8*x(idx+6*_num_variables)  + 2*x(idx+7*_num_variables);
            jac.coeffRef(0, idx+6*_num_variables) = 2*x(idx+ 4*_num_variables)   - 8*x(idx + 5*_num_variables)  + 12*x(idx+ 6*_num_variables)  - 8*x(idx+7*_num_variables)  + 2*x(idx+8*_num_variables);
            jac.coeffRef(0, idx+7*_num_variables) = 2*x(idx+ 5*_num_variables)   - 8*x(idx + 6*_num_variables)  + 12*x(idx+ 7*_num_variables)  - 8*x(idx+8*_num_variables)  + 2*x(idx+9*_num_variables);
            jac.coeffRef(0, idx+8*_num_variables) = 2*x(idx+ 6*_num_variables)   - 8*x(idx + 7*_num_variables)  + 12*x(idx+ 8*_num_variables)  - 8*x(idx+9*_num_variables)  + 2*x(idx+10*_num_variables);
            jac.coeffRef(0, idx+9*_num_variables) = 2*x(idx+ 7*_num_variables)   - 8*x(idx + 8*_num_variables)  + 12*x(idx+ 9*_num_variables)  - 8*x(idx+10*_num_variables) + 2*x(idx+11*_num_variables);
            jac.coeffRef(0, idx+10*_num_variables) = 2*x(idx+ 8*_num_variables)  - 8*x(idx + 9*_num_variables)  + 12*x(idx+ 10*_num_variables) - 8*x(idx+11*_num_variables) + 2*x(idx+12*_num_variables);
            jac.coeffRef(0, idx+11*_num_variables) = 2*x(idx+ 9*_num_variables)  - 8*x(idx + 10*_num_variables) + 12*x(idx+ 11*_num_variables) - 8*x(idx+12*_num_variables) + 2*x(idx+13*_num_variables);
            jac.coeffRef(0, idx+12*_num_variables) = 2*x(idx+ 10*_num_variables) - 8*x(idx + 11*_num_variables) + 12*x(idx+ 12*_num_variables) - 8*x(idx+13*_num_variables) + 2*x(idx+14*_num_variables);
            jac.coeffRef(0, idx+13*_num_variables) = 2*x(idx+ 11*_num_variables) - 8*x(idx + 12*_num_variables) + 12*x(idx+ 13*_num_variables) - 8*x(idx+14*_num_variables) + 2*x(idx+15*_num_variables);
            jac.coeffRef(0, idx+14*_num_variables) = 2*x(idx+ 12*_num_variables) - 8*x(idx + 13*_num_variables) + 12*x(idx+ 14*_num_variables) - 8*x(idx+15*_num_variables) + 2*x(idx+16*_num_variables);
            jac.coeffRef(0, idx+15*_num_variables) = 2*x(idx+ 13*_num_variables) - 8*x(idx + 14*_num_variables) + 12*x(idx+ 15*_num_variables) - 8*x(idx+16*_num_variables) + 2*x(idx+17*_num_variables);
            jac.coeffRef(0, idx+16*_num_variables) = 2*x(idx+ 14*_num_variables) - 8*x(idx + 15*_num_variables) + 12*x(idx+ 16*_num_variables) - 8*x(idx+17*_num_variables) + 2*x(idx+18*_num_variables);
            jac.coeffRef(0, idx+17*_num_variables) = 2*x(idx+ 15*_num_variables) - 8*x(idx + 16*_num_variables) + 12*x(idx+ 17*_num_variables) - 8*x(idx+18*_num_variables) + 2*x(idx+19*_num_variables);
            jac.coeffRef(0, idx+18*_num_variables) = 2*x(idx+ 16*_num_variables) - 8*x(idx + 17*_num_variables) + 10*x(idx+ 18*_num_variables) - 4*x(idx+19*_num_variables);
            jac.coeffRef(0, idx+19*_num_variables) = 2*x(idx+ 17*_num_variables) - 4*x(idx + 18*_num_variables) + 2*x(idx+  19*_num_variables);
        }
    }
}

} //namespace ifopt