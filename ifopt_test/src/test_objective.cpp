#include <ifopt_test/test_objective.h>

namespace motion_planning {

Objective::Objective()
    : Objective("k_order=2", 7, 20)
{
}

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
    VectorXd x = GetVariables()->GetComponent("x")->GetValues();
    Eigen::MatrixXd x_t(_num_variables, _num_time_slices);
    GetStateNodes(x, x_t);
    // std::cout << "\nx_t\n" << x_t << std::endl;

    
    Eigen::MatrixXd x_t_minus1(_num_variables, _num_time_slices);
    x_t_minus1.col(0) = x_t.col(0);
    x_t_minus1.block(0, 1, _num_variables, _num_time_slices-1) << x_t.block(0,0,_num_variables,_num_time_slices-1);
    // std::cout << "\nx_t_minus1\n" << x_t_minus1 << std::endl;


    Eigen::MatrixXd x_t_minus2(_num_variables, _num_time_slices);
    x_t_minus2.col(0) = x_t.col(0);
    x_t_minus2.col(1) = x_t.col(0);
    x_t_minus2.block(0, 2, _num_variables, _num_time_slices-2) << x_t.block(0,0,_num_variables,_num_time_slices-2);
    // std::cout << "\nx_t_minus2\n" << x_t_minus2 << std::endl;

    double cost = 0;
    // std::cout << "\ncost:\n";
    for (int i=0; i<_num_time_slices; i++)
    {
        // std::cout << "x_t(i): " << x_t.col(i) << "\t -2*x_t_minus1: " << -2*x_t_minus1.col(i) << "\t x_t_minus2: " << x_t_minus2.col(i) << "\n";
        Eigen::VectorXd f = (x_t.col(i) - (2*x_t_minus1.col(i)) + x_t_minus2.col(i));
        // Eigen::VectorXd f = x_t.col(i) - x_t_minus1.col(i);
        // std::cout << "f : "<< f << std::endl;

        cost += (f.transpose() * f);
        // std::cout << "i:" << i << "\t cost: " << cost << "\t f: " << f << "\t transpose: " << f.transpose() << "\n\n";
    }
    // std::cout << "cost " << cost << std::endl;
    return cost;    
}

void Objective::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
    if (var_set == "x") {
        VectorXd x = GetVariables()->GetComponent("x")->GetValues();

        // jac.coeffRef(0, 0) = 4*x(0) - 6*x(1) + 2*x(2);
        // jac.coeffRef(0, 1) = 12*x(1) - 6*x(0) - 8*x(2) + 2*x(3);
        // jac.coeffRef(0, 2) = 2*x(0) - 8*x(1) + 12*x(2) - 8*x(3) + 2*x(4);
        // jac.coeffRef(0, 3) = 2*x(1) -8*x(2) + 10*x(3) - 4*x(4);
        // jac.coeffRef(0, 4) = 2*x(2) - 4*x(3) + 2*x(4);

        // jac.coeffRef(0, 0) = 4*x(0) - 6*x(7) + 2*x(14);
        // jac.coeffRef(0, 1) = 4*x(1) - 6*x(8) + 2*x(15);

        int idx = 0; // (0col)
        for (idx; idx<_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 4*x(idx) - 6*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
        }
        idx++; // idx=7 (1col)
        for (idx; idx<2*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 12*x(idx) - 6*x(idx-_num_variables) - 8*x(idx + _num_variables) + 2*x(idx + 2*_num_variables);
        }
        idx++; //idx=14 (2col)
        for (idx; idx<3*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx - 2*_num_variables) - 8*x(idx - _num_variables) + 12*x(idx) - 8*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
        }
        idx++; //idx=21 (3col)
        for (idx; idx<4*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx - 2*_num_variables) - 8*x(idx - _num_variables) + 12*x(idx) - 8*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
        }
        idx++; //idx=28 (4col)
        for (idx; idx<5*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx - 2*_num_variables) - 8*x(idx - _num_variables) + 12*x(idx) - 8*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
        }
        idx++; //idx=35 (5col)
        for (idx; idx<6*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx - 2*_num_variables) - 8*x(idx - _num_variables) + 12*x(idx) - 8*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
        }
        idx++; //idx=42 (6col)
        for (idx; idx<7*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx - 2*_num_variables) - 8*x(idx - _num_variables) + 12*x(idx) - 8*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
        }
        idx++; //idx=49 (7col)
        for (idx; idx<8*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx - 2*_num_variables) - 8*x(idx - _num_variables) + 12*x(idx) - 8*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
        }
        idx++; //idx=56 (8col)
        for (idx; idx<9*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx - 2*_num_variables) - 8*x(idx - _num_variables) + 12*x(idx) - 8*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
        }
        idx++; //idx=63 (9col)
        for (idx; idx<10*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx - 2*_num_variables) - 8*x(idx - _num_variables) + 12*x(idx) - 8*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
        }
        idx++; //idx=70 (10col)
        for (idx; idx<11*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx - 2*_num_variables) - 8*x(idx - _num_variables) + 12*x(idx) - 8*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
        }
        idx++; //idx=77 (11col)
        for (idx; idx<12*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx - 2*_num_variables) - 8*x(idx - _num_variables) + 12*x(idx) - 8*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
        }
        idx++; //idx=84 (12col)
        for (idx; idx<13*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx - 2*_num_variables) - 8*x(idx - _num_variables) + 12*x(idx) - 8*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
        }
        idx++; //idx=91 (13col)
        for (idx; idx<14*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx - 2*_num_variables) - 8*x(idx - _num_variables) + 12*x(idx) - 8*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
        }
        idx++; //idx=98 (14col)
        for (idx; idx<15*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx - 2*_num_variables) - 8*x(idx - _num_variables) + 12*x(idx) - 8*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
        }
        idx++; //idx=105 (15col)
        for (idx; idx<16*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx - 2*_num_variables) - 8*x(idx - _num_variables) + 12*x(idx) - 8*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
        }
        idx++; //idx=112 (16col)
        for (idx; idx<17*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx - 2*_num_variables) - 8*x(idx - _num_variables) + 12*x(idx) - 8*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
        }
        idx++; //idx=119 (17col)
        for (idx; idx<18*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx - 2*_num_variables) - 8*x(idx - _num_variables) + 12*x(idx) - 8*x(idx+_num_variables) + 2*x(idx+2*_num_variables);
        }
        idx++; //idx=126 (18col)
        for (idx; idx<19*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx-2*_num_variables) -8*x(idx-_num_variables) + 10*x(idx) - 4*x(idx+_num_variables);
        }
        idx++; //idx=133 (19col)
        for (idx; idx<19*_num_variables; idx++)
        {
            jac.coeffRef(0, idx) = 2*x(idx - 2*_num_variables) - 4*x(idx - _num_variables) + 2*x(idx);
        }
    }
}

} //namespace ifopt