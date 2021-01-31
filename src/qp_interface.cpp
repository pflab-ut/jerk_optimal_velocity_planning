#include "qp_interface.h"

QPOptimizer::QPOptimizer(const OptimizerParam &param) : param_(param)
{
    qp_solver_.updateMaxIter(4000);
    qp_solver_.updateRhoInterval(0);  // 0 means automoatic
    qp_solver_.updateEpsRel(1.0e-4);  // def: 1.0e-4
    qp_solver_.updateEpsAbs(1.0e-4);  // def: 1.0e-4
    qp_solver_.updateVerbose(false);
}

void QPOptimizer::setParam(const OptimizerParam &param)
{
    param_ = param;
}

bool QPOptimizer::solve(const double &initial_vel,
                        const double &initial_acc,
                        const double &ds,
                        const std::vector<double> &ref_vel,
                        const std::vector<double> &max_vel,
                        const std::vector<double> &ref_acc,
                        QPOutputInfo &qp_output)
{
    assert(ref_vel.size()==max_vel.size());
    int N = ref_vel.size();

    /*
     * x = [b[0], b[1], ..., b[N] | a[0], a[1], .... a[N] | delta[0], ..., delta[N]
     *      | sigma[0], sigma[1], ...., sigma[N] | gamma[0], gamma[1], ..., gamma[N] ]
     * b[i]: velocity^2
     * delta: 0 < b[i]-delta[i] < max_vel[i]*max_vel[i]
     * sigma: amin < a[i] - sigma[i] < amax
     * gamma: jerk_min/ref_vel[i] < pseudo_jerk[i] - gamma[i] < jerk_max/ref_vel[i]
     */

    const uint32_t l_variables = 5 * N;
    const uint32_t l_constraints = 4 * N + 1;

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(
            l_constraints, l_variables);  // the matrix size depends on constraint numbers.

    std::vector<double> lower_bound(l_constraints, 0.0);
    std::vector<double> upper_bound(l_constraints, 0.0);

    Eigen::MatrixXd P = Eigen::MatrixXd::Zero(l_variables, l_variables);
    std::vector<double> q(l_variables, 0.0);

    const double amax = param_.max_accel;
    const double amin = param_.min_decel;
    const double jmax = param_.max_jerk;
    const double jmin = param_.min_jerk;
    const double smooth_weight = param_.smooth_weight;
    const double over_j_weight = param_.over_j_weight;
    const double over_v_weight = param_.over_v_weight;
    const double over_a_weight = param_.over_a_weight;

    /**************************************************************/
    /**************************************************************/
    /**************** design objective function *******************/
    /**************************************************************/
    /**************************************************************/
    for (unsigned int i = 0; i < N; ++i)  // bi
        q[i] = -1.0;                      // |vmax^2 - b| -> minimize (-bi)

    // pseudo jerk: d(ai)/ds -> minimize weight * (a1 - a0)^2
    for (unsigned int i = N; i < 2 * N - 1; ++i)
    {
        const double w_x_dsinv = smooth_weight * (1.0 / ds);
        P(i, i) += w_x_dsinv;
        P(i, i + 1) -= w_x_dsinv;
        P(i + 1, i) -= w_x_dsinv;
        P(i + 1, i + 1) += w_x_dsinv;
    }

    for (unsigned int i = 2 * N; i < 3 * N; ++i)   // over velocity cost
        P(i, i) += over_v_weight;

    for (unsigned int i = 3 * N; i < 4 * N; ++i)   // over acceleration cost
        P(i, i) += over_a_weight;

    for (unsigned int i = 4 * N; i < 5 * N; ++i)   // over acceleration cost
        P(i, i) += over_j_weight;

    /**************************************************************/
    /**************************************************************/
    /**************** design constraint matrix ********************/
    /**************************************************************/
    /**************************************************************/
    // 0 < b - delta < vmax^2
    // NOTE: The delta allows b to be negative. This is actually invalid because the definition is b=v^2.
    // But mathematically, the strict b>0 constraint may make the problem infeasible, such as the case of
    // v=0 & a<0. To avoid the infeasibility, we allow b<0. The negative b is dealt as b=0 when it is
    // converted to v with sqrt. If the weight of delta^2 is large (the value of delta is very small),
    // b is almost 0, and is not a big problem.
    for (unsigned int i = 0; i < N; ++i) {
        const unsigned int j = 2 * N + i;
        A(i, i) = 1.0;   // b_i
        A(i, j) = -1.0;  // -delta_i
        upper_bound[i] = max_vel[i] * max_vel[i];
        lower_bound[i] = 0.0;
    }

    // amin < a - sigma < amax
    for (unsigned int i = N; i < 2 * N; ++i) {
        const unsigned int j = 2 * N + i;
        A(i, i) = 1.0;   // a_i
        A(i, j) = -1.0;  // -sigma_i
        if (i != N && max_vel[i - N] < std::numeric_limits<double>::epsilon()) {
            upper_bound[i] = 0.0;
            lower_bound[i] = 0.0;
        } else {
            upper_bound[i] = amax;
            lower_bound[i] = amin;
        }
    }

    // jerk_min/ref_vel[i] < pseudo_jerk[i] - gamma[i] < jerk_max/ref_vel[i]
    for(unsigned int i=2*N; i<3*N-1; ++i)
    {
        const unsigned int j = 2 * N + i;
        A(i, i-N)   =  1.0 / ds;  //  a[i] / ds
        A(i, i-N+1) = -1.0 / ds;  // -a[i+1] / ds
        upper_bound[i] = jmax / std::max(ref_vel[i-2*N], 1.0);
        lower_bound[i] = jmin / std::max(ref_vel[i-2*N], 1.0);
    }
    // temporary
    A(3*N, 2*N) = 1.0;
    upper_bound[3*N] = 0.0;
    lower_bound[3*N] = 0.0;

    // b' = 2a ... (b(i+1) - b(i)) / ds = 2a(i)
    for (unsigned int i = 3 * N; i < 4 * N - 1; ++i) {
        const unsigned int j = i - 3 * N;
        const double dsinv = 1.0 / std::max(ds, 0.0001);
        A(i, j) = -dsinv;     // b(i)
        A(i, j + 1) = dsinv;  // b(i+1)
        A(i, j + N) = -2.0;   // a(i)
        upper_bound[i] = 0.0;
        lower_bound[i] = 0.0;
    }

    // initial condition
    {
        const unsigned int i = 4 * N - 1;
        A(i, 0) = 1.0;  // b0
        upper_bound[i] = initial_vel * initial_vel;
        lower_bound[i] = initial_vel * initial_vel;

        A(i + 1, N) = 1.0;  // a0
        upper_bound[i + 1] = initial_acc;
        lower_bound[i + 1] = initial_acc;
    }

    /* execute optimization */
    const auto result = qp_solver_.optimize(P, A, q, lower_bound, upper_bound);

    // [b0, b1, ..., bN, |  a0, a1, ..., aN, | delta0, delta1, ..., deltaN, | sigma0, sigma1, ..., sigmaN |
    // gamma0, gamma1, ..., gammaN]
    const std::vector<double> optval = std::get<0>(result);

    qp_output.resize(N);
    for(unsigned int i=0; i<N; ++i)
    {
        qp_output.qp_velocity[i] = std::sqrt(std::max(optval.at(i), 0.0));
        qp_output.qp_acceleration[i] = optval.at(i+N);
    }

    for(unsigned int i=4*N; i<5*N; ++i)
    {
       std::cout << "gamma[" << i << "]: " << optval.at(i) << std::endl;
    }


    for(unsigned int i=0; i<N-1; ++i)
    {
        double a_current = qp_output.qp_acceleration[i];
        double a_next    = qp_output.qp_acceleration[i+1];
        qp_output.qp_jerk[i] = (a_next - a_current) * qp_output.qp_velocity[i] / ds;
    }
    qp_output.qp_jerk[N-1] = qp_output.qp_jerk[N-2];

    return true;
}