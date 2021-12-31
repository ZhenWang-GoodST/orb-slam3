#include "gsl_optimize.h"

#define N      10000    /* number of data points to fit */
#define TMAX   (100.0)  /* time variable in [0,TMAX] */

int expb_f (const gsl_vector * x, void *data, gsl_vector * f) {

    size_t n = ((struct GslData *)data)->data_dim;
    double *t = ((struct GslData *)data)->t;
    double *y = ((struct GslData *)data)->y;

    double a = gsl_vector_get (x, 0);
    double b = gsl_vector_get (x, 1);
    double c = gsl_vector_get (x, 2);

    size_t i;

    for (i = 0; i < n; i++) {
        /* Model Yi = a * x^2 + b* x + c */
        double Yi = a * t[i] * t[i] + t[i] * b + c;
        gsl_vector_set (f, i, Yi - y[i]);
    }

    return GSL_SUCCESS;
}

int expb_df (const gsl_vector * x, void *data, gsl_matrix * J) {

    size_t n = ((struct GslData *)data)->data_dim;
    double *t = ((struct GslData *)data)->t;
    double a = gsl_vector_get (x, 0);
    double b = gsl_vector_get (x, 1);
    for (size_t i = 0; i < n; i++) {
        /* Jacobian matrix J(i,j) = dfi / dxj, */
        /* where fi = (Yi - yi)/sigma[i],      */
        /*       Yi = a * x^2 + b* x + c  */
        /* and the xj are the parameters (A,lambda,b) */
        gsl_matrix_set (J, i, 0, 2 * t[i] * t[i]);
        gsl_matrix_set (J, i, 1, t[i]);
        gsl_matrix_set (J, i, 2, 1.0);
    }
    return GSL_SUCCESS;
}

void callback(const size_t iter, void *params,
        const gsl_multifit_nlinear_workspace *w) {
    gsl_vector *f = gsl_multifit_nlinear_residual(w);
    gsl_vector *x = gsl_multifit_nlinear_position(w);
    double rcond;

    /* compute reciprocal condition number of J(x) */
    gsl_multifit_nlinear_rcond(&rcond, w);

    fprintf(stderr, "iter %2zu: A = %.4f, lambda = %.4f, b = %.4f, cond(J) = %8.4f, |f(x)| = %.4f\n",
            iter,
            gsl_vector_get(x, 0),
            gsl_vector_get(x, 1),
            gsl_vector_get(x, 2),
            1.0 / rcond,
            gsl_blas_dnrm2(f));
}

void GslOptimizer::setFunction(int (* _function) (const gsl_vector * x, void * params, gsl_vector * f), 
                int (* _derivatives) (const gsl_vector * x, void * params, gsl_matrix * df),
                void (*_callback)(size_t iter, void *params, const gsl_multifit_nlinear_workspace *w)) {
    // this->fdf.f = _function;
    // this->fdf.df = _derivatives;
    // this->callback = _callback;
}
void GslOptimizer::loadData(int _itration) {
    itration = _itration;
}
void GslOptimizer::solve(GslData &data, FunctionPtr func, DerivativeFuncPtr derivfunc, CallBackFuncPtr callbackfunc) {
    const gsl_multifit_nlinear_type *T = gsl_multifit_nlinear_trust;
    gsl_multifit_nlinear_workspace *workspace;
    gsl_multifit_nlinear_fdf fdf;
    gsl_multifit_nlinear_parameters fdf_params = gsl_multifit_nlinear_default_parameters();
    gsl_vector *cost_func;
    gsl_matrix *J;
    gsl_matrix *covar = gsl_matrix_alloc (data.para_dim, data.para_dim);
    // double t[N], y[N], weights[N];
    // struct GslData data = { data_dim, t, y };
    double paramter_init[3] = { 3.0, 1.5, 100.0 }; /* starting values */
    gsl_vector_view paramter = gsl_vector_view_array (paramter_init, data.para_dim);
    gsl_vector_view wts = gsl_vector_view_array(data.weights, data.data_dim);
    double chisq, chisq0;
    int status, info;

    const double xtol = 1e-8;
    const double gtol = 1e-8;
    const double ftol = 0.0;
    /* define the function to be minimized */
    fdf.f = func;
    fdf.df = derivfunc;   /* set to NULL for finite-difference Jacobian 有限差分法 */
    fdf.fvv = NULL;     /* not using geodesic acceleration */
    fdf.n = data.data_dim;
    fdf.p = data.para_dim;
    fdf.params = &data;
    /* allocate workspace with default parameters */
    workspace = gsl_multifit_nlinear_alloc (T, &fdf_params, data.data_dim, data.para_dim);
    /* initialize solver with starting point and weights */
    gsl_multifit_nlinear_winit (&paramter.vector, &wts.vector, &fdf, workspace);
    /* compute initial cost function */
    cost_func = gsl_multifit_nlinear_residual(workspace);
    gsl_blas_ddot(cost_func, cost_func, &chisq0);
    /* solve the system with a maximum of 100 iterations */
    status = gsl_multifit_nlinear_driver(itration, xtol, gtol, ftol,
                                        callbackfunc, NULL, &info, workspace);
    /* compute covariance of best fit parameters */
    J = gsl_multifit_nlinear_jac(workspace);
    gsl_multifit_nlinear_covar (J, 0.0, covar);
    /* compute final cost */
    gsl_blas_ddot(cost_func, cost_func, &chisq);
    for (size_t i = 0; i < data.para_dim; ++i) {
        data.paras[i] = gsl_vector_get(workspace->x, i);
    }
    
    #define FIT(i) gsl_vector_get(workspace->x, i)
    #define ERR(i) sqrt(gsl_matrix_get(covar,i,i))
    fprintf(stderr, "summary from method '%s/%s'\n",
            gsl_multifit_nlinear_name(workspace),
            gsl_multifit_nlinear_trs_name(workspace));
    fprintf(stderr, "number of iterations: %zu\n",
            gsl_multifit_nlinear_niter(workspace));
    fprintf(stderr, "function evaluations: %zu\n", fdf.nevalf);
    fprintf(stderr, "Jacobian evaluations: %zu\n", fdf.nevaldf);
    fprintf(stderr, "reason for stopping: %s\n",
            (info == 1) ? "small step size" : "small gradient");
    fprintf(stderr, "initial |f(x)| = %f\n", sqrt(chisq0));
    fprintf(stderr, "final   |f(x)| = %f\n", sqrt(chisq));

    {
        double dof = data.data_dim - data.para_dim;
        double c = GSL_MAX_DBL(1, sqrt(chisq / dof));

        fprintf(stderr, "chisq/dof = %g\n", chisq / dof);

        fprintf (stderr, "a      = %.5f +/- %.5f\n", FIT(0), c*ERR(0));
        fprintf (stderr, "b      = %.5f +/- %.5f\n", FIT(1), c*ERR(1));
        fprintf (stderr, "c      = %.5f +/- %.5f\n", FIT(2), c*ERR(2));
    }
    fprintf (stderr, "status = %s\n", gsl_strerror (status));
    gsl_multifit_nlinear_free (workspace);
    gsl_matrix_free (covar);
}    

GslOptimizer::GslOptimizer(/* args */) {
    // this->fdf.f = expb_f;
    // this->fdf.df = expb_df;
    // this->callback = callback;
}