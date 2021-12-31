#include <stdlib.h>
#include <stdio.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_vector.h>
#include <gsl/gsl_blas.h>
#include <gsl/gsl_multifit_nlinear.h>

struct GslData {
  size_t data_dim;
  size_t para_dim;
  double * t;
  double * y;
  double * weights;
  double * paras;
};

typedef int (*FunctionPtr)(const gsl_vector *x, void *params, gsl_vector *f);
typedef int (*DerivativeFuncPtr) (const gsl_vector * x, void * params, gsl_matrix * df);
typedef void (*CallBackFuncPtr)(size_t iter, void *params, const gsl_multifit_nlinear_workspace *w);

int expb_f (const gsl_vector * x, void *data, gsl_vector * f);

int expb_df (const gsl_vector * x, void *data, gsl_matrix * J);

void callback(const size_t iter, void *params,
        const gsl_multifit_nlinear_workspace *w);

class GslOptimizer {
private:
    int itration = 10;
public:
    GslOptimizer(/* args */);
    ~GslOptimizer() {};
    void setFunction(FunctionPtr, DerivativeFuncPtr, CallBackFuncPtr);
    void loadData(int _itration) ;
    void solve(GslData &data, FunctionPtr func, DerivativeFuncPtr derivfunc, CallBackFuncPtr callbackfunc) ;   
};